#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <lcm/lcm-cpp.hpp>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "eecs467_util.h"    // This is where a lot of the internals live

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "OccupancyGridGuiHandler.hpp"

#define SCREEN_WIDTH 100
#define SCREEN_HEIGHT 100

using namespace std;

// It's good form for every application to keep its state in a struct.
typedef struct state state_t;
struct state {
    bool running;

    getopt_t        *gopt;
    parameter_gui_t *pg;

    // image stuff
    char *img_url;
    int   img_height;
    int   img_width;

    // vx stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;      // where vx objects are live
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    vx_mouse_event_t    last_mouse_event;

    // threads stuff
    pthread_t animate_thread;
    pthread_t lcm_thread;
    
    // LCM stuff
    lcm::LCM *lcm;
    pthread_mutex_t lcm_mutex;

    // for accessing the arrays stuff
    pthread_mutex_t mutex;

    // occupancy grid stuff
    eecs467::OccupancyGrid grid;
};

// === Parameter listener =================================================
// This function is handed to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// state, etc if need be.
static void
my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    if (0==strcmp ("sl1", name))
        printf ("sl1 = %f\n", pg_gd (pg, name));
    else if (0==strcmp ("sl2", name))
        printf ("sl2 = %d\n", pg_gi (pg, name));
    else if (0==strcmp ("cb1", name) || 0==strcmp ("cb2", name))
        printf ("%s = %d\n", name, pg_gb (pg, name));
    else
        printf ("%s changed\n", name);
}

static int
mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    state_t *state = (state_t*)vxeh->impl;

    // vx_camera_pos_t contains camera location, field of view, etc
    // vx_mouse_event_t contains scroll, x/y, and button click events

    if ((mouse->button_mask & VX_BUTTON1_MASK) &&
        !(state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {

        vx_ray3_t ray;
        vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

        double ground[3];
        vx_ray3_intersect_xy (&ray, 0, ground);

        printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Ground clicked at coords: [%6.3f, %6.3f]\n",
                mouse->x, mouse->y, ground[0], ground[1]);
    }

    // store previous mouse event to see if the user *just* clicked or released
    state->last_mouse_event = *mouse;

    return 0;
}

static int
key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
{
    //state_t *state = vxeh->impl;
    return 0;
}

static int
touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0; // Does nothing
}

//Converts logodds [-128, 127] to a grayscale vaue [0, 255]
static int to_grayscale(int a)
{
	return a + 128;
}

void *
lcm_handle_thread (void *data)
{
    state_t *state = (state_t*)data;
    while(1) state->lcm->handle();
}

// === Your code goes here ================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by state
void *
animate_thread (void *data)
{
    const int fps = 60;
    state_t *state = (state_t*)data;

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (state->running) 
    {
		//cout << "Animate Thread!" << endl;
		image_u32_t *im = image_u32_create(10, 10);
		for(int y = 0; y < im->height; y++)
		{
			for(int x = 0; x < im->width; x++)
			{
				int a = 255; //alpha transparency value.
				int v = to_grayscale(state->grid.logOdds(x, y)) % 255;
				im->buf[y*im->stride+x] = (a<<24) + (v<<16) + (v<<8) + (v<<0);
			}
		}
		vx_object_t * vo = vxo_image_from_u32(im, VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER);
		const double scale = 1./im->width;
		                vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "bitmap"),
		                                    vxo_chain (vxo_mat_scale3 (scale, scale, 1.0),
		                                               vxo_mat_translate3 (-im->width/2., -im->height/2., 0.),
		                                               vo));
		vx_buffer_swap (vx_world_get_buffer (state->vxworld, "bitmap"));
		image_u32_destroy (im);
		vx_buffer_swap (vx_world_get_buffer (state->vxworld, "axes"));

		usleep (1000000/fps);
    }

    return NULL;
}

//void * 
//receive_lcm_msg (void *data)
//{
//    state_t *state = (state_t *)data;
//
//    while(1)
//    
//    return NULL;
//}

state_t *
state_create (void)
{
    state_t *state = new state_t;

    state->vxworld = vx_world_create ();
    state->vxeh = new vx_event_handler_t;
    state->vxeh->key_event = key_event;
    state->vxeh->mouse_event = mouse_event;
    state->vxeh->touch_event = touch_event;
    state->vxeh->dispatch_order = 100;
    state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

    state->vxapp.display_started = eecs467_default_display_started;
    state->vxapp.display_finished = eecs467_default_display_finished;
    state->vxapp.impl = eecs467_default_implementation_create (state->vxworld, state->vxeh);

    state->running = 1;

    return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;

    if (state->vxeh)
        free (state->vxeh);

    if (state->gopt)
        getopt_destroy (state->gopt);

    if (state->pg)
        pg_destroy (state->pg);

    free (state);
}

// This is intended to give you a starting point to work with for any program
// requiring a GUI. This handles all of the GTK and vx setup, allowing you to
// fill in the functionality with your own code.
int
main (int argc, char *argv[])
{
    eecs467_init (argc, argv);
    state_t *state = state_create ();
    
    OccupancyGridGuiHandler gui_handler(&state->grid);
    state->lcm = new lcm::LCM;
    pthread_mutex_init(&state->lcm_mutex, NULL);
    state->lcm->subscribe("OCCUPANCY_GRID_GUI",
                          &OccupancyGridGuiHandler::handleMessage,
                          &gui_handler);

    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt,  'h', "help", 0, "Show help");
    getopt_add_string (state->gopt, '\0', "url", "", "Camera URL");
    getopt_add_bool   (state->gopt,  'l', "list", 0, "Lists available camera URLs and exit");

    if (!getopt_parse (state->gopt, argc, argv, 1) || getopt_get_bool (state->gopt, "help")) {
        printf ("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }

    // Set up the imagesource. This looks for a camera url specified on
    // the command line and, if none is found, enumerates a list of all
    // cameras imagesource can find and picks the first url it finds.
    if (strncmp (getopt_get_string (state->gopt, "url"), "", 1)) {
        state->img_url = strdup (getopt_get_string (state->gopt, "url"));
        printf ("URL: %s\n", state->img_url);
    }
    else {
        // No URL specified. Show all available and then use the first
        zarray_t *urls = image_source_enumerate ();
        printf ("Cameras:\n");
        for (int i = 0; i < zarray_size (urls); i++) {
            char *url;
            zarray_get (urls, i, &url);
            printf ("  %3d: %s\n", i, url);
        }

        if (0==zarray_size (urls)) {
            printf ("Found no cameras.\n");
            return -1;
        }

        zarray_get (urls, 0, &state->img_url);
    }

    if (getopt_get_bool (state->gopt, "list")) {
        state_destroy (state);
        exit (EXIT_SUCCESS);
    }

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    // Initialize a parameter gui
    state->pg = pg_create ();

    parameter_listener_t *my_listener = new parameter_listener_t;
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener (state->pg, my_listener);

    // Launch our worker threads
    pthread_create (&state->animate_thread, NULL, animate_thread, (void*)state);
    pthread_create (&state->lcm_thread, NULL, lcm_handle_thread, (void*)state);

    // This is the main loop
    eecs467_gui_run (&state->vxapp, state->pg, SCREEN_WIDTH, SCREEN_HEIGHT);

    // Quit when GTK closes
    state->running = 0;
    pthread_join (state->animate_thread, NULL);
    
    
    // Cleanup
    free (my_listener);
    state_destroy (state);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();
}
