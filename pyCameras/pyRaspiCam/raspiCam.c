


// We use some GNU extensions (basename)
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <sysexits.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#define VERSION_STRING "v1.3.5"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"


#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"


#include <semaphore.h>
#include "raspiCam.h"

//#include "RaspiStillYUV.c"

// Stills format information
// 0 implies variable
#define STILLS_FRAME_RATE_NUM 0
#define STILLS_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3



/** Structure containing all state information for the current run
 */
typedef struct
{
   int width;                          /// Requested width of image
   int height;                         /// requested height of image
   int verbose;                        /// !0 if want detailed run information
   int fullResPreview;                 /// If set, the camera preview port runs at capture resolution. Reduces fps.
   int frameNextMethod;                /// Which method to use to advance to next frame
   int settings;                       /// Request settings from the camera
   int cameraNum;                      /// Camera number
   int burstCaptureMode;               /// Enable burst mode
   int format;							/// image format e.g. FORMAT_RAW, FORMAT_RGB, FORMAT_YUV_Y
   RASPIPREVIEW_PARAMETERS preview_parameters;    /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters
   int flash; 			/// Flash mode flash;					// iff flash enabled
   int waitFlash;
   int bufNum;
   int sensor_mode;
   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *null_sink_component;    /// Pointer to the camera component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
   MMAL_POOL_T *camera_pool;              /// Pointer to the pool of buffers used by camera stills port
} RASPISTILLYUV_STATE;

RASPISTILLYUV_STATE state;

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
   MMAL_STATUS_T status;

   status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

   if (status == MMAL_SUCCESS)
   {
      status =  mmal_connection_enable(*connection);
      if (status != MMAL_SUCCESS)
         mmal_connection_destroy(*connection);
   }

   return status;
}





/** Struct used to pass information in camera still port userdata to callback
 */
typedef struct
{
	unsigned short *arrayBuffer;                   /// buffer to result array
	VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
	RASPISTILLYUV_STATE *pstate;            /// pointer to our state in case required in callback
} PORT_USERDATA_BUFFER;
MMAL_PORT_T *preview_input_port = NULL;
MMAL_PORT_T *camera_preview_port = NULL;
MMAL_PORT_T *camera_video_port = NULL;
MMAL_PORT_T *camera_still_port = NULL;
PORT_USERDATA_BUFFER callback_data;

int mmal_status_to_int(MMAL_STATUS_T status);

/**
 *  buffer header callback function for camera control
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void camera_control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   fprintf(stderr, "Camera control callback  cmd=0x%08x", buffer->cmd);

   if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED)
   {
      MMAL_EVENT_PARAMETER_CHANGED_T *param = (MMAL_EVENT_PARAMETER_CHANGED_T *)buffer->data;
      switch (param->hdr.id) {
         case MMAL_PARAMETER_CAMERA_SETTINGS:
         {
            MMAL_PARAMETER_CAMERA_SETTINGS_T *settings = (MMAL_PARAMETER_CAMERA_SETTINGS_T*)param;
            vcos_log_error("Exposure now %u, analog gain %u/%u, digital gain %u/%u",
			settings->exposure,
                        settings->analog_gain.num, settings->analog_gain.den,
                        settings->digital_gain.num, settings->digital_gain.den);
            vcos_log_error("AWB R=%u/%u, B=%u/%u",
                        settings->awb_red_gain.num, settings->awb_red_gain.den,
                        settings->awb_blue_gain.num, settings->awb_blue_gain.den
                        );
         }
         break;
      }
   }
   else if (buffer->cmd == MMAL_EVENT_ERROR)
   {
      vcos_log_error("No data received from sensor. Check all connections, including the Sunny one on the camera board");
   }
   else
   {
      vcos_log_error("Received unexpected camera control callback event, 0x%08x", buffer->cmd);
   }

   mmal_buffer_header_release(buffer);
}



/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_STATUS_T create_camera_component(RASPISTILLYUV_STATE *state)
{
   MMAL_COMPONENT_T *camera = 0;
   MMAL_ES_FORMAT_T *format;
   MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   /* Create the component */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create camera component");
      goto error;
   }

   MMAL_PARAMETER_INT32_T camera_num =
      {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->cameraNum};

   status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not select camera : error %d", status);
      goto error;
   }

   if (!camera->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Camera doesn't have output ports");
      goto error;
   }

   status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->sensor_mode);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set sensor mode : error %d", status);
      goto error;
   }

   preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
   video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
   still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

   if (state->settings)
   {
      MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T change_event_request =
         {{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T)},
          MMAL_PARAMETER_CAMERA_SETTINGS, 1};

      status = mmal_port_parameter_set(camera->control, &change_event_request.hdr);
      if ( status != MMAL_SUCCESS )
      {
         vcos_log_error("No camera settings events");
      }
   }

/*   MMAL_PARAMETER_FLASH_T camera_flash =
     {{MMAL_PARAMETER_FLASH, sizeof(MMAL_PARAMETER_FLASH_T)}, state->flash};
   status = mmal_port_parameter_set(camera->control, &camera_flash.hdr);

   if (status != MMAL_SUCCESS )
      {
         vcos_log_error("Unable to enable flash : error %d", status);
         goto error;
      }*/

   // Enable the camera, and tell it its control callback function
   status = mmal_port_enable(camera->control, camera_control_callback);

   if (status != MMAL_SUCCESS )
   {
      vcos_log_error("Unable to enable control port : error %d", status);
      goto error;
   }

   //  set up the camera configuration
   {
      MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
      {
         { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
         .max_stills_w = state->width,
         .max_stills_h = state->height,
         .stills_yuv422 = 1,
         .one_shot_stills = 1,
         .max_preview_video_w = state->preview_parameters.previewWindow.width,
         .max_preview_video_h = state->preview_parameters.previewWindow.height,
         .num_preview_video_frames = 1,
         .stills_capture_circular_buffer_height = 0,
         .fast_preview_resume = 1,
         .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
      };

      mmal_port_parameter_set(camera->control, &cam_config.hdr);
   }

   raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

   // Now set up the port formats

   format = preview_port->format;


   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                                     { 50, 1000 }, {166, 1000}};
        mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                                     { 166, 1000 }, {999, 1000}};
        mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }
   if (state->fullResPreview)
   {
      // In this mode we are forcing the preview to be generated from the full capture resolution.
      // This runs at a max of 15fps with the OV5647 sensor.
      format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
      format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
      format->es->video.crop.x = 0;
      format->es->video.crop.y = 0;
      format->es->video.crop.width = state->width;
      format->es->video.crop.height = state->height;
      format->es->video.frame_rate.num = FULL_RES_PREVIEW_FRAME_RATE_NUM;
      format->es->video.frame_rate.den = FULL_RES_PREVIEW_FRAME_RATE_DEN;
   }
   else
   {
      // Use a full FOV 4:3 mode
      format->es->video.width = VCOS_ALIGN_UP(state->preview_parameters.previewWindow.width, 32);
      format->es->video.height = VCOS_ALIGN_UP(state->preview_parameters.previewWindow.height, 16);
      format->es->video.crop.x = 0;
      format->es->video.crop.y = 0;
      format->es->video.crop.width = state->preview_parameters.previewWindow.width;
      format->es->video.crop.height = state->preview_parameters.previewWindow.height;
      format->es->video.frame_rate.num = PREVIEW_FRAME_RATE_NUM;
      format->es->video.frame_rate.den = PREVIEW_FRAME_RATE_DEN;
   }

   status = mmal_port_format_commit(preview_port);

   if (status != MMAL_SUCCESS )
   {
      vcos_log_error("camera viewfinder format couldn't be set");
      goto error;
   }

   // Set the same format on the video  port (which we dont use here)
   mmal_format_full_copy(video_port->format, format);
   status = mmal_port_format_commit(video_port);

   if (status != MMAL_SUCCESS )
   {
      vcos_log_error("camera video format couldn't be set");
      goto error;
   }

   // Ensure there are enough buffers to avoid dropping frames
   if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   format = still_port->format;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                                     { 50, 1000 }, {166, 1000}};
        mmal_port_parameter_set(still_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                                     { 167, 1000 }, {999, 1000}};
        mmal_port_parameter_set(still_port, &fps_range.hdr);
   }
   // Set our stills format on the stills  port
   if (state->format == FORMAT_RGB)
   {
      format->encoding = MMAL_ENCODING_BGR24;
      format->encoding_variant = MMAL_ENCODING_BGR24;
   }
   else if (state->format == FORMAT_YUV_Y)
   {
      format->encoding = MMAL_ENCODING_I420;
      format->encoding_variant = MMAL_ENCODING_I420;
   }
   else{
	   printf("Unknown format %d\n", state->format);
	   return 1;
   }
   format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->width;
   format->es->video.crop.height = state->height;
   format->es->video.frame_rate.num = STILLS_FRAME_RATE_NUM;
   format->es->video.frame_rate.den = STILLS_FRAME_RATE_DEN;

   if (still_port->buffer_size < still_port->buffer_size_min)
      still_port->buffer_size = still_port->buffer_size_min;

   still_port->buffer_num = state->bufNum;



   status = mmal_port_format_commit(still_port);
   if (status != MMAL_SUCCESS )
   {
	   printf("%d\n", status);
      vcos_log_error("camera still format couldn't be set");
      goto error;
   }

   /* Enable component */
   status = mmal_component_enable(camera);

   if (status != MMAL_SUCCESS )
   {
      vcos_log_error("camera component couldn't be enabled");
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(still_port, still_port->buffer_num, still_port->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for camera still port %s", still_port->name);
   }

   state->camera_pool = pool;
   state->camera_component = camera;

   if (state->verbose)
      fprintf(stderr, "Camera component done\n");

   return status;

error:

   if (camera)
      mmal_component_destroy(camera);

   return status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPISTILLYUV_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}

/**
 * Handler for sigint signals
 *
 * @param signal_number ID of incoming signal.
 *
 */
static void signal_handler(int signal_number)
{
   if (signal_number == SIGUSR1)
   {
      // Handle but ignore - prevents us dropping out if started in none-signal mode
      // and someone sends us the USR1 signal anyway
   }
   else
   {
      // Going to abort on all other signals
      vcos_log_error("Aborting program\n");
      exit(130);
   }
}

/**
 * Assign set of default parameters to the passed in parameter block
 *
 * @param state Pointer to parameter block
 *
 */
void raspipreview_set_measurement(RASPIPREVIEW_PARAMETERS *state)
{
   state->wantPreview = 0;
   state->wantFullScreenPreview = 0;
   state->opacity = 255;
   state->previewWindow.x = 0;
   state->previewWindow.y = 0;
   //state->previewWindow.width = 1024;
   state->previewWindow.width = 512;
   //state->previewWindow.height = 768;
   state->previewWindow.height = 384;
   state->preview_component = NULL;
}

/**
 * Give the supplied parameter block a set of default values
 * @params Pointer to parameter block
 */
void raspicamcontrol_set_measurement(RASPICAM_CAMERA_PARAMETERS *params)
{
   vcos_assert(params);

   params->sharpness = 0;
   params->contrast = 0;
   params->brightness = 50;
   params->saturation = 0;
   params->ISO = 100;                    // 0 = auto
   params->videoStabilisation = 0;
   params->exposureCompensation = 0;
   params->exposureMode = MMAL_PARAM_EXPOSUREMODE_OFF;
//   params->exposureMode = MMAL_PARAM_EXPOSUREMODE_AUTO;
//   params->exposureMeterMode = MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
   params->awbMode = MMAL_PARAM_AWBMODE_OFF;
//   params->awbMode = MMAL_PARAM_AWBMODE_AUTO;
   params->imageEffect = MMAL_PARAM_IMAGEFX_NONE;
   params->colourEffects.enable = 0;
   params->colourEffects.u = 128;
   params->colourEffects.v = 128;
   params->rotation = 0;
   params->hflip = params->vflip = 0;
   params->roi.x = params->roi.y = 0.0;
   params->roi.w = params->roi.h = 1.0;
   params->shutter_speed = 10000;          // 0 = auto
   params->awb_gains_r = 0;      // Only have any function if AWB OFF is used.
   params->awb_gains_b = 0;
   params->drc_level = MMAL_PARAMETER_DRC_STRENGTH_OFF;
   params->stats_pass = MMAL_FALSE;
   params->enable_annotate = 0;
   params->annotate_string[0] = '\0';
   params->annotate_text_size = 0;	//Use firmware default
   params->annotate_text_colour = -1;   //Use firmware default
   params->annotate_bg_colour = -1;     //Use firmware default
   params->stereo_mode.mode = MMAL_STEREOSCOPIC_MODE_NONE;
   params->stereo_mode.decimate = MMAL_FALSE;
   params->stereo_mode.swap_eyes = MMAL_FALSE;
}

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
void measurement_status(RASPISTILLYUV_STATE *state, int format)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   // Default everything to zero
   memset(state, 0, sizeof(RASPISTILLYUV_STATE));

   // Now set anything non-zero
   state->width = 2592;
   state->height = 1944;
   state->verbose = 0;
   state->frameNextMethod = FRAME_NEXT_SINGLE;
   state->settings = 0;
   state->burstCaptureMode=0;
   state->format = format;
   state->flash = FLASH_OFF;
   state->bufNum = 5;
   state->sensor_mode = 0;
   raspipreview_set_measurement(&state->preview_parameters);

   // Set up the camera_parameters to default
   raspicamcontrol_set_measurement(&state->camera_parameters);

   // Set default camera
   state->cameraNum = 0;
}



/**
 *  buffer header callback function for camera output port
 *
 *  Callback will save buffer for further analysis
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void camera_buffer_callback_buffer(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{

	int complete = 0;
	int i;
	PORT_USERDATA_BUFFER *pData = (PORT_USERDATA_BUFFER *)port->userdata;
	if (buffer->length){
            //printf("buflen: %d \n", buffer->length);
            int copyLen = 2592*1944;
            if (pData->pstate->format == FORMAT_RGB)
                copyLen *= 3;
	    if (state.sensor_mode == 4){
            	copyLen = (1296+16)*(972);
            }
            //printf("copying %d bytes\n", copyLen);

	    mmal_buffer_header_mem_lock(buffer);
	    memcpy(pData->arrayBuffer, buffer->data, copyLen);
	    mmal_buffer_header_mem_unlock(buffer);
	    if (state.flash != FLASH_OFF)
	    		bcm2835_gpio_write(state.flash, LOW);
	}

	if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
		 complete = 1;

	mmal_buffer_header_release(buffer);
	// and send one back to the port (if still open)
	if (port->is_enabled)
	{
		MMAL_STATUS_T status;
		MMAL_BUFFER_HEADER_T *new_buffer = mmal_queue_get(pData->pstate->camera_pool->queue);

		// and back to the port from there.
		if (new_buffer)
		{
			status = mmal_port_send_buffer(port, new_buffer);
		}

		if (!new_buffer || status != MMAL_SUCCESS)
			printf("Unable to return the buffer to the camera still port\n");
	}

	if (complete)
   {
		//printf("releasing samphore %d\n", pData->num);
		vcos_semaphore_post(&(pData->complete_semaphore));
   }

}

#pragma GCC push_options
#pragma GCC optimize ("O0")

int Sleep(int ms){
    clock_t t0 = clock();
    clock_t end = t0+((float)ms/1000.0);
    //kind'a active wait
    int result = 0;
    while(clock() < end) {
        result++;
    }
    return result;
}

#pragma GCC pop_options

PyObject* getImageBuffer(unsigned char* seq, int len, unsigned int shutter){


}

int getImage(unsigned char* seq, int n1, int n2, int n3, unsigned int 
shutter, int delay){


	//if (n1 != 1944 || n2 != 2592)
	//	return -1;


	state.camera_parameters.shutter_speed = shutter;
	if (mmal_status_to_int(mmal_port_parameter_set_uint32(state.camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, state.camera_parameters.shutter_speed) != MMAL_SUCCESS)){
		printf("Unable to set shutter speed\n");
	}



	callback_data.arrayBuffer = seq;


	// tell mmal to capture an image
	if (state.flash != FLASH_OFF){
//                int delay = 1000000;
                printf("delay: %d\n", delay);
		bcm2835_gpio_write(state.flash, HIGH);
                bcm2835_delay(delay);
		bcm2835_gpio_write(state.flash, LOW);
        }

	if (mmal_port_parameter_set_boolean(camera_still_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
	{
		printf("%s: Failed to start capture\n", __func__);
		return -1;
	}
//	if (state.flash != FLASH_OFF){
//		bcm2835_gpio_write(state.flash, LOW);
//	}


	// printf("Waiting for callback %d\n", shutter);
	// Wait for capture to complete
	// For some reason using vcos_semaphore_wait_timeout sometimes returns immediately with bad parameter error
	// even though it appears to be all correct, so reverting to untimed one until figure out why its erratic
	vcos_semaphore_wait(&callback_data.complete_semaphore);

	return 0;
}




int initialize(int format, int flash, int waitFlash, int binning){

	MMAL_STATUS_T status = MMAL_SUCCESS;


	// port to capture an image
	//MMAL_PORT_T *preview_input_port = NULL;

	// dunno
	bcm_host_init();

	// dunno either
	signal(SIGINT, signal_handler);

	// Disable USR1 for the moment - may be reenabled if go in to signal capture mode
	signal(SIGUSR1, SIG_IGN);

	measurement_status(&state, format);

	state.flash = flash;
	state.waitFlash = waitFlash;
        if (binning == 2){
            state.sensor_mode = 4;
	    state.width = 1296;
            state.height = 972;
        } else if (binning != 0){
            printf("WARNING, binning must be 0 or 2\n");
	    return 1;
	}

	if (flash != FLASH_OFF){
		if (!bcm2835_init())
		        return 1;
		    // Set the pin to be an output
		bcm2835_gpio_fsel(flash, BCM2835_GPIO_FSEL_OUTP);
		    // Blink

		bcm2835_gpio_write(flash, HIGH);
		bcm2835_delay(500);
		bcm2835_gpio_write(flash, LOW);
		bcm2835_delay(500);
		bcm2835_gpio_write(flash, HIGH);
		bcm2835_delay(500);
		bcm2835_gpio_write(flash, LOW);
	}


	if ((status = create_camera_component(&state)) != MMAL_SUCCESS)
	{
		//vcos_log_error("%s: Failed to create camera component", __func__);
		printf("Error creating cam! line: %d\n", __LINE__);
		return -1;
	}


	if ((status = raspipreview_create(&state.preview_parameters)) != MMAL_SUCCESS)
	{
		printf("Error creating preview! line: %d\n", __LINE__);
		return -1;
	}

	camera_preview_port = state.camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
	camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
	camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
	// Note we are lucky that the preview and null sink components use the same input port
	// so we can simple do this without conditionals
	preview_input_port  = state.preview_parameters.preview_component->input[0];

	// Connect camera to preview (which might be a null_sink if no preview required)
//	if (format == FORMAT_RGB)
//		status = connect_ports(camera_preview_port, preview_input_port, &state.preview_connection);
//	if (mmal_port_parameter_set_boolean(camera_still_port, MMAL_PARAMETER_ENABLE_RAW_CAPTURE, 1) != MMAL_SUCCESS)
//		{
//			vcos_log_error("INIT RAW was requested, but failed to enable");
//		}


	if (status != MMAL_SUCCESS){
		mmal_status_to_int(status);
		return 1;
	}

	VCOS_STATUS_T vcos_status;


	// Set up our userdata - this is passed though to the callback where we need the information.
	// Null until we open our filename
	//callback_data.file_handle = NULL;
	callback_data.pstate = &state;

	vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0);
	vcos_assert(vcos_status == VCOS_SUCCESS);

	camera_still_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;

	if (state.verbose)
		fprintf(stderr, "Enabling camera still output port\n");

	// Enable the camera still output port and tell it its callback function
	status = mmal_port_enable(camera_still_port, camera_buffer_callback_buffer);

	if (status != MMAL_SUCCESS)
	{
		printf("Failed to setup camera output! line: %d\n", __LINE__);
		return 1;
	}



	int num,q;

	num = mmal_queue_length(state.camera_pool->queue);

	printf("Adding %d buffers\n", num);

	for (q=0;q<num;q++)
	{
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.camera_pool->queue);

		if (!buffer)
			printf("Unable to get a required buffer %d from pool queue\n", q);

		if (mmal_port_send_buffer(camera_still_port, buffer)!= MMAL_SUCCESS)
			printf("Unable to send a buffer to camera output port (%d)\n", q);
	}
	mmal_port_parameter_set_boolean(state.camera_component->control,  MMAL_PARAMETER_CAMERA_BURST_CAPTURE, 1);
	return 0;
}

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port)
{
   if (port && port->is_enabled)
      mmal_port_disable(port);
}

void setFlash(int flash){
    state.flash = flash;
}

int finalize(void){



	if (state.verbose)
		fprintf(stderr, "Closing down\n");

	// Disable all our ports that are not handled by connections
	check_disable_port(camera_video_port);

	if (state.preview_connection)
		mmal_connection_destroy(state.preview_connection);

	/* Disable components */
	if (state.preview_parameters.preview_component)
		mmal_component_disable(state.preview_parameters.preview_component);

	if (state.camera_component)
		mmal_component_disable(state.camera_component);

	raspipreview_destroy(&state.preview_parameters);
	destroy_camera_component(&state);

	if (state.verbose)
		fprintf(stderr, "Close down completed, all components disconnected, disabled and destroyed\n\n");
	if (state.flash != FLASH_OFF)
		bcm2835_close();
	return 0;
}

