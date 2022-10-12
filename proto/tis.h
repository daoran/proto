#ifndef TIS_H
#define TIS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <gst/gst.h>
#include <gst/video/video.h>

#define SDL_DISABLE_IMMINTRIN_H 1
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

typedef struct viz_t {
  SDL_Window *window;
  SDL_Renderer *renderer;
  SDL_Texture *texture;

  int win_width;
  int win_height;
  int loop;
  GstClockTime last_ts;
} viz_t;

void viz_setup(viz_t *viz,
               const char *win_title,
               const int win_w,
               const int win_h);
void viz_free(viz_t *viz);
void viz_update(viz_t *viz, const uint8_t *img_data, const int pitch);
void viz_loop(viz_t *viz);
void bgr2csv(const GstMapInfo *frame);

typedef struct tis_t {
  int cam_idx;
  const char *serial;
  GstElement *pipeline;
  viz_t *viz;
} tis_t;

void gst_setup(int argc, char *argv[]);
GstFlowReturn tis_callback(GstElement *sink, void *user_data);
int tis_setup(tis_t *cam, const int cam_idx, const char *serial, viz_t *viz);
void tis_run(tis_t *cam);
void tis_cleanup(tis_t *cam);

#endif // TIS_H

//////////////////////////////////////////////////////////////////////////////
//                             IMPLEMENTATION                               //
//////////////////////////////////////////////////////////////////////////////

#ifdef TIS_IMPLEMENTATION

/** Setup visualizer **/
void viz_setup(viz_t *viz,
               const char *win_title,
               const int win_w,
               const int win_h) {
  // SDL init
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    printf("SDL_Init Error: %s/n", SDL_GetError());
  }

  // Calculate window x and y position
  SDL_DisplayMode disp_mode;
  SDL_GetCurrentDisplayMode(0, &disp_mode);
  const int disp_w = disp_mode.w;
  const int disp_h = disp_mode.h;
  const int win_x = disp_w / 2 - win_w / 2;
  const int win_y = disp_h / 2 - win_h / 2;

  // Setup window
  viz->window =
      SDL_CreateWindow(win_title, win_x, win_y, win_w, win_h, SDL_WINDOW_SHOWN);
  viz->renderer = SDL_CreateRenderer(viz->window, -1, SDL_RENDERER_ACCELERATED);
  viz->texture = SDL_CreateTexture(viz->renderer,
                                   SDL_PIXELFORMAT_BGRA32,
                                   SDL_TEXTUREACCESS_STREAMING,
                                   win_w,
                                   win_h);

  // Set window properties
  viz->win_width = win_w;
  viz->win_height = win_h;
  viz->loop = 1;
  viz->last_ts = 0;
}

/** Free visualizer **/
void viz_free(viz_t *viz) {
  SDL_DestroyTexture(viz->texture);
  SDL_DestroyRenderer(viz->renderer);
  SDL_DestroyWindow(viz->window);
  SDL_Quit();
}

/** Updat visualizer **/
void viz_update(viz_t *viz, const uint8_t *img_data, const int pitch) {
  SDL_UpdateTexture(viz->texture, NULL, img_data, pitch);
  SDL_RenderCopy(viz->renderer, viz->texture, NULL, NULL);
  SDL_RenderPresent(viz->renderer);
}

/** Start event loop visualizer **/
void viz_loop(viz_t *viz) {
  // Wait
  sleep(1);

  // SDL event loop
  SDL_Event event;
  while (SDL_PollEvent(&event) || viz->loop == 1) {
    if (event.type == SDL_QUIT) {
      break;
    }

    if (event.type == SDL_KEYDOWN) {
      switch (event.key.keysym.sym) {
        case SDLK_ESCAPE:
          viz->loop = 0;
          break;
        case SDLK_q:
          viz->loop = 0;
          break;
      }
    }
  }
}

// /** Write BGR image to csv file **/
// void bgr2csv(const GstMapInfo *frame) {
//   FILE *csv = fopen("/tmp/test.csv", "w");
//   for (int i = 0; i < frame->size; i += 4) {
//     // Assuming BGRX format
//     const uint8_t b = frame->data[i];
//     const uint8_t g = frame->data[i + 1];
//     const uint8_t r = frame->data[i + 2];
//     const uint8_t v = 0.3 * r + 0.59 * g + 0.11 * b;
//     fprintf(csv, "%d ", v);
//   }
//   fflush(csv);
//   fclose(csv);
// }

/** Setup Gstreamer **/
void gst_setup(int argc, char *argv[]) {
  gst_debug_set_default_threshold(GST_LEVEL_WARNING);
  gst_init(&argc, &argv);
}

/** Camera callback **/
GstFlowReturn tis_callback(GstElement *sink, void *user_data) {
  tis_t *cam = (tis_t *) user_data;

  // Retrieve buffer data
  GstSample *sample = NULL;
  g_signal_emit_by_name(sink, "pull-sample", &sample, NULL);
  if (sample == NULL) {
    return GST_FLOW_OK;
  }

  GstBuffer *buffer = gst_sample_get_buffer(sample);
  GstMapInfo frame;

  if (gst_buffer_map(buffer, &frame, GST_MAP_READ)) {
    GstVideoInfo *video_info = gst_video_info_new();
    if (!gst_video_info_from_caps(video_info, gst_sample_get_caps(sample))) {
      g_warning("Failed to parse video info");
      return GST_FLOW_ERROR;
    }

    if (cam->viz) {
      const int img_width = video_info->width;
      viz_update(cam->viz, frame.data, img_width * 4);
    }

    // Clean up
    gst_buffer_unmap(buffer, &frame);
    gst_video_info_free(video_info);
  }

  GstClockTime ts = GST_BUFFER_PTS(buffer);
  g_print("Timestamp=%" GST_TIME_FORMAT "\t", GST_TIME_ARGS(ts));
  if (cam->viz && cam->viz->last_ts != 0) {
    const uint64_t last_ts = cam->viz->last_ts;
    const float time_diff_s = ((float) ts - last_ts) * 1e-9;
    const float frame_rate = 1.0 / time_diff_s;
    cam->viz->last_ts = ts;
    printf("frame_rate: %f\n", frame_rate);
  } else {
    g_print("\n");
  }

  // Clean up
  gst_sample_unref(sample);
  if (cam->viz) {
    cam->viz->last_ts = ts;
  }

  return GST_FLOW_OK;
}

/** Setup TIS Camera **/
int tis_setup(tis_t *cam, const int cam_idx, const char *serial, viz_t *viz) {
  // Properties
  cam->cam_idx = cam_idx;
  cam->serial = serial;
  cam->viz = viz;

  // Setup pipeline
  const char *pipeline_str =
      "tcambin name=source ! videoconvert ! appsink name=sink";
  GError *err = NULL;
  GstElement *pipeline = gst_parse_launch(pipeline_str, &err);
  if (pipeline == NULL) {
    printf("Could not create pipeline. Cause: %s\n", err->message);
    return 1;
  }
  cam->pipeline = pipeline;

  // Setup pipeline source
  if (serial != NULL) {
    GstElement *source = gst_bin_get_by_name(GST_BIN(pipeline), "source");
    GValue val = {0};
    g_value_init(&val, G_TYPE_STRING);
    g_value_set_static_string(&val, serial);
    g_object_set_property(G_OBJECT(source), "serial", &val);
    gst_object_unref(source);
  }

  // Setup pipeline sink
  GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  g_object_set(G_OBJECT(sink), "emit-signals", TRUE, NULL);
  g_signal_connect(sink, "new-sample", G_CALLBACK(tis_callback), cam);
  gst_object_unref(sink);

  return 0;
}

/** Run TIS camera **/
void tis_run(tis_t *cam) {
  gst_element_set_state(cam->pipeline, GST_STATE_PLAYING);
}

/** Clean up TIS camera **/
void tis_cleanup(tis_t *cam) {
  gst_element_set_state(cam->pipeline, GST_STATE_NULL);
  gst_object_unref(cam->pipeline);
}

#endif // TIS_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef TIS_UNITTEST

int main(int argc, char *argv[]) {
  gst_setup(argc, argv);

  // Setup
  int cam_idx = 0;
  const char *cam_serial = "19220362";
  // int cam_idx = 1;
  // const char *cam_serial = "19220363";
  tis_t cam;
  viz_t viz;

  viz_setup(&viz, "TIS Camera", 744, 480);
  if (tis_setup(&cam, cam_idx, cam_serial, &viz) != 0) {
    return -1;
  }

  // Run
  tis_run(&cam);
  viz_loop(&viz);

  // Clean up
  tis_cleanup(&cam);
  viz_free(&viz);

  return 0;
}

#endif // TIS_UNITTEST
