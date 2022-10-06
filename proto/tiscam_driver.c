#include <gst/gst.h>
#include <gst/video/video.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static GstFlowReturn callback(GstElement *sink,
                              void *user_data __attribute__((unused))) {
  // Retrieve buffer data
  GstSample *sample = NULL;
  g_signal_emit_by_name(sink, "pull-sample", &sample, NULL);
  if (sample) {
    // we have a valid sample
    // do things with the image here
    static guint framecount = 0;
    int pixel_data = -1;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo info; // contains the actual image
    if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
      GstVideoInfo *video_info = gst_video_info_new();
      if (!gst_video_info_from_caps(video_info, gst_sample_get_caps(sample))) {
        // Could not parse video info (should not happen)
        g_warning("Failed to parse video info");
        return GST_FLOW_ERROR;
      }

      // pointer to the image data

      // unsigned char* data = info.data;

      // Get the pixel value of the center pixel

      // int stride = video_info->finfo->bits / 8;
      // unsigned int pixel_offset = video_info->width / 2 * stride
      //                             + video_info->width * video_info->height /
      //                             2 * stride;

      // this is only one pixel
      // when dealing with formats like BGRx
      // pixel_data will consist out of
      // pixel_offset   => B
      // pixel_offset+1 => G
      // pixel_offset+2 => R
      // pixel_offset+3 => x

      // pixel_data = info.data[pixel_offset];

      gst_buffer_unmap(buffer, &info);
      gst_video_info_free(video_info);
    }

    GstClockTime timestamp = GST_BUFFER_PTS(buffer);
    g_print("Captured frame %d, Pixel Value=%03d Timestamp=%" GST_TIME_FORMAT
            "            \r",
            framecount,
            pixel_data,
            GST_TIME_ARGS(timestamp));
    framecount++;

    // delete our reference so that gstreamer can handle the sample
    gst_sample_unref(sample);
  }
  return GST_FLOW_OK;
}

int main(int argc, char *argv[]) {
  // Start GStreamer
  gst_debug_set_default_threshold(GST_LEVEL_WARNING);
  gst_init(&argc, &argv);

  // Setup pipeline
  const char *serial = NULL; // the serial number of the camera we want to use
  const char *pipeline_str =
      "tcambin name=source ! videoconvert ! appsink name=sink";
  GError *err = NULL;
  GstElement *pipeline = gst_parse_launch(pipeline_str, &err);
  if (pipeline == NULL) {
    printf("Could not create pipeline. Cause: %s\n", err->message);
    return 1;
  }

  // Setup pipeline source
  if (serial != NULL) {
    GstElement *source = gst_bin_get_by_name(GST_BIN(pipeline), "source");

    GValue val = {};
    g_value_init(&val, G_TYPE_STRING);
    g_value_set_static_string(&val, serial);

    g_object_set_property(G_OBJECT(source), "serial", &val);
    gst_object_unref(source);
  }

  // Setup pipeline sink
  GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  g_object_set(G_OBJECT(sink), "emit-signals", TRUE, NULL);
  g_signal_connect(sink, "new-sample", G_CALLBACK(callback), NULL);
  gst_object_unref(sink);

  // Start pipeline
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  g_print("Press 'enter' to stop the stream.\n");
  getchar();

  // Clean up
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);

  return 0;
}
