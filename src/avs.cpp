#define AVS_UNITTESTS
#include "avs.hpp"

int main() {
  run_unittests();

  // Setup
  // const char *data_path = "/data/euroc/V1_01";
  // euroc_data_t *data = euroc_data_load(data_path);
  // euroc_timeline_t *timeline = data->timeline;

  // auto front_end = Tracker();
  // for (size_t k = 0; k < timeline->num_timestamps; k++) {
  //   const timestamp_t ts = timeline->timestamps[k];
  //   const euroc_event_t *event = &timeline->events[k];

  //   if (event->has_cam0 && event->has_cam1) {
  //     const cv::Mat img0 = cv::imread(event->cam0_image);
  //     const cv::Mat img1 = cv::imread(event->cam1_image);

  //     front_end.detect(img0, img1, true);

  //     cv::Mat viz;
  //     cv::hconcat(img0, img1, viz);
  //     cv::imshow("Stereo-Camera", viz);
  //     cv::waitKey(1);
  //   }
  // }

  // Clean up
  // euroc_data_free(data);

  return 0;
}
