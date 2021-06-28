#include "../imshow.h"

int main() {
  const char *data_dir = "/data/euroc/raw/MH_01/mav0/cam0/data";
  int nb_files = 0;
  char **files = list_files(data_dir, &nb_files);

  imshow_t im;
  const char *title = "Test";
  imshow_load(&im, title, files[0]);

  /* for (int i = 1; i < nb_files; i++) { */
  while (1) {
    /* printf("."); */
    /* fflush(stdout); */

    /* image_t *img = image_load(files[i]); */
    /* imshow_update(&im, img); */
    /* image_free(img); */

    imshow_draw(&im);
    if (XK_q == imshow_wait(&im)) {
      break;
    }
  }
  imshow_free(&im);

  list_files_free(files, nb_files);

  return 0;
}
