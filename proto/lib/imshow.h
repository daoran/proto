#include <unistd.h>
#include <pthread.h>

#include <X11/Xlib.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>

#include "proto.h"

typedef struct imshow_t {
  /* Image data */
  char title[100];
  unsigned int texture_id;
  int img_w;
  int img_h;
  int img_c;
  uint8_t *data;

  /* X11 */
  Display *disp;
  int screen;
  Window root;
  Window win;

  int x;
  int y;
  int border_width;

  int mouse_pressed;
  int mouse_prev_x;
  int mouse_prev_y;
  int mouse_dx;
  int mouse_dy;

  float pan_x;
  float pan_y;
  float pan_factor;

  float zoom;
  float zoom_factor;

  XVisualInfo *vi;
  Colormap cmap;
  XWindowAttributes gwa;
  XSetWindowAttributes swa;

  GLXContext glc;

} imshow_t;

void imshow_setup(imshow_t *im);
void imshow_load(imshow_t *im, const char *title, const char *image_path);
void imshow_free(imshow_t *im);

void imshow_reset(imshow_t *im);
void imshow_pan(imshow_t *im, double dx, double dy);
void imshow_zoom_in(imshow_t *im, double x, double y);
void imshow_zoom_out(imshow_t *im, double x, double y);

void imshow_draw(imshow_t *im);
void imshow_update(imshow_t *im, image_t *img);

int imshow_wait(imshow_t *im);
void imshow_loop(imshow_t *im);
void *imshow_thread(void *data);
