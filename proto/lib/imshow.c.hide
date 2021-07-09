#include "imshow.h"

void imshow_setup(imshow_t *im) {
  /* Setup display, screen and default root window */
  im->disp = XOpenDisplay(NULL);
  if (im->disp == NULL) {
    fprintf(stderr, "Cannot open display\n");
    exit(1);
  }
  im->screen = DefaultScreen(im->disp);
  im->root = DefaultRootWindow(im->disp);

  /* Visual info - Setup OpenGL context */
  GLint att[] = {GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None};
  im->vi = glXChooseVisual(im->disp, 0, att);
  if (im->vi == NULL) {
    FATAL("Failed to setup GL context!");
  }

  /* UI event defaults */
  im->mouse_pressed = 0;
  im->mouse_prev_x = 0;
  im->mouse_prev_y = 0;
  im->mouse_dx = 0;
  im->mouse_dy = 0;
  im->pan_x = 0;
  im->pan_y = 0;
  im->pan_factor = 0.95;
  im->zoom = 1.0;
  im->zoom_factor = 0.95;

  /* Colormap */
  im->cmap = XCreateColormap(im->disp, im->root, im->vi->visual, AllocNone);
  im->swa.colormap = im->cmap;
  im->swa.event_mask = ExposureMask | KeyPressMask;

  /* Setup window */
  im->x = (DisplayWidth(im->disp, im->screen) - im->img_w) / 2.0;
  im->y = (DisplayHeight(im->disp, im->screen) - im->img_h) / 2.0;
  im->border_width = 1;
  im->win = XCreateWindow(im->disp,
                          im->root,
                          im->x,
                          im->y,
                          im->img_w,
                          im->img_h,
                          im->border_width,
                          im->vi->depth,
                          InputOutput,
                          im->vi->visual,
                          CWColormap | CWEventMask,
                          &im->swa);
  int io_mask = ExposureMask;
  io_mask |= KeyPressMask;
  io_mask |= ButtonPressMask;
  io_mask |= ButtonReleaseMask;
  io_mask |= PointerMotionMask;
  XSelectInput(im->disp, im->win, io_mask);
  XMapWindow(im->disp, im->win);
  XStoreName(im->disp, im->win, im->title);

  /* Move the window to the middle of the screen */
  XMoveWindow(im->disp, im->win, im->x, im->y);
  XSync(im->disp, 0);

  /* Fix window size with hints */
  XSizeHints size_hints;
  size_hints.min_width = im->img_w;
  size_hints.max_width = im->img_w;
  size_hints.min_height = im->img_h;
  size_hints.max_height = im->img_h;
  XSetWMNormalHints(im->disp, im->win, &size_hints);

  /* Create OpenGL context for the window */
  im->glc = glXCreateContext(im->disp, im->vi, NULL, GL_TRUE);
  glXMakeCurrent(im->disp, im->win, im->glc);
  glEnable(GL_DEPTH_TEST);

  /* Load image data */
  glGenTextures(1, &im->texture_id);
  glBindTexture(GL_TEXTURE_2D, im->texture_id);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  GLenum format = 0;
  switch (im->img_c) {
  case 1: format = GL_LUMINANCE; break;
  case 3: format = GL_RGB; break;
  case 4: format = GL_RGBA; break;
  }

  glTexImage2D(GL_TEXTURE_2D,
               0,
               format,
               im->img_w,
               im->img_h,
               0,
               format,
               GL_UNSIGNED_BYTE,
               im->data);
  glEnable(GL_TEXTURE_2D);
}

void imshow_load(imshow_t *im, const char *title, const char *image_path) {
  image_t *img = image_load(image_path);

  strcpy(im->title, title);
  im->img_w = img->width;
  im->img_h = img->height;
  im->img_c = img->channels;
  im->data = img->data;

  imshow_setup(im);
}

void imshow_free(imshow_t *im) {
  glXMakeCurrent(im->disp, None, NULL);
  glXDestroyContext(im->disp, im->glc);
  XDestroyWindow(im->disp, im->win);
  XCloseDisplay(im->disp);
}

void imshow_reset(imshow_t *im) {
  im->pan_x = 0.0;
  im->pan_y = 0.0;
  im->zoom = 200.0;
}

void imshow_pan(imshow_t *im, double dx, double dy) {
  im->pan_x += dx / im->zoom;
  im->pan_y += dy / im->zoom;
}

void imshow_zoom_in(imshow_t *im, double x, double y) {
  imshow_pan(im, -x, -y);
  im->zoom *= im->zoom_factor;
  imshow_pan(im, x, y);
}

void imshow_zoom_out(imshow_t *im, double x, double y) {
  imshow_pan(im, -x, -y);
  im->zoom /= im->zoom_factor;
  imshow_pan(im, x, y);
}

void imshow_draw(imshow_t *im) {
  glMatrixMode(GL_MODELVIEW);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glBindTexture(GL_TEXTURE_2D, im->texture_id);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(-1.0, -1.0, 0.0);
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(1.0, -1.0, 0.0);
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(1.0, 1.0, 0.0);
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(-1.0, 1.0, 0.0);
  glEnd();
  glFlush();

  glLoadIdentity();
  glTranslatef(im->pan_x, im->pan_y, -1.0f);
  glScalef(im->zoom, im->zoom, 1.0f);

  glXSwapBuffers(im->disp, im->win);
}

void imshow_update(imshow_t *im, image_t *img) {
  for (int i = 0; i < (img->width * img->height); i++) {
    im->data[i] = img->data[i];
  }

  GLenum img_format = 0;
  switch (im->img_c) {
  case 1: img_format = GL_LUMINANCE; break;
  case 3: img_format = GL_RGB; break;
  case 4: img_format = GL_RGBA; break;
  }
  glBindTexture(GL_TEXTURE_2D, im->texture_id);
  glTexSubImage2D(GL_TEXTURE_2D,
                  0,
                  0,
                  0,
                  im->img_w,
                  im->img_h,
                  img_format,
                  GL_UNSIGNED_BYTE,
                  im->data);
  glBindTexture(GL_TEXTURE_2D, 0);

  imshow_draw(im);
}

int imshow_wait(imshow_t *im) {
  XEvent ev;
  XNextEvent(im->disp, &ev);

  /* Draw imshow */
  /* if (ev.type == Expose) { */
  /*   imshow_draw(im); */
  /* } */

  /* Window root, child; */
  /* int rootX, rootY, winX, winY; */
  /* unsigned int mask; */
  /* XQueryPointer(im->disp, DefaultRootWindow(im->disp), */
  /*               &root, &child, */
  /*               &rootX, &rootY, &winX, &winY, &mask); */

  /* Capture scroll events */
  if (ev.type == ButtonPress) {
    switch (ev.xbutton.button) {
    case Button1:
      im->mouse_pressed = 1;
      im->mouse_prev_x = ev.xbutton.x;
      im->mouse_prev_y = ev.xbutton.y;
      break;

    case Button4:
      /* imshow_zoom_in(im, ev.xmotion.x, ev.xmotion.y); */
      break;
    case Button5:
      /* imshow_zoom_out(im, ev.xmotion.x, ev.xmotion.y); */
      break;
    }
  }

  if (ev.type == MotionNotify && im->mouse_pressed) {
    im->mouse_dx = im->mouse_prev_x - ev.xmotion.x;
    im->mouse_dy = im->mouse_prev_y - ev.xmotion.y;

    im->pan_x -= (im->mouse_dx / (float) im->img_w) * 2.0 * im->zoom;
    im->pan_y += (im->mouse_dy / (float) im->img_h) * 2.0 * im->zoom;

    im->mouse_prev_x = ev.xmotion.x;
    im->mouse_prev_y = ev.xmotion.y;
    /* printf("dx: %d, dy: %d\n", im->mouse_dx, im->mouse_dy); */
    /* printf("panx: %f, pany: %f\n", im->pan_x, im->pan_y); */
  }

  if (ev.type == ButtonRelease) {
    switch (ev.xbutton.button) {
    case Button1:
      im->mouse_pressed = 0;
      im->mouse_dx = 0;
      im->mouse_dy = 0;
      /* printf("x: %d\n", ev.xbutton.x); */
      /* printf("y: %d\n", ev.xbutton.y); */
      /* printf("release!\n"); */
      break;
    }
  }

  /* Keyboard listener */
  if (ev.type == KeyPress) {
    int retval = 0;
    int keysyms_per_keycode_return;
    KeySym *keysym = XGetKeyboardMapping(im->disp,
                                         ev.xkey.keycode,
                                         1,
                                         &keysyms_per_keycode_return);
    retval = keysym[0];
    XFree(keysym);

    /* Reset view */
    if (retval == XK_r) {
      im->zoom = 1.0;
      im->pan_x = 0;
      im->pan_y = 0;
    }

    return retval;
  }

  return 0;
}

void imshow_loop(imshow_t *im) {
  while (1) {
    int key = imshow_wait(im);
    if (key == XK_q || key == XK_Escape) {
      break;
    }
  }

  imshow_free(im);
}

void *imshow_thread(void *data) {
  imshow_t *im = (imshow_t *) data;
  imshow_setup(im);
  imshow_loop(im);
  return NULL;
}
