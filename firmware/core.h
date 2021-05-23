#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

inline float deg2rad(const float d) {
  return d * (M_PI / 180.0);
}

inline float rad2deg(const float r) {
  return r * (180.0 / M_PI);
}

inline int8_t int8(const uint8_t *data, const size_t offset = 0) {
  return (int8_t)(data[offset]);
}

inline uint8_t uint8(const uint8_t *data, const size_t offset = 0) {
  return (uint8_t)(data[offset]);
}

inline int16_t int16(const uint8_t *data, const size_t offset = 0) {
  return (int16_t)((data[offset + 1] << 8) | (data[offset]));
}

inline uint16_t uint16(const uint8_t *data, const size_t offset = 0) {
  return (uint16_t)((data[offset + 1] << 8) | (data[offset]));
}

inline int32_t int32(const uint8_t *data, const size_t offset = 0) {
  return (int32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                   (data[offset + 1] << 8) | (data[offset]));
}

inline uint32_t uint32(const uint8_t *data, const size_t offset = 0) {
  return (uint32_t)((data[offset + 3] << 24) | (data[offset + 2] << 16) |
                    (data[offset + 1] << 8) | (data[offset]));
}

void euler321(const float euler[3], float C[3 * 3]) {
  assert(euler != NULL);
  assert(C != NULL);

  const float phi = euler[0];
  const float theta = euler[1];
  const float psi = euler[2];

  /* 1st row */
  C[0] = cos(psi) * cos(theta);
  C[1] = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  C[2] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  /* 2nd row */
  C[3] = sin(psi) * cos(theta);
  C[4] = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  C[5] = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  /* 3rd row */
  C[6] = -sin(theta);
  C[7] = cos(theta) * sin(phi);
  C[8] = cos(theta) * cos(phi);
}

void quat2euler(const float q[4], float euler[3]) {
  assert(q != NULL);
  assert(euler != NULL);

  const float qw = q[0];
  const float qx = q[1];
  const float qy = q[2];
  const float qz = q[3];

  const float qw2 = qw * qw;
  const float qx2 = qx * qx;
  const float qy2 = qy * qy;
  const float qz2 = qz * qz;

  const float t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const float t2 = asin(2 * (qy * qw - qx * qz));
  const float t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  euler[0] = t1;
  euler[1] = t2;
  euler[2] = t3;
}

void rot2quat(const float C[3 * 3], float q[4]) {
  assert(C != NULL);
  assert(q != NULL);

  const float C00 = C[0];
  const float C01 = C[1];
  const float C02 = C[2];
  const float C10 = C[3];
  const float C11 = C[4];
  const float C12 = C[5];
  const float C20 = C[6];
  const float C21 = C[7];
  const float C22 = C[8];

  const float tr = C00 + C11 + C22;
  float S = 0.0f;
  float qw = 0.0f;
  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;

  if (tr > 0) {
    S = sqrt(tr + 1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (C21 - C12) / S;
    qy = (C02 - C20) / S;
    qz = (C10 - C01) / S;
  } else if ((C00 > C11) && (C[0] > C22)) {
    S = sqrt(1.0 + C[0] - C11 - C22) * 2; // S=4*qx
    qw = (C21 - C12) / S;
    qx = 0.25 * S;
    qy = (C01 + C10) / S;
    qz = (C02 + C20) / S;
  } else if (C11 > C22) {
    S = sqrt(1.0 + C11 - C[0] - C22) * 2; // S=4*qy
    qw = (C02 - C20) / S;
    qx = (C01 + C10) / S;
    qy = 0.25 * S;
    qz = (C12 + C21) / S;
  } else {
    S = sqrt(1.0 + C22 - C[0] - C11) * 2; // S=4*qz
    qw = (C10 - C01) / S;
    qx = (C02 + C20) / S;
    qy = (C12 + C21) / S;
    qz = 0.25 * S;
  }

  q[0] = qw;
  q[1] = qx;
  q[2] = qy;
  q[3] = qz;
}

void tf_trans(const float T[16], float r[3]) {
  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

void tf_rot(const float T[16], float C[9]) {
  C[0] = T[0]; C[1] = T[1]; C[2] = T[2];
  C[3] = T[3]; C[4] = T[4]; C[5] = T[5];
  C[6] = T[6]; C[7] = T[7]; C[8] = T[8];
}

void tf_quat(const float T[4 * 4], float q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  float C[3 * 3] = {0};
  tf_rot(T, C);
  rot2quat(C, q);
}

/* void __assert(const char *__func, */
/*               const char *__file, */
/*               int __lineno, const char *__sexp) { */
/*   // transmit diagnostic informations through serial link. */
/*   Serial.println(__func); */
/*   Serial.println(__file); */
/*   Serial.println(__lineno, DEC); */
/*   Serial.println(__sexp); */
/*   Serial.flush(); */
/*   // abort program execution. */
/*   abort(); */
/* } */
