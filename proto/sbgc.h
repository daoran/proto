#ifndef SBGC_H
#define SBGC_H

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

// MACROS
#ifndef WARN_UNUSED
#define WARN_UNUSED __attribute__((warn_unused_result))
#endif

/**
 * Log info
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef SBGC_INFO
#define SBGC_INFO(...)                                                         \
  do {                                                                         \
    fprintf(stderr,                                                            \
            "[SBGC-INFO] [%s:%d:%s()]: ",                                      \
            __FILE__,                                                          \
            __LINE__,                                                          \
            __func__);                                                         \
    fprintf(stderr, __VA_ARGS__);                                              \
    fprintf(stderr, "\n");                                                     \
  } while (0)
#endif // SBGC_INFO

/**
 * Log error
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef SBGC_ERROR
#define SBGC_ERROR(...)                                                        \
  do {                                                                         \
    fprintf(stderr,                                                            \
            "[SBGC-ERROR] [%s:%d:%s()]: ",                                     \
            __FILE__,                                                          \
            __LINE__,                                                          \
            __func__);                                                         \
    fprintf(stderr, __VA_ARGS__);                                              \
    fprintf(stderr, "\n");                                                     \
  } while (0)
#endif // SBGC_ERROR

// GENERAL
#define SBGC_CMD_MAX_BYTES 255
#define SBGC_CMD_PAYLOAD_BYTES 5
#define SBGC_DEG_PER_BIT 0.02197265625    // deg per bit
#define SBGC_DEG_SEC_PER_BIT 0.1220740379 // deg/sec per bit
#define SBGC_ACC_UNIT (1.0 / 512.0)       // G
#define SBGC_GYRO_UNIT 0.06103701895      // deg per sec

// CMD ID
#define SBGC_CMD_READ_PARAMS 82
#define SBGC_CMD_WRITE_PARAMS 87
#define SBGC_CMD_REALTIME_DATA 68
#define SBGC_CMD_BOARD_INFO 86
#define SBGC_CMD_CALIB_ACC 65
#define SBGC_CMD_CALIB_GYRO 103
#define SBGC_CMD_CALIB_EXT_GAIN 71
#define SBGC_CMD_USE_DEFAULTS 70
#define SBGC_CMD_CALIB_POLES 80
#define SBGC_CMD_RESET 114
#define SBGC_CMD_HELPER_DATA 72
#define SBGC_CMD_CALIB_OFFSET 79
#define SBGC_CMD_CALIB_BAT 66
#define SBGC_CMD_MOTORS_ON 77
#define SBGC_CMD_MOTORS_OFF 109
#define SBGC_CMD_CONTROL 67
#define SBGC_CMD_TRIGGER_PIN 84
#define SBGC_CMD_EXECUTE_MENU 69
#define SBGC_CMD_GET_ANGLES 73
#define SBGC_CMD_GET_ANGLES_EXT 61
#define SBGC_CMD_CONFIRM 67
#define SBGC_CMD_BOARD_INFO_3 20
#define SBGC_CMD_READ_PARAMS_3 21
#define SBGC_CMD_WRITE_PARAMS_3 22
#define SBGC_CMD_REALTIME_DATA_3 23
#define SBGC_CMD_REALTIME_DATA_4 25
#define SBGC_CMD_SELECT_IMU_3 24
#define SBGC_CMD_READ_PROFILE_NAMES 28
#define SBGC_CMD_WRITE_PROFILE_NAMES 29
#define SBGC_CMD_QUEUE_PARAMS_INFO_3 30
#define SBGC_CMD_SET_ADJ_VARS_VAL 31
#define SBGC_CMD_SAVE_PARAMS_3 32
#define SBGC_CMD_READ_PARAMS_EXT 33
#define SBGC_CMD_WRITE_PARAMS_EXT 34
#define SBGC_CMD_AUTO_PID 35
#define SBGC_CMD_SERVO_OUT 36
#define SBGC_CMD_I2C_WRITE_REG_BUF 39
#define SBGC_CMD_I2C_READ_REG_BUF 40
#define SBGC_CMD_WRITE_EXTERNAL_DATA 41
#define SBGC_CMD_READ_EXTERNAL_DATA 42
#define SBGC_CMD_READ_ADJ_VARS_CFG 43
#define SBGC_CMD_WRITE_ADJ_VARS_CFG 44
#define SBGC_CMD_API_VIRT_CH_CONTROL 45
#define SBGC_CMD_ADJ_VARS_STATE 46
#define SBGC_CMD_EEPROM_WRITE 47
#define SBGC_CMD_EEPROM_READ 48
#define SBGC_CMD_CALIB_INFO 49
#define SBGC_CMD_BOOT_MODE_3 51

// CMD FRAME SIZE
#define SBGC_MIN_FRAME_SIZE 5 // 4 bytes for header + 1 body checksum
#define SBGC_CMD_BOARD_INFO_FRAME_SIZE 5 + 18
#define SBGC_CMD_REALTIME_DATA_3_FRAME_SIZE 5 + 63

// CMD CONTROL
#define SBGC_MODE_NO_CONTROL 0
#define SBGC_MODE_SPEED 1
#define SBGC_MODE_ANGLE 2
#define SBGC_MODE_SPEED_ANGLE 3
#define SBGC_MODE_RC 4
#define SBGC_MODE_ANGLE_REL_FRAME 5

// SYSTEM ERRORS
// ERR_NO_SENSOR (1<<0)
// ERR_CALIB_ACC (1<<1)
// ERR_SET_POWER (1<<2)
// ERR_CALIB_POLES (1<<3)
// ERR_PROTECTION (1<<4)
// ERR_SERIAL (1<<5)
// Beside that, extended error contains bits:
// ERR_LOW_BAT1 (1<<6)
// ERR_LOW_BAT2 (1<<7)
// ERR_GUI_VERSION (1<<8)
// ERR_MISS_STEPS (1<<9)
// ERR_SYSTEM (1<<10)
// ERR_EMERGENCY_STOP (1<<11)

typedef struct sbgc_frame_t {
  uint8_t cmd_id;
  uint8_t data_size;
  uint8_t header_checksum;
  uint8_t *data;
  uint8_t data_checksum;

} sbgc_frame_t;

typedef struct sbgc_data_t {
  float accel[3];
  float gyro[3];

  float camera_angles[3];
  float frame_angles[3];
  float rc_angles[3];
  float encoder_angles[3];

  int cycle_time;
  int i2c_error_count;
  int system_error;
  int battery_level;
} sbgc_data_t;

typedef struct sbgc_t {
  int connected;
  sbgc_data_t data;
  const char *port;
  int serial;

  uint8_t board_version;
  uint16_t firmware_version;
  uint8_t debug_mode;
  uint16_t board_features;
  uint8_t connection_flags;
} sbgc_t;

void sbgc_frame_print(const sbgc_frame_t *frame);
void sbgc_frame_set_header(sbgc_frame_t *frame,
                           uint8_t cmd_id,
                           uint8_t data_size);
void sbgc_frame_set_checksum(sbgc_frame_t *frame);
void sbgc_frame_set_body(sbgc_frame_t *frame, uint8_t *data);
void sbgc_frame_set_frame(sbgc_frame_t *frame,
                          int cmd_id,
                          uint8_t *data,
                          int data_size);
void sbgc_frame_set_cmd(sbgc_frame_t *frame, int cmd_id);
int sbgc_frame_parse_header(sbgc_frame_t *frame, uint8_t *data);
int sbgc_frame_parse_body(sbgc_frame_t *frame, uint8_t *data);
int sbgc_frame_parse_frame(sbgc_frame_t *frame, uint8_t *data);
void sbgc_data_t_print(sbgc_data_t *data);

int sbgc_connect(sbgc_t *sbgc, const char *port);
void sbgc_reset(sbgc_t *sbgc);
int sbgc_disconnect(sbgc_t *sbgc);
int sbgc_send(const sbgc_t *sbgc, const sbgc_frame_t *frame);
int sbgc_read(const sbgc_t *sbgc,
              const uint8_t read_length,
              sbgc_frame_t *frame);
int sbgc_on(const sbgc_t *sbgc);
int sbgc_off(const sbgc_t *sbgc);
int sbgc_info(sbgc_t *sbgc);
int sbgc_get_realtime_data_4(sbgc_t *sbgc);
int sbgc_get_realtime_data(sbgc_t *sbgc);
int sbgc_get_angle_ext(sbgc_t *sbgc);
int sbgc_set_angle(const sbgc_t *sbgc,
                   const float roll,
                   const float pitch,
                   const float yaw);
int sbgc_set_angle_speed(const sbgc_t *sbgc,
                         const float roll,
                         const float pitch,
                         const float yaw,
                         const float roll_speed,
                         const float pitch_speed,
                         const float yaw_speed);
#endif // SBGC_H

//////////////////////////////////////////////////////////////////////////////
//                             IMPLEMENTATION                               //
//////////////////////////////////////////////////////////////////////////////

#ifdef SBGC_IMPLEMENTATION

// SBGC UTILS ////////////////////////////////////////////////////////////////

int sbgc_set_interface_attributes(int fd, int speed, int parity) {
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0) {
    SBGC_ERROR("%d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK; // disable break processing
  tty.c_lflag = 0;        // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;     // no remapping, no delays
  tty.c_cc[VMIN] = 0;  // read doesn't block
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    SBGC_ERROR("%d from tcsetattr", errno);
    return -1;
  }

  return 0;
}

void sbgc_set_blocking(int fd, int should_block) {
  struct termios tty;

  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    SBGC_ERROR("%d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN] = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    SBGC_ERROR("%d setting term attributes", errno);
  }
}

int16_t sbgc_s16bit(const uint8_t *data,
                    const int hi_byte,
                    const int low_byte) {
  return (int16_t)((data[hi_byte] << 8) | (data[low_byte] & 0xff));
}

uint16_t sbgc_u16bit(const uint8_t *data,
                     const int hi_byte,
                     const int low_byte) {
  return (uint16_t)((data[hi_byte] << 8) | (data[low_byte] & 0xff));
}

float sbgc_parse_accel(const uint8_t *data,
                       const int hi_byte,
                       const int low_byte) {
  return SBGC_ACC_UNIT * sbgc_s16bit(data, hi_byte, low_byte);
}

float sbgc_parse_gyro(const uint8_t *data,
                      const int hi_byte,
                      const int low_byte) {
  return SBGC_GYRO_UNIT * sbgc_s16bit(data, hi_byte, low_byte);
}

float sbgc_parse_angle(const uint8_t *data,
                       const int hi_byte,
                       const int low_byte) {
  return SBGC_DEG_PER_BIT * sbgc_s16bit(data, hi_byte, low_byte);
}

// SBGC FRAME ////////////////////////////////////////////////////////////////

void sbgc_frame_print(const sbgc_frame_t *frame) {
  int i;

  // Header
  printf("[%d]: %c\n", 0, '>');
  printf("[%d]: %c\n", 1, frame->cmd_id);
  printf("[%d]: %d\n", 2, frame->data_size);
  printf("[%d]: %d\n", 3, frame->header_checksum);

  // Body
  for (i = 4; i < (frame->data_size + 4); i++) {
    printf("[%d]: %d\n", i, frame->data[i]);
  }
  printf("[%d]: %d\n", i, frame->data_checksum);
}

void sbgc_frame_set_header(sbgc_frame_t *frame,
                           uint8_t cmd_id,
                           uint8_t data_size) {
  frame->cmd_id = cmd_id;
  frame->data_size = data_size;
  frame->header_checksum = (frame->cmd_id + frame->data_size) % 256;
}

void sbgc_frame_set_checksum(sbgc_frame_t *frame) {
  frame->data_checksum = 0x0;
  for (int i = 0; i < frame->data_size; i++) {
    frame->data_checksum += frame->data[i];
  }
  frame->data_checksum = frame->data_checksum % 256;
}

void sbgc_frame_set_body(sbgc_frame_t *frame, uint8_t *data) {
  frame->data = data;
  sbgc_frame_set_checksum(frame);
}

void sbgc_frame_set_frame(sbgc_frame_t *frame,
                          int cmd_id,
                          uint8_t *data,
                          int data_size) {
  sbgc_frame_set_header(frame, (uint8_t) cmd_id, (uint8_t) data_size);
  sbgc_frame_set_body(frame, data);
}

void sbgc_frame_set_cmd(sbgc_frame_t *frame, int cmd_id) {
  sbgc_frame_set_header(frame, (uint8_t) cmd_id, 0);
  sbgc_frame_set_body(frame, NULL);
}

int sbgc_frame_parse_header(sbgc_frame_t *frame, uint8_t *data) {
  uint8_t expected_checksum;

  // Pre-check
  if (data[0] != '>') {
    return -1;
  }

  // Parse header
  frame->cmd_id = data[1];
  frame->data_size = data[2];
  frame->header_checksum = data[3];

  // Check the header checksum
  expected_checksum = (frame->cmd_id + frame->data_size) % 256;
  if (frame->header_checksum != expected_checksum) {
    return -1;
  }

  return 0;
}

int sbgc_frame_parse_body(sbgc_frame_t *frame, uint8_t *data) {
  uint8_t i;
  uint8_t expected_checksum;

  // Setup
  expected_checksum = 0x0;
  frame->data = malloc(sizeof(uint8_t) * frame->data_size);

  // Parse body
  for (i = 0; i < frame->data_size; i++) {
    frame->data[i] = data[4 + i]; // +4 because header is 4 bytes
    expected_checksum += data[4 + i];
  }
  frame->data_checksum = data[4 + i];

  // Check the body checksum
  expected_checksum = expected_checksum % 256;
  if (frame->data_checksum != expected_checksum) {
    SBGC_ERROR("Failed body checksum!");
    free(frame->data);
    return -1;
  }

  return 0;
}

int sbgc_frame_parse_frame(sbgc_frame_t *frame, uint8_t *data) {
  int retval;

  // Parse header
  retval = sbgc_frame_parse_header(frame, data);
  if (retval == -1) {
    SBGC_ERROR("Failed to parse header!");
    return -1;
  }

  // Parse body
  retval = sbgc_frame_parse_body(frame, data);
  if (retval == -1) {
    SBGC_ERROR("Failed to parse body!");
    return -1;
  }

  return 0;
}

// SBGC DATA /////////////////////////////////////////////////////////////////

void sbgc_data_t_print(sbgc_data_t *data) {
  // Accelerometer and gyroscope
  printf("accelerometer: %.2f\t%.2f\t%.2f\n",
         data->accel[0],
         data->accel[1],
         data->accel[2]);
  printf("gyroscope: %.2f\t%.2f\t%.2f\n",
         data->gyro[0],
         data->gyro[1],
         data->gyro[2]);
  printf("\n");

  // Angles
  printf("camera_angles: %.2f\t%.2f\t%.2f\n",
         data->camera_angles[0],
         data->camera_angles[1],
         data->camera_angles[2]);
  printf("frame_angles: %.2f\t%.2f\t%.2f\n",
         data->frame_angles[0],
         data->frame_angles[1],
         data->frame_angles[2]);
  printf("rc_angles: %.2f\t%.2f\t%.2f\n",
         data->rc_angles[0],
         data->rc_angles[1],
         data->rc_angles[2]);

  // Misc
  printf("cycle_time: %d\n", data->cycle_time);
  printf("i2c_error_count: %d\n", data->i2c_error_count);
  printf("system_error: %d\n", data->system_error);
  printf("battery_level: %d\n\n", data->battery_level);
}

// SBGC //////////////////////////////////////////////////////////////////////

int sbgc_connect(sbgc_t *sbgc, const char *port) {
  sbgc->connected = 0;

  sbgc->port = port;
  sbgc->serial = -1;

  sbgc->board_version = 0;
  sbgc->firmware_version = 0;
  sbgc->debug_mode = 0;
  sbgc->board_features = 0;
  sbgc->connection_flags = 0;

  // Open serial port
  sbgc->serial = open(sbgc->port, O_RDWR | O_NOCTTY | O_SYNC);
  if (sbgc->serial < 0) {
    return -1;
  }

  // Configure serial commnication
  sbgc_set_interface_attributes(sbgc->serial, B115200, 0);
  sbgc_set_blocking(sbgc->serial, 1);
  sbgc->connected = 1;

  return 0;
}

int sbgc_disconnect(sbgc_t *sbgc) {
  if (close(sbgc->serial) != 0) {
    return -1;
  }

  sbgc->connected = 0;
  return 0;
}

void sbgc_reset(sbgc_t *sbgc) {
  sbgc->connected = 0;

  sbgc->port = NULL;
  sbgc->serial = -1;

  sbgc->board_version = 0;
  sbgc->firmware_version = 0;
  sbgc->debug_mode = 0;
  sbgc->board_features = 0;
  sbgc->connection_flags = 0;
}

int sbgc_send(const sbgc_t *sbgc, const sbgc_frame_t *frame) {
  // Check connection
  if (sbgc->connected == 0) {
    return -1;
  }

  // Check command data size
  int data_size_limit;
  data_size_limit = SBGC_CMD_MAX_BYTES - SBGC_CMD_PAYLOAD_BYTES;
  if (frame->data_size >= data_size_limit) {
    return -1;
  }

  // Header
  ssize_t retval = 0;
  const uint8_t start = 0x3E; // ">" character
  retval += write(sbgc->serial, &start, 1);
  retval += write(sbgc->serial, &frame->cmd_id, 1);
  retval += write(sbgc->serial, &frame->data_size, 1);

  // Body
  retval += write(sbgc->serial, &frame->header_checksum, 1);
  retval += write(sbgc->serial, frame->data, frame->data_size);
  retval += write(sbgc->serial, &frame->data_checksum, 1);

  // Flush
  tcflush(sbgc->serial, TCIOFLUSH); // VERY CRITICAL
  usleep(10 * 1000);
  if (retval != (5 + frame->data_size)) {
    printf("Opps! frame wasn't sent completely!\n");
  }

  return 0;
}

int sbgc_read(const sbgc_t *sbgc,
              const uint8_t read_length,
              sbgc_frame_t *frame) {
  // Check connection
  if (sbgc->connected == 0) {
    return -1;
  }

  // Send query
  uint8_t buffer[150];
  int16_t nb_bytes = read(sbgc->serial, buffer, read_length);
  if (nb_bytes <= 0 || nb_bytes != read_length) {
    SBGC_ERROR("Failed to read SBGC frame!");
    return -1;
  }
  for (int i = 0; i < nb_bytes; i++) {
    printf("%c", buffer[i]);
  }
  printf("\n");

  // Parse sbgc frame
  int retval = sbgc_frame_parse_frame(frame, buffer);
  if (retval == -1) {
    SBGC_ERROR("Failed to parse SBGC frame!");
    return -1;
  }

  return 0;
}

int sbgc_on(const sbgc_t *sbgc) {
  sbgc_frame_t frame;
  sbgc_frame_set_cmd(&frame, SBGC_CMD_MOTORS_ON);
  return sbgc_send(sbgc, &frame);
}

int sbgc_off(const sbgc_t *sbgc) {
  // Check connection
  if (sbgc->connected == 0) {
    return -1;
  }

  // Turn off motor control
  {
    uint8_t data[13];
    data[0] = SBGC_MODE_NO_CONTROL;
    for (int i = 1; i < 13; i++) {
      data[i] = 0;
    }

    sbgc_frame_t frame;
    sbgc_frame_set_frame(&frame, SBGC_CMD_CONTROL, data, 13);
    if (sbgc_send(sbgc, &frame) != 0) {
      return -2;
    }
  }

  // Turn off motors
  {
    sbgc_frame_t frame;
    sbgc_frame_set_cmd(&frame, SBGC_CMD_MOTORS_OFF);
    if (sbgc_send(sbgc, &frame) != 0) {
      return -2;
    }
  }

  return 0;
}

int sbgc_info(sbgc_t *sbgc) {
  // Request board info
  sbgc_frame_t frame;
  sbgc_frame_set_cmd(&frame, SBGC_CMD_BOARD_INFO);
  SBGC_INFO("Sending board info request!");
  if (sbgc_send(sbgc, &frame) == -1) {
    SBGC_ERROR("Failed to request SBGC board info!");
    return -1;
  }

  // Obtain board info
  sbgc_frame_t info;
  SBGC_INFO("Read board info request!");
  if (sbgc_read(sbgc, SBGC_CMD_BOARD_INFO_FRAME_SIZE, &info) == -1) {
    SBGC_ERROR("Failed to parse SBGC frame for board info!");
    return -1;
  }
  sbgc->board_version = info.data[0];
  sbgc->firmware_version = (info.data[2] << 8) | (info.data[1] & 0xff);
  sbgc->debug_mode = info.data[3];
  sbgc->board_features = (info.data[5] << 8) | (info.data[4] & 0xff);
  sbgc->connection_flags = info.data[6];
  free(frame.data);

  return 0;
}

int sbgc_get_realtime_data_4(sbgc_t *sbgc) {
  // Request real time data
  sbgc_frame_t frame;
  sbgc_frame_set_cmd(&frame, SBGC_CMD_REALTIME_DATA_4);
  if (sbgc_send(sbgc, &frame) == -1) {
    SBGC_ERROR("Failed to request SBGC realtime data!");
    return -1;
  }

  // Obtain real time data
  if (sbgc_read(sbgc, 129, &frame) == -1) {
    SBGC_ERROR("Failed to parse SBGC frame for realtime data!");
    return -1;
  }

  // Parse real time data
  sbgc->data.accel[0] = sbgc_parse_accel(frame.data, 1, 0);
  sbgc->data.accel[1] = sbgc_parse_accel(frame.data, 5, 4);
  sbgc->data.accel[2] = sbgc_parse_accel(frame.data, 9, 8);

  sbgc->data.gyro[0] = sbgc_parse_gyro(frame.data, 3, 2);
  sbgc->data.gyro[1] = sbgc_parse_gyro(frame.data, 7, 6);
  sbgc->data.gyro[2] = sbgc_parse_gyro(frame.data, 11, 10);

  sbgc->data.camera_angles[0] = sbgc_parse_angle(frame.data, 33, 32);
  sbgc->data.camera_angles[1] = sbgc_parse_angle(frame.data, 35, 34);
  sbgc->data.camera_angles[2] = sbgc_parse_angle(frame.data, 37, 36);

  sbgc->data.frame_angles[0] = sbgc_parse_angle(frame.data, 39, 38);
  sbgc->data.frame_angles[1] = sbgc_parse_angle(frame.data, 41, 40);
  sbgc->data.frame_angles[2] = sbgc_parse_angle(frame.data, 43, 42);

  sbgc->data.rc_angles[0] = sbgc_parse_angle(frame.data, 45, 44);
  sbgc->data.rc_angles[1] = sbgc_parse_angle(frame.data, 47, 45);
  sbgc->data.rc_angles[2] = sbgc_parse_angle(frame.data, 49, 46);

  sbgc->data.encoder_angles[0] = sbgc_parse_angle(frame.data, 64, 63);
  sbgc->data.encoder_angles[1] = sbgc_parse_angle(frame.data, 66, 65);
  sbgc->data.encoder_angles[2] = sbgc_parse_angle(frame.data, 68, 67);

  sbgc->data.cycle_time = sbgc_u16bit(frame.data, 51, 50);
  sbgc->data.i2c_error_count = sbgc_u16bit(frame.data, 53, 52);
  sbgc->data.system_error = sbgc_u16bit(frame.data, 15, 14);
  sbgc->data.battery_level = sbgc_u16bit(frame.data, 56, 55);

  return 0;
}

int sbgc_get_realtime_data(sbgc_t *sbgc) {
  // Request real time data
  sbgc_frame_t frame;
  sbgc_frame_set_cmd(&frame, SBGC_CMD_REALTIME_DATA_3);
  if (sbgc_send(sbgc, &frame) == -1) {
    SBGC_ERROR("Failed to request SBGC realtime data!");
    return -1;
  }

  // Obtain real time data
  sbgc_frame_t info;
  if (sbgc_read(sbgc, 68, &info) == -1) {
    SBGC_ERROR("Failed to parse SBGC frame for realtime data!");
    return -1;
  }

  // Parse real time data
  sbgc->data.accel[0] = sbgc_parse_accel(frame.data, 1, 0);
  sbgc->data.accel[1] = sbgc_parse_accel(frame.data, 5, 4);
  sbgc->data.accel[2] = sbgc_parse_accel(frame.data, 9, 8);

  sbgc->data.gyro[0] = sbgc_parse_gyro(frame.data, 3, 2);
  sbgc->data.gyro[1] = sbgc_parse_gyro(frame.data, 7, 6);
  sbgc->data.gyro[2] = sbgc_parse_gyro(frame.data, 11, 10);

  sbgc->data.camera_angles[0] = sbgc_parse_angle(frame.data, 33, 32);
  sbgc->data.camera_angles[1] = sbgc_parse_angle(frame.data, 35, 34);
  sbgc->data.camera_angles[2] = sbgc_parse_angle(frame.data, 37, 36);

  sbgc->data.frame_angles[0] = sbgc_parse_angle(frame.data, 39, 38);
  sbgc->data.frame_angles[1] = sbgc_parse_angle(frame.data, 41, 40);
  sbgc->data.frame_angles[2] = sbgc_parse_angle(frame.data, 43, 42);

  sbgc->data.rc_angles[0] = sbgc_parse_angle(frame.data, 45, 44);
  sbgc->data.rc_angles[1] = sbgc_parse_angle(frame.data, 47, 45);
  sbgc->data.rc_angles[2] = sbgc_parse_angle(frame.data, 49, 46);

  sbgc->data.cycle_time = sbgc_u16bit(frame.data, 51, 50);
  sbgc->data.i2c_error_count = sbgc_u16bit(frame.data, 53, 52);
  sbgc->data.system_error = sbgc_u16bit(frame.data, 15, 14);
  sbgc->data.battery_level = sbgc_u16bit(frame.data, 56, 55);

  return 0;
}

int sbgc_get_angle_ext(sbgc_t *sbgc) {
  // Request real time data
  sbgc_frame_t frame;
  sbgc_frame_set_cmd(&frame, SBGC_CMD_GET_ANGLES_EXT);
  if (sbgc_send(sbgc, &frame) == -1) {
    SBGC_ERROR("Failed to request SBGC realtime data!");
    return -1;
  }

  // Obtain real time data
  if (sbgc_read(sbgc, 54, &frame) == -1) {
    SBGC_ERROR("Failed to parse SBGC frame for realtime data!");
    return -1;
  }

  sbgc->data.camera_angles[0] = sbgc_parse_angle(frame.data, 1, 0);
  sbgc->data.camera_angles[1] = sbgc_parse_angle(frame.data, 19, 18);
  sbgc->data.camera_angles[2] = sbgc_parse_angle(frame.data, 37, 36);

  sbgc->data.rc_angles[0] = sbgc_parse_angle(frame.data, 3, 2);
  sbgc->data.rc_angles[1] = sbgc_parse_angle(frame.data, 21, 20);
  sbgc->data.rc_angles[2] = sbgc_parse_angle(frame.data, 39, 38);

  sbgc->data.encoder_angles[0] = sbgc_parse_angle(frame.data, 8, 4);
  sbgc->data.encoder_angles[1] = sbgc_parse_angle(frame.data, 26, 22);
  sbgc->data.encoder_angles[2] = sbgc_parse_angle(frame.data, 44, 40);

  return 0;
}

int sbgc_set_angle(const sbgc_t *sbgc,
                   const float roll,
                   const float pitch,
                   const float yaw) {
  // Adjust roll, pitch and yaw
  const int16_t roll_adjusted = roll / SBGC_DEG_PER_BIT;
  const int16_t pitch_adjusted = pitch / SBGC_DEG_PER_BIT;
  const int16_t yaw_adjusted = yaw / SBGC_DEG_PER_BIT;

  // Control mode
  uint8_t data[13];
  data[0] = SBGC_MODE_ANGLE;

  // Speed roll
  data[1] = 0;
  data[2] = 0;

  // Angle roll
  data[3] = ((roll_adjusted >> 0) & 0xff);
  data[4] = ((roll_adjusted >> 8) & 0xff);

  // Speed pitch
  data[5] = 0;
  data[6] = 0;

  // Angle pitch
  data[7] = ((pitch_adjusted >> 0) & 0xff);
  data[8] = ((pitch_adjusted >> 8) & 0xff);

  // Speed yaw
  data[9] = 0;
  data[10] = 0;

  // Angle yaw
  data[11] = ((yaw_adjusted >> 0) & 0xff);
  data[12] = ((yaw_adjusted >> 8) & 0xff);

  // Build frame and send
  sbgc_frame_t frame;
  sbgc_frame_set_frame(&frame, SBGC_CMD_CONTROL, data, 13);
  sbgc_send(sbgc, &frame);

  return 0;
}

int sbgc_set_angle_speed(const sbgc_t *sbgc,
                         const float roll,
                         const float pitch,
                         const float yaw,
                         const float roll_speed,
                         const float pitch_speed,
                         const float yaw_speed) {
  // Adjust roll, pitch and yaw
  const int16_t roll_adjusted = roll / SBGC_DEG_PER_BIT;
  const int16_t pitch_adjusted = pitch / SBGC_DEG_PER_BIT;
  const int16_t yaw_adjusted = yaw / SBGC_DEG_PER_BIT;
  const int16_t roll_speed_adjusted = roll_speed / SBGC_DEG_SEC_PER_BIT;
  const int16_t pitch_speed_adjusted = pitch_speed / SBGC_DEG_SEC_PER_BIT;
  const int16_t yaw_speed_adjusted = yaw_speed / SBGC_DEG_SEC_PER_BIT;

  // Control mode
  uint8_t data[13];
  data[0] = SBGC_MODE_SPEED_ANGLE;

  // Speed roll
  data[1] = ((roll_speed_adjusted >> 0) & 0xff);
  data[2] = ((roll_speed_adjusted >> 8) & 0xff);

  // Angle roll
  data[3] = ((roll_adjusted >> 0) & 0xff);
  data[4] = ((roll_adjusted >> 8) & 0xff);

  // Speed pitch
  data[5] = ((pitch_speed_adjusted >> 0) & 0xff);
  data[6] = ((pitch_speed_adjusted >> 8) & 0xff);

  // Angle pitch
  data[7] = ((pitch_adjusted >> 0) & 0xff);
  data[8] = ((pitch_adjusted >> 8) & 0xff);

  // Speed yaw
  data[9] = ((yaw_speed_adjusted >> 0) & 0xff);
  data[10] = ((yaw_speed_adjusted >> 8) & 0xff);

  // Angle yaw
  data[11] = ((yaw_adjusted >> 0) & 0xff);
  data[12] = ((yaw_adjusted >> 8) & 0xff);

  // Build frame and send
  sbgc_frame_t frame;
  sbgc_frame_set_frame(&frame, SBGC_CMD_CONTROL, data, 13);
  sbgc_send(sbgc, &frame);

  return 0;
}

#endif // SBGC_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef SBGC_UNITTEST

#include <stdio.h>

#define SBGC_DEV "/dev/ttyUSB0"

// UNITESTS GLOBAL VARIABLES
static int nb_tests = 0;
static int nb_passed = 0;
static int nb_failed = 0;

#define ENABLE_TERM_COLORS 0
#if ENABLE_TERM_COLORS == 1
#define TERM_RED "\x1B[1;31m"
#define TERM_GRN "\x1B[1;32m"
#define TERM_WHT "\x1B[1;37m"
#define TERM_NRM "\x1B[1;0m"
#else
#define TERM_RED
#define TERM_GRN
#define TERM_WHT
#define TERM_NRM
#endif

/**
 * Run unittests
 * @param[in] test_name Test name
 * @param[in] test_ptr Pointer to unittest
 */
void run_test(const char *test_name, int (*test_ptr)()) {
  printf("-> [%s] ", test_name);

  if ((*test_ptr)() == 0) {
    printf(TERM_GRN "OK!\n" TERM_NRM);
    fflush(stdout);
    nb_passed++;
  } else {
    printf(TERM_RED "FAILED!\n" TERM_NRM);
    fflush(stdout);
    nb_failed++;
  }
  nb_tests++;
}

/**
 * Add unittest
 * @param[in] TEST Test function
 */
#define TEST(TEST_FN) run_test(#TEST_FN, TEST_FN);

/**
 * Unit-test assert
 * @param[in] TEST Test condition
 */
#define TEST_ASSERT(TEST)                                                      \
  do {                                                                         \
    if ((TEST) == 0) {                                                         \
      printf(TERM_RED "ERROR!" TERM_NRM " [%s:%d] %s FAILED!\n",               \
             __func__,                                                         \
             __LINE__,                                                         \
             #TEST);                                                           \
      return -1;                                                               \
    }                                                                          \
  } while (0)

int test_sbgc_connect_disconnect() {
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);
  TEST_ASSERT(sbgc_disconnect(&sbgc) == 0);
  return 0;
}

int test_sbgc_send() {
  // Connect
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);

  // Motors on
  sbgc_frame_t frame0;
  sbgc_frame_set_cmd(&frame0, SBGC_CMD_MOTORS_ON);
  TEST_ASSERT(sbgc_send(&sbgc, &frame0) == 0);
  sleep(1);

  // Motors off
  sbgc_frame_t frame1;
  sbgc_frame_set_cmd(&frame1, SBGC_CMD_MOTORS_OFF);
  TEST_ASSERT(sbgc_send(&sbgc, &frame1) == 0);
  sleep(1);

  return 0;
}

int test_sbgc_read() {
  // Connect
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV));

  // Send request
  sbgc_frame_t req;
  sbgc_frame_set_cmd(&req, SBGC_CMD_BOARD_INFO);
  sbgc_send(&sbgc, &req);

  // Get response
  sbgc_frame_t resp;
  sbgc_read(&sbgc, SBGC_CMD_BOARD_INFO_FRAME_SIZE, &resp);

  // Assert
  TEST_ASSERT(resp.data_size == 18);

  return 0;
}

int test_sbgc_info() {
  // Connect
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);

  // Get board info
  sbgc_info(&sbgc);
  printf("board version: %d\n", sbgc.board_version);
  printf("firmware version: %d\n", sbgc.firmware_version);
  printf("debug mode: %d\n", sbgc.debug_mode);
  printf("board features: %d\n", sbgc.board_features);
  printf("connection flags: %d\n", sbgc.connection_flags);

  return 0;
}

int test_sbgc_get_realtime_data() {
  // Connect
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);

  // Get imu data
  for (int i = 0; i < 100; ++i) {
    sbgc_get_realtime_data(&sbgc);
    sbgc_data_t_print(&sbgc.data);
  }

  return 0;
}

int test_sbgc_set_angle() {
  // Connect and turn motors on
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);
  TEST_ASSERT(sbgc_on(&sbgc) == 0);

  // Test Roll
  SBGC_INFO("Testing roll!");
  sbgc_set_angle(&sbgc, -20, 0, 0);
  sleep(2);

  for (int target_angle = -20; target_angle <= 20; target_angle += 5) {
    sbgc_set_angle(&sbgc, target_angle, 0, 0);
    sleep(2);

    sbgc_get_realtime_data(&sbgc);
    TEST_ASSERT((sbgc.data.camera_angles[0] - target_angle) < 2.0);
  }

  // Zero gimal
  SBGC_INFO("Zero-ing gimbal!");
  sbgc_set_angle(&sbgc, 0, 0, 0);
  sleep(2);

  // Test Pitch
  SBGC_INFO("Testing pitch!");
  for (int target_angle = 0; target_angle <= 20; target_angle += 5) {
    sbgc_set_angle(&sbgc, 0, target_angle, 0);
    sleep(2);

    sbgc_get_realtime_data(&sbgc);
    TEST_ASSERT((sbgc.data.camera_angles[1] - target_angle) < 2.0);
  }
  sbgc_off(&sbgc);

  return 0;
}

int test_sbgc_set_angle_speed() {
  // Connect and turn motors on
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);
  TEST_ASSERT(sbgc_on(&sbgc) == 0);

  // Set angle with speed
  TEST_ASSERT(sbgc_set_angle_speed(&sbgc, 0, 10, 0, 0, -2, 0) == 0);
  TEST_ASSERT(sbgc_off(&sbgc) == 0);
  sleep(3);

  return 0;
}

int main(int argc, char *argv[]) {
  // TEST(test_sbgc_connect_disconnect);
  // TEST(test_sbgc_send);
  // TEST(test_sbgc_read);
  TEST(test_sbgc_info);
  // TEST(test_sbgc_get_realtime_data);
  // TEST(test_sbgc_set_angle);
  // TEST(test_sbgc_set_angle_speed);

  return (nb_failed) ? -1 : 0;
}

#endif // SBGC_UNITTEST
