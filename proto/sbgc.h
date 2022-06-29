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

#define SBGC_EXEC(FN)                                                          \
  do {                                                                         \
    int retval = (FN);                                                         \
    if (retval != 0) {                                                         \
      return retval;                                                           \
    }                                                                          \
  } while (0)

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
#define SBGC_CMD_BOARD_INFO_FRAME_SIZE 6 + 18
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

// SBGC ERRORS
#define SBGC_NOT_CONNECTED -1
#define SBGC_DISCONNECT_FAILED -2
#define SBGC_INVALID_START -3
#define SBGC_INVALID_HEADER -4
#define SBGC_INVALID_PAYLOAD_SIZE -5
#define SBGC_INVALID_CRC16 -6
#define SBGC_SEND_FAILED -7
#define SBGC_READ_FAILED -8

typedef struct sbgc_frame_t {
  uint8_t cmd_id;
  uint8_t payload_size;
  uint8_t header_checksum;
  uint8_t payload[256];
  uint8_t crc16[2];

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
  uint8_t state_flags;
  uint16_t board_features;
  uint8_t connection_flags;
} sbgc_t;

void sbgc_frame_print(const sbgc_frame_t *frame);
void sbgc_frame_setup(sbgc_frame_t *frame,
                      uint8_t cmd_id,
                      uint8_t *payload,
                      uint8_t payload_size);
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
int sbgc_update(sbgc_t *sbgc);
int sbgc_set_angle(const sbgc_t *sbgc,
                   const float roll,
                   const float pitch,
                   const float yaw);
#endif // SBGC_H

//////////////////////////////////////////////////////////////////////////////
//                             IMPLEMENTATION                               //
//////////////////////////////////////////////////////////////////////////////

#ifdef SBGC_IMPLEMENTATION

// SBGC UTILS ////////////////////////////////////////////////////////////////

/**
 * Set SBGC UART interface
 */
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

/**
 * Set SBGC serial as blocking
 */
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

/**
 * Parse SBGC signed 16-bit
 */
int16_t sbgc_s16bit(const uint8_t *data,
                    const int hi_byte,
                    const int low_byte) {
  return (int16_t)((data[hi_byte] << 8) | (data[low_byte] & 0xff));
}

/**
 * Parse SBGC unsigned 16-bit
 */
uint16_t sbgc_u16bit(const uint8_t *data,
                     const int hi_byte,
                     const int low_byte) {
  return (uint16_t)((data[hi_byte] << 8) | (data[low_byte] & 0xff));
}

/**
 * Parse Accelerometer data
 */
float sbgc_parse_accel(const uint8_t *data,
                       const int hi_byte,
                       const int low_byte) {
  return SBGC_ACC_UNIT * sbgc_s16bit(data, hi_byte, low_byte);
}

/**
 * Parse Gyroscope data
 */
float sbgc_parse_gyro(const uint8_t *data,
                      const int hi_byte,
                      const int low_byte) {
  return SBGC_GYRO_UNIT * sbgc_s16bit(data, hi_byte, low_byte);
}

/**
 * Parse Angle
 */
float sbgc_parse_angle(const uint8_t *data,
                       const int hi_byte,
                       const int low_byte) {
  return SBGC_DEG_PER_BIT * sbgc_s16bit(data, hi_byte, low_byte);
}

/**
 * SBGC CRC16 checksum
 */
void sbgc_crc16(const uint8_t *payload,
                const uint8_t payload_size,
                uint8_t crc[2]) {
  crc[0] = 0.0;
  crc[1] = 0.0;

  uint16_t polynom = 0x8005;
  uint16_t crc_reg = (uint16_t) crc[0] | ((uint16_t) crc[1] << 8);

  for (uint8_t i = 0; i < payload_size; i++) {
    for (uint8_t shift_reg = 0x01; shift_reg > 0x00; shift_reg <<= 1) {
      uint8_t data_bit = (payload[i] & shift_reg) ? 1 : 0;
      uint8_t crc_bit = crc_reg >> 15;
      crc_reg <<= 1;

      if (data_bit != crc_bit)
        crc_reg ^= polynom;
    }
  }

  crc[0] = crc_reg;
  crc[1] = (crc_reg >> 8);
}

// SBGC FRAME ////////////////////////////////////////////////////////////////

/**
 * Print SBGC frame
 */
void sbgc_frame_print(const sbgc_frame_t *frame) {
  int i;

  // Header
  printf("[%d]: 0x%x\n", 0, '$');
  printf("[%d]: 0x%x\n", 1, frame->cmd_id);
  printf("[%d]: 0x%x\n", 2, frame->payload_size);
  printf("[%d]: 0x%x\n", 3, frame->header_checksum);

  // Payload
  for (i = 4; i < (frame->payload_size + 4); i++) {
    printf("[%d]: 0x%x\n", i, frame->payload[i]);
  }

  // CRC16
  printf("[%d]: 0x%x\n", (frame->payload_size + 4 + 1), frame->crc16[0]);
  printf("[%d]: 0x%x\n", (frame->payload_size + 4 + 2), frame->crc16[1]);
}

/**
 * Setup SBGC frame
 */
void sbgc_frame_setup(sbgc_frame_t *frame,
                      uint8_t cmd_id,
                      uint8_t *payload,
                      uint8_t payload_size) {
  // Header
  frame->cmd_id = cmd_id;
  frame->payload_size = payload_size;
  frame->header_checksum = (frame->cmd_id + frame->payload_size) % 256;

  // Payload
  for (uint8_t i = 0; i < payload_size; i++) {
    frame->payload[i] = payload[i];
  }

  // CRC16
  const uint16_t bitstr_len = frame->payload_size + 3;
  uint8_t bitstr[256] = {0};
  bitstr[0] = frame->cmd_id;
  bitstr[1] = frame->payload_size;
  bitstr[2] = frame->header_checksum;
  for (uint8_t i = 0; i < frame->payload_size; i++) {
    bitstr[3 + i] = frame->payload[i];
  }
  sbgc_crc16(bitstr, bitstr_len, frame->crc16);
}

/**
 * Print SBGC frame
 */
int sbgc_frame_parse_frame(sbgc_frame_t *frame, uint8_t *data) {
  // Pre-check
  if (data[0] != '$') {
    return SBGC_INVALID_START;
  }

  // Parse header
  frame->cmd_id = data[1];
  frame->payload_size = data[2];
  frame->header_checksum = data[3];
  uint8_t expected_checksum = (frame->cmd_id + frame->payload_size) % 256;
  if (frame->header_checksum != expected_checksum) {
    return SBGC_INVALID_HEADER;
  }

  // Parse payload
  for (uint8_t i = 0; i < frame->payload_size; i++) {
    frame->payload[i] = data[4 + i];
  }

  // CRC16
  // -- Calculate expected CRC16
  const uint16_t bitstr_len = frame->payload_size + 3;
  uint8_t bitstr[256] = {0};
  bitstr[0] = frame->cmd_id;
  bitstr[1] = frame->payload_size;
  bitstr[2] = frame->header_checksum;
  for (uint8_t i = 0; i < frame->payload_size; i++) {
    bitstr[3 + i] = frame->payload[i];
  }
  uint8_t crc16[2] = {0};
  sbgc_crc16(bitstr, bitstr_len, crc16);
  // -- Verify with obtained CRC16
  frame->crc16[0] = data[4 + frame->payload_size];
  frame->crc16[1] = data[4 + frame->payload_size + 1];
  if (crc16[0] != frame->crc16[0] || crc16[1] != frame->crc16[1]) {
    return SBGC_INVALID_CRC16;
  }

  return 0;
}

// SBGC DATA /////////////////////////////////////////////////////////////////

/**
 * Print SBGC data
 */
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

/**
 * Connect SBGC
 */
int sbgc_connect(sbgc_t *sbgc, const char *port) {
  sbgc->connected = 0;

  sbgc->port = port;
  sbgc->serial = -1;

  sbgc->board_version = 0;
  sbgc->firmware_version = 0;
  sbgc->state_flags = 0;
  sbgc->board_features = 0;
  sbgc->connection_flags = 0;

  // Open serial port
  sbgc->serial = open(sbgc->port, O_RDWR | O_NOCTTY | O_SYNC);
  if (sbgc->serial == -1) {
    printf("Oh dear, open()! %s\n", strerror(errno));
    return SBGC_NOT_CONNECTED;
  }

  // Configure serial commnication
  sbgc_set_interface_attributes(sbgc->serial, B115200, 0);
  sbgc_set_blocking(sbgc->serial, 1);
  sbgc->connected = 1;

  return 0;
}

/**
 * Disconnect SBGC
 */
int sbgc_disconnect(sbgc_t *sbgc) {
  if (close(sbgc->serial) != 0) {
    return SBGC_DISCONNECT_FAILED;
  }

  sbgc->connected = 0;
  return 0;
}

/**
 * Reset SBGC
 */
void sbgc_reset(sbgc_t *sbgc) {
  sbgc->connected = 0;

  sbgc->port = NULL;
  sbgc->serial = -1;

  sbgc->board_version = 0;
  sbgc->firmware_version = 0;
  sbgc->state_flags = 0;
  sbgc->board_features = 0;
  sbgc->connection_flags = 0;
}

/**
 * Send SBGC frame
 */
int sbgc_send(const sbgc_t *sbgc, const sbgc_frame_t *frame) {
  // Check connection
  if (sbgc->connected == 0) {
    return SBGC_NOT_CONNECTED;
  }

  // Check command data size
  int payload_size_limit;
  payload_size_limit = SBGC_CMD_MAX_BYTES - SBGC_CMD_PAYLOAD_BYTES;
  if (frame->payload_size >= payload_size_limit) {
    return SBGC_INVALID_PAYLOAD_SIZE;
  }

  // Header
  ssize_t retval = 0;
  const uint8_t start = 0x24; // "$" character
  retval += write(sbgc->serial, &start, 1);
  retval += write(sbgc->serial, &frame->cmd_id, 1);
  retval += write(sbgc->serial, &frame->payload_size, 1);
  retval += write(sbgc->serial, &frame->header_checksum, 1);

  // Payload
  retval += write(sbgc->serial, frame->payload, frame->payload_size);
  retval += write(sbgc->serial, frame->crc16, 2);

  // Flush
  tcflush(sbgc->serial, TCIOFLUSH); // VERY CRITICAL
  usleep(10 * 1000);
  if (retval != (6 + frame->payload_size)) {
    return SBGC_SEND_FAILED;
  }

  return 0;
}

/**
 * Read SBGC frame
 */
int sbgc_read(const sbgc_t *sbgc,
              const uint8_t read_length,
              sbgc_frame_t *frame) {
  // Check connection
  if (sbgc->connected == 0) {
    return -1;
  }

  // Send query
  uint8_t buf[256] = {0};
  int16_t nb_bytes = read(sbgc->serial, buf, read_length);
  if (nb_bytes <= 0 || nb_bytes != read_length) {
    return SBGC_READ_FAILED;
  }

  // Parse sbgc frame
  int retval = sbgc_frame_parse_frame(frame, buf);
  if (retval != 0) {
    return retval;
  }

  return 0;
}

/**
 * Switch SBGC on
 */
int sbgc_on(const sbgc_t *sbgc) {
  sbgc_frame_t frame;
  sbgc_frame_setup(&frame, SBGC_CMD_MOTORS_ON, NULL, 0);
  return sbgc_send(sbgc, &frame);
}

/**
 * Switch SBGC off
 */
int sbgc_off(const sbgc_t *sbgc) {
  // Check connection
  if (sbgc->connected == 0) {
    return SBGC_NOT_CONNECTED;
  }

  // Turn off motor control
  {
    uint8_t data[13];
    data[0] = SBGC_MODE_NO_CONTROL;
    for (int i = 1; i < 13; i++) {
      data[i] = 0;
    }

    sbgc_frame_t frame;
    sbgc_frame_setup(&frame, SBGC_CMD_CONTROL, data, 13);
    SBGC_EXEC(sbgc_send(sbgc, &frame));
  }

  // Turn off motors
  {
    sbgc_frame_t frame;
    sbgc_frame_setup(&frame, SBGC_CMD_MOTORS_OFF, NULL, 0);
    SBGC_EXEC(sbgc_send(sbgc, &frame));
  }

  return 0;
}

/**
 * Obtain SBGC board information
 */
int sbgc_info(sbgc_t *sbgc) {
  // Request board info
  sbgc_frame_t frame;
  sbgc_frame_setup(&frame, SBGC_CMD_BOARD_INFO, NULL, 0);
  SBGC_EXEC(sbgc_send(sbgc, &frame));

  // Obtain board info
  sbgc_frame_t info;
  SBGC_EXEC(sbgc_read(sbgc, SBGC_CMD_BOARD_INFO_FRAME_SIZE, &info));

  // Parse info
  sbgc->board_version = info.payload[0];
  sbgc->firmware_version = (info.payload[2] << 8) | (info.payload[1] & 0xff);
  sbgc->state_flags = info.payload[3];
  sbgc->board_features = (info.payload[5] << 8) | (info.payload[4] & 0xff);
  sbgc->connection_flags = info.payload[6];

  return 0;
}

/**
 * SBGC update
 */
int sbgc_update(sbgc_t *sbgc) {
  // Request real time data
  sbgc_frame_t frame;
  sbgc_frame_setup(&frame, SBGC_CMD_REALTIME_DATA_3, NULL, 0);
  SBGC_EXEC(sbgc_send(sbgc, &frame));

  // Obtain real time data
  sbgc_frame_t info;
  SBGC_EXEC(sbgc_read(sbgc, 69, &info));

  // Parse real time data
  sbgc->data.accel[0] = sbgc_parse_accel(frame.payload, 1, 0);
  sbgc->data.accel[1] = sbgc_parse_accel(frame.payload, 5, 4);
  sbgc->data.accel[2] = sbgc_parse_accel(frame.payload, 9, 8);

  sbgc->data.gyro[0] = sbgc_parse_gyro(frame.payload, 3, 2);
  sbgc->data.gyro[1] = sbgc_parse_gyro(frame.payload, 7, 6);
  sbgc->data.gyro[2] = sbgc_parse_gyro(frame.payload, 11, 10);

  sbgc->data.camera_angles[0] = sbgc_parse_angle(frame.payload, 33, 32);
  sbgc->data.camera_angles[1] = sbgc_parse_angle(frame.payload, 35, 34);
  sbgc->data.camera_angles[2] = sbgc_parse_angle(frame.payload, 37, 36);

  sbgc->data.frame_angles[0] = sbgc_parse_angle(frame.payload, 39, 38);
  sbgc->data.frame_angles[1] = sbgc_parse_angle(frame.payload, 41, 40);
  sbgc->data.frame_angles[2] = sbgc_parse_angle(frame.payload, 43, 42);

  sbgc->data.rc_angles[0] = sbgc_parse_angle(frame.payload, 45, 44);
  sbgc->data.rc_angles[1] = sbgc_parse_angle(frame.payload, 47, 45);
  sbgc->data.rc_angles[2] = sbgc_parse_angle(frame.payload, 49, 46);

  sbgc->data.cycle_time = sbgc_u16bit(frame.payload, 51, 50);
  sbgc->data.i2c_error_count = sbgc_u16bit(frame.payload, 53, 52);
  sbgc->data.system_error = sbgc_u16bit(frame.payload, 15, 14);
  sbgc->data.battery_level = sbgc_u16bit(frame.payload, 56, 55);

  return 0;
}

/**
 * Set SBGC angle
 */
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
  sbgc_frame_setup(&frame, SBGC_CMD_CONTROL, data, 13);
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
  if ((*test_ptr)() == 0) {
    printf("-> [%s] " TERM_GRN "OK!\n" TERM_NRM, test_name);
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

  // // Motors on
  // sbgc_frame_t frame0;
  // sbgc_frame_set_cmd(&frame0, SBGC_CMD_MOTORS_ON);
  // TEST_ASSERT(sbgc_send(&sbgc, &frame0) == 0);
  // sleep(1);

  // // Motors off
  // sbgc_frame_t frame1;
  // sbgc_frame_set_cmd(&frame1, SBGC_CMD_MOTORS_OFF);
  // TEST_ASSERT(sbgc_send(&sbgc, &frame1) == 0);
  // sleep(1);

  return 0;
}

int test_sbgc_read() {
  // Connect
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);

  // Send request
  sbgc_frame_t req;
  sbgc_frame_setup(&req, SBGC_CMD_BOARD_INFO, NULL, 0);
  TEST_ASSERT(sbgc_send(&sbgc, &req) == 0);

  // Get response
  sbgc_frame_t resp;
  TEST_ASSERT(sbgc_read(&sbgc, SBGC_CMD_BOARD_INFO_FRAME_SIZE, &resp) == 0);

  // Assert
  TEST_ASSERT(resp.payload_size == 18);

  return 0;
}

int test_sbgc_info() {
  // Connect
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);

  // Get board info
  sbgc_info(&sbgc);

  // printf("board version: %d\n", sbgc.board_version);
  // printf("firmware version: %d\n", sbgc.firmware_version);
  // printf("state_flags: %d\n", sbgc.state_flags);
  // printf("board features: %d\n", sbgc.board_features);
  // printf("connection flags: %d\n", sbgc.connection_flags);

  return 0;
}

int test_sbgc_update() {
  // Connect
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);
  TEST_ASSERT(sbgc_on(&sbgc) == 0);

  // Get imu data
  for (int i = 0; i < 100; ++i) {
    int retval = sbgc_update(&sbgc);
    if (retval != 0) {
      printf("retval: %d\n", retval);
      return -1;
    }
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

    sbgc_update(&sbgc);
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

    sbgc_update(&sbgc);
    TEST_ASSERT((sbgc.data.camera_angles[1] - target_angle) < 2.0);
  }
  sbgc_off(&sbgc);

  return 0;
}

int main(int argc, char *argv[]) {
  TEST(test_sbgc_connect_disconnect);
  TEST(test_sbgc_send);
  TEST(test_sbgc_read);
  TEST(test_sbgc_info);
  TEST(test_sbgc_update);
  // TEST(test_sbgc_set_angle);

  return (nb_failed) ? -1 : 0;
}

#endif // SBGC_UNITTEST
