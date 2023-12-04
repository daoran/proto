#ifndef SBGC_H
#define SBGC_H

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define SBGC_INVERT_YAW 1

#ifndef SBGC_DEV
#define SBGC_DEV "/dev/ttyUSB0"
#endif
// MACROS
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
#define SBGC_DEG_PER_BIT 0.02197265625            // deg per bit
#define SBGC_DEG_SEC_PER_BIT 0.1220740379         // deg/sec per bit
#define SBGC_ACC_UNIT (1.0 / 512.0)               // G
#define SBGC_GYRO_UNIT 0.06103701895              // deg per sec
#define SBGC_COUNT_PER_DEG (360.0 / (pow(2, 24))) // counts per deg

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
#define SBGC_CMD_REALTIME_DATA_CUSTOM 88
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
#define SBGC_CMD_REALTIME_DATA_3_FRAME_SIZE 6 + 63
#define SBGC_CMD_REALTIME_DATA_4_FRAME_SIZE 6 + 63 + 61

// SBGC_CMD_REALTIME_DATA_3 Frame contents:
// Offset       Field              Field size
// [0, 4, 8]    ACC_DATA           2s * 3
// [2, 6, 10]   GYRO_DATA          2s * 3
// [12]         SERIAL_ERR_CNT     2u
// [14]         SYSTEM_ERROR       2u
// [16]         SYSTEM_SUB_ERROR   1u
// [18]         RESERVED           3b
// [20]         RC_ROLL            2s
// [22]         RC_PITCH           2s
// [24]         RC_YAW             2s
// [26]         RC_CMD             2s
// [28]         EXT_FC_ROLL        2s
// [30]         EXT_FC_PITCH       2s
// [32]         IMU_ANGLE          2s * 3
// [38]         FRAME_IMU_ANGLE    2s * 3
// [44]         TARGET_ANGLE       2s * 3
// [50]         CYCLE_TIME         2u
// [52]         I2C_ERROR_COUNT    2u
// [54]         ERROR_CODE         1u
// [55]         BAT_LEVEL          2u
// [57]         RT_DATA_FLAGS      1u
// [58]         CUR_IMU            1u
// [59]         CUR_PROFILE        1u
// [60]         MOTOR_POWER        1u * 3
// Total Payload Size: 63 Bytes

// SBGC_CMD_REALTIME_DATA_4 Frame contents:
// Offset       Field              Field size
// [63]         FRAME_CAM_ANGLE        2s * 3
// [69]         RESERVED               1b
// [70]         BALANCE_ERROR          2s * 3
// [76]         CURRENT                2u
// [78]         MAG_DATA               2s * 3
// [84]         IMU_TEMPERATURE        1s
// [85]         FRAME_IMU_TEMPERATURE  1s
// [86]         IMU_G_ERR              1u
// [87]         IMU_H_ERR              1u
// [88]         MOTOR_OUT              2s * 3
// [94]         CALIB_MODE             1u
// [95]         CAN_IMU_EXT_SENS_ERR   1u
// [96]         RESERVED               28b
// Total Payload Size: 63 + 61 = 124 Bytes

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

typedef struct sbgc_status_t {
  uint16_t serial_error_count;
  uint16_t system_error;
  uint16_t cycle_time;
  uint16_t i2c_error_count;
  uint16_t battery_level;
  uint8_t motors_on;
  uint8_t imu_selected;
  uint8_t profile_selected;
  uint8_t motor_power[3];
  int16_t balance_error[3];
  int current;
} sbgc_status_t;

typedef struct sbgc_t {
  int connected;
  const char *port;
  int serial;

  int64_t timestamp;
  float camera_angles[3];
  float target_angles[3];
  float encoder_angles[3];
  int32_t encoder_offsets[3];

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
void sbgc_status_setup(sbgc_status_t *data);
void sbgc_status_print(sbgc_status_t *data);

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
int sbgc_calib(sbgc_t *sbgc);
int sbgc_update(sbgc_t *sbgc);
int sbgc_get_status(const sbgc_t *sbgc, sbgc_status_t *status);
int sbgc_set_angle(sbgc_t *sbgc,
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
 * Get Unix timestamp since epoch in nano-seconds
 */
int64_t timestamp_now() {
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  return t.tv_sec * 1000000000LL + t.tv_nsec;
}

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

  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_cflag |= CS8 | CREAD | CLOCAL;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;

  // tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  // // disable IGNBRK for mismatched speed tests; otherwise receive break
  // // as \000 chars
  // tty.c_iflag &= ~IGNBRK; // disable break processing
  // tty.c_lflag = 0;        // no signaling chars, no echo,
  // // no canonical processing
  // tty.c_oflag = 0;     // no remapping, no delays
  // tty.c_cc[VMIN] = 0;  // read doesn't block
  // tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  // tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  // tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls,
  // // enable reading
  // tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  // tty.c_cflag |= parity;
  // tty.c_cflag &= ~CSTOPB;
  // tty.c_cflag &= ~CRTSCTS;

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
  return (int16_t) ((data[hi_byte] << 8) | (data[low_byte] & 0xff));
}

/**
 * Parse SBGC unsigned 16-bit
 */
uint16_t sbgc_u16bit(const uint8_t *data,
                     const int hi_byte,
                     const int low_byte) {
  return (uint16_t) ((data[hi_byte] << 8) | (data[low_byte] & 0xff));
}

/**
 * Parse SBGC signed 24-bit encoder raw data
 */
int32_t sbgc_parse_encoder_raw(const uint8_t *data, const int low_byte) {
  int32_t val = 0;
  val |= data[low_byte + 2] << 16;
  val |= data[low_byte + 1] << 8;
  val |= data[low_byte] << 0;
  return val;
}

/**
 * Parse SBGC signed 24-bit encoder angle
 */
float sbgc_parse_encoder(const uint8_t *data,
                         const int low_byte,
                         const int32_t offset) {
  // Convert from [0, 360] to [-180, 180]
  int32_t val = (sbgc_parse_encoder_raw(data, low_byte) - offset);
  if (val < 0) {
    val += (int32_t) pow(2, 24);
  }
  // int32_t val = sbgc_parse_encoder_raw(data, low_byte);
  float angle = fabs(val * SBGC_COUNT_PER_DEG);
  angle = fmod(angle + 180.0, 360.0) - 180.0;
  return angle;
}

/**
 * Parse Accelerometer data
 */
float sbgc_parse_accel(const uint8_t *data, const int low_byte) {
  return SBGC_ACC_UNIT * sbgc_s16bit(data, low_byte + 1, low_byte);
}

/**
 * Parse Gyroscope data
 */
float sbgc_parse_gyro(const uint8_t *data, const int low_byte) {
  return SBGC_GYRO_UNIT * sbgc_s16bit(data, low_byte + 1, low_byte);
}

/**
 * Parse Angle
 */
float sbgc_parse_angle(const uint8_t *data, const int low_byte) {
  return SBGC_DEG_PER_BIT * sbgc_s16bit(data, low_byte + 1, low_byte);
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

// SBGC STATUS /////////////////////////////////////////////////////////////////

/**
 * Setup SBGC data
 */
void sbgc_status_setup(sbgc_status_t *data) {
  data->serial_error_count = 0;
  data->system_error = 0;
  data->cycle_time = 0;
  data->i2c_error_count = 0;
  data->battery_level = 0;
  data->motors_on = 0;
  data->imu_selected = 0;
  data->profile_selected = 0;
  memset(data->motor_power, 0, sizeof(uint8_t) * 3);
  memset(data->motor_power, 0, sizeof(uint8_t) * 3);
  memset(data->balance_error, 0, sizeof(int16_t) * 3);
  data->current = 0;
}

/**
 * Print SBGC data
 */
void sbgc_status_print(sbgc_status_t *data) {
  printf("serial_error_count: %d\n", data->serial_error_count);
  printf("system_error: %d\n", data->system_error);
  printf("cycle_time: %d\n", data->cycle_time);
  printf("i2c_error_count: %d\n", data->i2c_error_count);
  printf("battery_level: %d\n\n", data->battery_level);
  printf("motors_on: %d\n", data->motors_on);
  printf("imu_selected: %d\n", data->imu_selected);
  printf("profile_selected: %d\n", data->profile_selected);
  printf("motor_power: [%d, %d, %d]\n",
         data->motor_power[0],
         data->motor_power[1],
         data->motor_power[2]);
  printf("balance_error: [%d, %d, %d]\n",
         data->balance_error[0],
         data->balance_error[1],
         data->balance_error[2]);
  printf("current: %d\n", data->current);
}

// SBGC //////////////////////////////////////////////////////////////////////

/**
 * Connect SBGC
 */
int sbgc_connect(sbgc_t *sbgc, const char *port) {
  sbgc->connected = 0;

  sbgc->port = port;
  sbgc->serial = -1;

  sbgc->timestamp = 0;
  sbgc->camera_angles[0] = 0.0f;
  sbgc->camera_angles[1] = 0.0f;
  sbgc->camera_angles[2] = 0.0f;
  sbgc->target_angles[0] = 0.0f;
  sbgc->target_angles[1] = 0.0f;
  sbgc->target_angles[2] = 0.0f;
  sbgc->encoder_angles[0] = 0.0f;
  sbgc->encoder_angles[1] = 0.0f;
  sbgc->encoder_angles[2] = 0.0f;
  // sbgc->encoder_offsets[0] = 0;
  // sbgc->encoder_offsets[1] = 0;
  // sbgc->encoder_offsets[2] = 0;
  sbgc->encoder_offsets[0] = 11889664;
  sbgc->encoder_offsets[1] = 6031360;
  sbgc->encoder_offsets[2] = 16063488;

  sbgc->board_version = 0;
  sbgc->firmware_version = 0;
  sbgc->state_flags = 0;
  sbgc->board_features = 0;
  sbgc->connection_flags = 0;

  // Open serial port
  sbgc->serial = open(sbgc->port, O_RDWR | O_NOCTTY | O_SYNC);
  sbgc->serial = open(sbgc->port, O_RDWR | O_NOCTTY | O_NDELAY);
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

  // Check
  uint16_t bytes = 0;
  ioctl(sbgc->serial, FIONREAD, &bytes);
  if (bytes == 0) {
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

int sbgc_calib(sbgc_t *sbgc) {
  // Swtich on gimbal
  SBGC_EXEC(sbgc_on(sbgc));
  sleep(3);

  // Request Encoder Data
  {
    uint32_t flags = (1 << 11);
    uint8_t payload[10] = {0};
    payload[0] = (flags >> 0) & 0xFF;
    payload[1] = (flags >> 8) & 0xFF;
    payload[2] = (flags >> 16) & 0xFF;
    payload[3] = (flags >> 24) & 0xFF;
    sbgc_frame_t frame;
    sbgc_frame_setup(&frame, SBGC_CMD_REALTIME_DATA_CUSTOM, payload, 10);
    SBGC_EXEC(sbgc_send(sbgc, &frame));
  }

  // Get response
  {
    uint8_t resp_size = 6 + 2 + 9;
    sbgc_frame_t resp;
    SBGC_EXEC(sbgc_read(sbgc, resp_size, &resp));
    const uint8_t *payload = resp.payload;
    sbgc->encoder_offsets[0] = sbgc_parse_encoder_raw(payload, 2);
    sbgc->encoder_offsets[1] = sbgc_parse_encoder_raw(payload, 5);
    sbgc->encoder_offsets[2] = sbgc_parse_encoder_raw(payload, 8);

    printf("encoder_offset[0]: %d\n", sbgc->encoder_offsets[0]);
    printf("encoder_offset[1]: %d\n", sbgc->encoder_offsets[1]);
    printf("encoder_offset[2]: %d\n", sbgc->encoder_offsets[2]);
  }

  // Switch off gimal
  SBGC_EXEC(sbgc_off(sbgc));

  return 0;
}

/**
 * Calibrate SBGC
 */
int sbgc_update(sbgc_t *sbgc) {
  // Whick data to return
  const uint8_t enable_imu_angles = 1;
  const uint8_t enable_target_angles = 1;
  const uint8_t enable_target_speed = 0;
  const uint8_t enable_frame_cam_angle = 0;
  const uint8_t enable_gyro_data = 0;
  const uint8_t enable_rc_data = 0;
  const uint8_t enable_z_vector = 0;
  const uint8_t enable_h_vector = 0;
  const uint8_t enable_rc_channels = 0;
  const uint8_t enable_acc_data = 0;
  const uint8_t enable_motor4_control = 0;
  const uint8_t enable_ahrs_debug_info = 0;
  const uint8_t enable_encoder_raw24 = 1;
  const uint8_t enable_imu_angles_rad = 0;
  const uint8_t enable_script_vars_float = 0;
  const uint8_t enable_script_vars_int16 = 0;
  const uint8_t enable_system_power_state = 0;
  const uint8_t enable_frame_cam_rate = 0;
  const uint8_t enable_imu_angles_20 = 0;
  const uint8_t enable_target_angles_20 = 0;

  // Response size
  uint8_t resp_size = 0;
  resp_size = 6;  // Header
  resp_size += 2; // Timestamp
  resp_size += (enable_imu_angles) ? 6 : 0;
  resp_size += (enable_target_angles) ? 6 : 0;
  resp_size += (enable_target_speed) ? 6 : 0;
  resp_size += (enable_frame_cam_angle) ? 6 : 0;
  resp_size += (enable_gyro_data) ? 6 : 0;
  resp_size += (enable_rc_data) ? 12 : 0;
  resp_size += (enable_z_vector) ? 12 : 0;
  resp_size += (enable_h_vector) ? 12 : 0;
  resp_size += (enable_rc_channels) ? 36 : 0;
  resp_size += (enable_acc_data) ? 6 : 0;
  resp_size += (enable_motor4_control) ? 8 : 0;
  resp_size += (enable_ahrs_debug_info) ? 26 : 0;
  resp_size += (enable_encoder_raw24) ? 9 : 0;
  resp_size += (enable_imu_angles_rad) ? 12 : 0;
  resp_size += (enable_script_vars_float) ? 12 : 0;
  resp_size += (enable_script_vars_int16) ? 6 : 0;
  resp_size += (enable_frame_cam_rate) ? 6 : 0;
  resp_size += (enable_imu_angles_20) ? 12 : 0;
  resp_size += (enable_target_angles_20) ? 12 : 0;

  // Request custom real time data
  {
    uint32_t flags = 0;
    flags |= (enable_imu_angles << 0);
    flags |= (enable_target_angles << 1);
    flags |= (enable_target_speed << 2);
    flags |= (enable_frame_cam_angle << 3);
    flags |= (enable_gyro_data << 4);
    flags |= (enable_rc_data << 5);
    flags |= (enable_z_vector << 6);
    flags |= (enable_rc_channels << 7);
    flags |= (enable_acc_data << 8);
    flags |= (enable_motor4_control << 9);
    flags |= (enable_ahrs_debug_info << 10);
    flags |= (enable_encoder_raw24 << 11);
    flags |= (enable_imu_angles_rad << 12);
    flags |= (enable_script_vars_float << 13);
    flags |= (enable_script_vars_int16 << 14);
    flags |= (enable_system_power_state << 15);
    flags |= (enable_frame_cam_rate << 16);
    flags |= (enable_imu_angles_20 << 17);
    flags |= (enable_target_angles_20 << 18);

    uint8_t payload[10] = {0};
    payload[0] = (flags >> 0) & 0xFF;
    payload[1] = (flags >> 8) & 0xFF;
    payload[2] = (flags >> 16) & 0xFF;
    payload[3] = (flags >> 24) & 0xFF;

    sbgc_frame_t frame;
    sbgc_frame_setup(&frame, SBGC_CMD_REALTIME_DATA_CUSTOM, payload, 10);
    SBGC_EXEC(sbgc_send(sbgc, &frame));
  }

  // Get response
  sbgc_frame_t resp;
  SBGC_EXEC(sbgc_read(sbgc, resp_size, &resp));

  // -- Parse real time data
  // clang-format off
  const uint8_t *payload = resp.payload;
  float imu_roll = sbgc_parse_angle(payload, 2);
  float imu_pitch = sbgc_parse_angle(payload, 4);
  float imu_yaw = sbgc_parse_angle(payload, 6);
  float target_roll = sbgc_parse_angle(payload, 8);
  float target_pitch = sbgc_parse_angle(payload, 10);
  float target_yaw = sbgc_parse_angle(payload, 12);
  float encoder_roll = sbgc_parse_encoder(payload, 14, sbgc->encoder_offsets[0]);
  float encoder_pitch = sbgc_parse_encoder(payload, 17, sbgc->encoder_offsets[1]);
  float encoder_yaw = sbgc_parse_encoder(payload, 20, sbgc->encoder_offsets[2]);
  // clang-format on

  // Invert Yaw?
  imu_yaw = SBGC_INVERT_YAW ? -1 * imu_yaw : imu_yaw;
  target_yaw = SBGC_INVERT_YAW ? -1 * target_yaw : target_yaw;

  // -- Get timestamp
  const int64_t timestamp = timestamp_now();

  sbgc->timestamp = timestamp;
  sbgc->camera_angles[0] = imu_roll;
  sbgc->camera_angles[1] = imu_pitch;
  sbgc->camera_angles[2] = imu_yaw;
  sbgc->target_angles[0] = target_roll;
  sbgc->target_angles[1] = target_pitch;
  sbgc->target_angles[2] = target_yaw;
  sbgc->encoder_angles[0] = encoder_roll;
  sbgc->encoder_angles[1] = encoder_pitch;
  sbgc->encoder_angles[2] = encoder_yaw;

  printf("\n");
  printf("timestamp: %ld\n", timestamp);
  printf("imu:     %f, %f, %f\n", imu_roll, imu_pitch, imu_yaw);
  printf("target:  %f, %f, %f\n", target_roll, target_pitch, target_yaw);
  printf("encoder: %f, %f, %f\n", encoder_roll, encoder_pitch, encoder_yaw);

  return 0;
}

/**
 * SBGC status
 */
int sbgc_get_status(const sbgc_t *sbgc, sbgc_status_t *status) {
  // Request real time data
  sbgc_frame_t frame;
  sbgc_frame_setup(&frame, SBGC_CMD_REALTIME_DATA_4, NULL, 0);
  SBGC_EXEC(sbgc_send(sbgc, &frame));
  usleep(10 * 1000);

  // Obtain real time data
  sbgc_frame_t info;
  SBGC_EXEC(sbgc_read(sbgc, SBGC_CMD_REALTIME_DATA_4_FRAME_SIZE, &info));

  // Parse real time data
  const uint8_t *payload = info.payload;
  // status->accel[0] = sbgc_parse_accel(payload, 0);
  // status->accel[1] = sbgc_parse_accel(payload, 4);
  // status->accel[2] = sbgc_parse_accel(payload, 8);
  // status->gyro[0] = sbgc_parse_gyro(payload, 2);
  // status->gyro[1] = sbgc_parse_gyro(payload, 6);
  // status->gyro[2] = sbgc_parse_gyro(payload, 10);
  // status->camera_angles[0] = sbgc_parse_angle(payload, 32);
  // status->camera_angles[1] = sbgc_parse_angle(payload, 34);
  // status->camera_angles[2] = sbgc_parse_angle(payload, 36);
  // status->frame_angles[0] = sbgc_parse_angle(payload, 38);
  // status->frame_angles[1] = sbgc_parse_angle(payload, 40);
  // status->frame_angles[2] = sbgc_parse_angle(payload, 42);
  // status->target_angles[0] = sbgc_parse_angle(payload, 44);
  // status->target_angles[1] = sbgc_parse_angle(payload, 45);
  // status->target_angles[2] = sbgc_parse_angle(payload, 46);
  // status->frame_cam_angles[0] = sbgc_parse_angle(payload, 63);
  // status->frame_cam_angles[1] = sbgc_parse_angle(payload, 65);
  // status->frame_cam_angles[2] = sbgc_parse_angle(payload, 67);
  status->serial_error_count = sbgc_u16bit(payload, 13, 12);
  status->system_error = sbgc_u16bit(payload, 15, 14);
  status->cycle_time = sbgc_u16bit(payload, 51, 50);
  status->i2c_error_count = sbgc_u16bit(payload, 53, 52);
  status->battery_level = sbgc_u16bit(payload, 56, 55);
  status->motors_on = payload[57];
  status->imu_selected = payload[58];
  status->profile_selected = payload[59];
  status->motor_power[0] = payload[60];
  status->motor_power[1] = payload[61];
  status->motor_power[2] = payload[62];
  status->balance_error[0] = sbgc_s16bit(payload, 71, 70);
  status->balance_error[1] = sbgc_s16bit(payload, 73, 72);
  status->balance_error[2] = sbgc_s16bit(payload, 75, 74);
  status->current = sbgc_u16bit(payload, 77, 76);

  return 0;
}

/**
 * Set SBGC angle
 */
int sbgc_set_angle(sbgc_t *sbgc,
                   const float roll,
                   const float pitch,
                   const float yaw) {
  // Get SBGC data (if too old)
  const int64_t ts_now = timestamp_now();
  const float time_diff = (ts_now - sbgc->timestamp) * 1e-9;
  if (time_diff > 0.05) {
    int retry_limit = 5;
    int success = 0;
    for (int i = 0; i < retry_limit; i++) {
      if (sbgc_update(sbgc) == 0) {
        success = 1;
        break;
      }
    }
    if (success == 0) {
      return -1;
    }
  }

  // Adjust roll, pitch and yaw
  double yaw_error = yaw - sbgc->encoder_angles[2];
  const int16_t roll_adjusted = roll / SBGC_DEG_PER_BIT;
  const int16_t pitch_adjusted = pitch / SBGC_DEG_PER_BIT;
  const int16_t yaw_adjusted =
      (sbgc->camera_angles[2] + (sbgc->encoder_angles[2] - yaw)) /
      SBGC_DEG_PER_BIT;
      -(sbgc->camera_angles[2] + yaw_error) / SBGC_DEG_PER_BIT;

  // printf("camera_angle:  %f\n", sbgc->camera_angles[2]);
  // printf("encoder_angle: %f\n", sbgc->encoder_angles[2]);
  // printf("yaw_adjusted:  %f\n",
  //        (sbgc->camera_angles[2] + (sbgc->encoder_angles[2] - yaw)));

  // Control mode
  uint8_t data[13];
  data[0] = SBGC_MODE_ANGLE;
  // data[0] = SBGC_MODE_ANGLE_REL_FRAME;

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

int test_sbgc_on_off() {
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);
  TEST_ASSERT(sbgc_on(&sbgc) == 0);
  TEST_ASSERT(sbgc_off(&sbgc) == 0);
  return 0;
}

int test_sbgc_send() {
  // Connect
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);

  // Motors on
  // sbgc_frame_t frame0;
  // sbgc_frame_set_cmd(&frame0, SBGC_CMD_MOTORS_ON);
  // TEST_ASSERT(sbgc_send(&sbgc, &frame0) == 0);
  // sleep(1);

  // Motors off
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

  printf("board version: %d\n", sbgc.board_version);
  printf("firmware version: %d\n", sbgc.firmware_version);
  printf("state_flags: %d\n", sbgc.state_flags);
  printf("board features: %d\n", sbgc.board_features);
  printf("connection flags: %d\n", sbgc.connection_flags);

  return 0;
}

int test_sbgc_calib() {
  // Connect
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);

  // Calibrate Gimbal
  sbgc_calib(&sbgc);

  return 0;
}

int test_sbgc_update() {
  // Connect
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);
  TEST_ASSERT(sbgc_on(&sbgc) == 0);

  // Get board info
  struct timespec tic;
  clock_gettime(CLOCK_MONOTONIC, &tic);
  while (1) {
    sbgc_update(&sbgc);

    struct timespec toc;
    clock_gettime(CLOCK_MONOTONIC, &toc);
    float time_elasped = (toc.tv_sec - tic.tv_sec);
    time_elasped += (toc.tv_nsec - tic.tv_nsec) / 1000000000.0;
    clock_gettime(CLOCK_MONOTONIC, &tic);

    printf("time_elasped: %.2f [s]\n", time_elasped);
  }

  return 0;
}

int test_sbgc_get_status() {
  // Connect
  sbgc_t sbgc;
  sbgc_status_t status;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);
  // TEST_ASSERT(sbgc_on(&sbgc) == 0);

  // Get imu data
  for (int i = 0; i < 100; ++i) {
    int retval = sbgc_get_status(&sbgc, &status);
    if (retval != 0) {
      printf("retval: %d\n", retval);
      return -1;
    }
    sbgc_status_print(&status);
  }

  return 0;
}

int test_sbgc_set_angle() {
  // Connect and turn motors on
  sbgc_t sbgc;
  TEST_ASSERT(sbgc_connect(&sbgc, SBGC_DEV) == 0);
  TEST_ASSERT(sbgc_on(&sbgc) == 0);
  TEST_ASSERT(sbgc_off(&sbgc) == 0);

  // // Zero gimal
  // SBGC_INFO("Zero-ing gimbal!");
  // while (1) {
  //   sbgc_set_angle(&sbgc, 0, 0, 0);
  //   printf("\n");
  //   printf("camera_angle:  %f\n", sbgc.camera_angles[2]);
  //   printf("encoder_angle: %f\n", sbgc.encoder_angles[2]);
  //   usleep(10 * 1000);
  // }

  // // Test Roll
  // SBGC_INFO("Testing roll!");
  // for (int target_angle = -35; target_angle <= 35; target_angle += 10) {
  //   sbgc_set_angle(&sbgc, target_angle, 0, 0);
  //   printf("Setting roll to %d\n", target_angle);
  //   fflush(stdout);
  //   sleep(2);
  // }

  // // Zero gimal
  // SBGC_INFO("Zero-ing gimbal!");
  // sbgc_set_angle(&sbgc, 0, 0, 0);
  // sleep(2);

  // // Test Pitch
  // SBGC_INFO("Testing pitch!");
  // for (int target_angle = 0; target_angle <= 40; target_angle += 10) {
  //   sbgc_set_angle(&sbgc, 0, target_angle, 0);
  //   printf("Setting pitch to %d\n", target_angle);
  //   fflush(stdout);
  //   sleep(2);
  // }

  // // Test Pitch
  // SBGC_INFO("Testing yaw!");
  // for (int target_angle = -30; target_angle <= 30; target_angle += 10) {
  //   sbgc_set_angle(&sbgc, 0, 0, target_angle);
  //   printf("Setting yaw to %d\n", target_angle);
  //   fflush(stdout);
  //   sleep(2);
  // }

  // // Zero gimal
  // SBGC_INFO("Zero-ing gimbal!");
  // sbgc_set_angle(&sbgc, 0, 0, 0);
  // sleep(2);

  // Switch off
  // sbgc_off(&sbgc);

  return 0;
}

int main(int argc, char *argv[]) {
  // TEST(test_sbgc_connect_disconnect);
  // TEST(test_sbgc_on_off);
  // TEST(test_sbgc_send);
  // TEST(test_sbgc_read);
  // TEST(test_sbgc_info);
  // TEST(test_sbgc_calib);
  TEST(test_sbgc_update);
  // TEST(test_sbgc_get_status);
  // TEST(test_sbgc_set_angle);

  return (nb_failed) ? -1 : 0;
}

#endif // SBGC_UNITTEST
