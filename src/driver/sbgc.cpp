#include "prototype/driver/sbgc.hpp"

namespace prototype {

void sbgc_frame_print(const sbgc_frame_t &frame) {
  int i;

  // print header
  printf("[%d]: %c\n", 0, '>');
  printf("[%d]: %c\n", 1, frame.cmd_id);
  printf("[%d]: %d\n", 2, frame.data_size);
  printf("[%d]: %d\n", 3, frame.header_checksum);

  // print body
  for (i = 4; i < (frame.data_size + 4); i++) {
    printf("[%d]: %d\n", i, frame.data[i]);
  }
  printf("[%d]: %d\n", i, frame.data_checksum);
}

void sbgc_frame_set_header(sbgc_frame_t &frame,
                           uint8_t cmd_id,
                           uint8_t data_size) {
  frame.cmd_id = cmd_id;
  frame.data_size = data_size;
  frame.header_checksum = (frame.cmd_id + frame.data_size) % 256;
}

void sbgc_frame_set_checksum(sbgc_frame_t &frame) {
  frame.data_checksum = 0x0;
  for (int i = 0; i < frame.data_size; i++) {
    frame.data_checksum += frame.data[i];
  }
  frame.data_checksum = frame.data_checksum % 256;
}

void sbgc_frame_set_body(sbgc_frame_t &frame, uint8_t *data) {
  frame.data = data;
  sbgc_frame_set_checksum(frame);
}

void sbgc_frame_build(sbgc_frame_t &frame,
                      int cmd_id,
                      uint8_t *data,
                      int data_size) {
  sbgc_frame_set_header(frame, (uint8_t) cmd_id, (uint8_t) data_size);
  sbgc_frame_set_body(frame, data);
  frame.ok = true;
}

int sbgc_frame_parse_header(sbgc_frame_t &frame, uint8_t *data) {
  uint8_t expected_checksum;

  // Pre-check
  if (data[0] != '>') {
    return -1;
  }

  // Parse header
  frame.cmd_id = data[1];
  frame.data_size = data[2];
  frame.header_checksum = data[3];

  // Check the header checksum
  expected_checksum = (frame.cmd_id + frame.data_size) % 256;
  if (frame.header_checksum != expected_checksum) {
    return -1;
  }

  return 0;
}

int sbgc_frame_parse_body(sbgc_frame_t &frame, uint8_t *data) {
  uint8_t i;
  uint8_t expected_checksum;

  // Setup
  expected_checksum = 0x0;
  frame.data = (uint8_t *) malloc(sizeof(uint8_t) * frame.data_size);

  // Parse body
  for (i = 0; i < frame.data_size; i++) {
    frame.data[i] = data[4 + i]; // +4 because header is 4 bytes
    expected_checksum += data[4 + i];
  }
  frame.data_checksum = data[4 + i];

  // Check the body checksum
  expected_checksum = expected_checksum % 256;
  if (frame.data_checksum != expected_checksum) {
    // LOG_ERROR("Failed body checksum!");
    free(frame.data);
    return -1;
  }

  return 0;
}

int sbgc_frame_parse(sbgc_frame_t &frame, uint8_t *data) {
  int retval;

  // Header
  retval = sbgc_frame_parse_header(frame, data);
  if (retval == -1) {
    // LOG_ERROR("Failed to parse header!");
    return -1;
  }

  // Body
  retval = sbgc_frame_parse_body(frame, data);
  if (retval == -1) {
    // LOG_ERROR("Failed to parse body!");
    return -1;
  }

  frame.ok = true;
  return 0;
}

void sbgc_realtime_data_print(const sbgc_realtime_data_t &data) {
  // ACCELEROMOETER AND GYROSCOPE
  printf("accelerometer: %.2f\t%.2f\t%.2f\n",
         data.accel(0),
         data.accel(1),
         data.accel(2));
  printf("gyroscope: %.2f\t%.2f\t%.2f\n",
         data.gyro(0),
         data.gyro(1),
         data.gyro(2));
  printf("\n");

  // ANGLES
  printf("camera_angles: %.2f\t%.2f\t%.2f\n",
         data.camera_angles(0),
         data.camera_angles(1),
         data.camera_angles(2));
  printf("frame_angles: %.2f\t%.2f\t%.2f\n",
         data.frame_angles(0),
         data.frame_angles(1),
         data.frame_angles(2));
  printf("rc_angles: %.2f\t%.2f\t%.2f\n",
         data.rc_angles(0),
         data.rc_angles(1),
         data.rc_angles(2));

  // MISC
  printf("cycle_time: %d\n", data.cycle_time);
  printf("i2c_error_count: %d\n", data.i2c_error_count);
  printf("system_error: %d\n", data.system_error);
  printf("battery_level: %d\n\n", data.battery_level);
}

int sbgc_connect(sbgc_t &sbgc) {
  // Open serial port
  sbgc.serial = open(sbgc.port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (sbgc.serial < 0) {
    return -1;
  }

  // Configure serial commnication
  // set_interface_attribs(sbgc.serial, B115200, 0);
  // set_blocking(sbgc.serial, 1);
  LOG_INFO("Connected to SBGC!");

  sbgc.connected = true;
  return 0;
}

int sbgc_disconnect(sbgc_t &sbgc) {
  if (close(sbgc.serial) != 0) {
    LOG_ERROR("Failed to disconnect from SBGC!");
    return -1;

  } else {
    LOG_INFO("Disconnect from SBGC!");
    sbgc.connected = false;
    sbgc.serial = -1;
    return 0;
  }
}

int sbgc_send(const sbgc_t &sbgc, const sbgc_frame_t &cmd) {
  // Check connection
  if (sbgc.connected == false) {
    LOG_ERROR("Not connected to SBGC!");
    return -1;
  }

  // Check command data size
  int data_size_limit;
  data_size_limit = SBGC_CMD_MAX_BYTES - SBGC_CMD_PAYLOAD_BYTES;
  if (cmd.data_size >= data_size_limit) {
    return -1;
  }

  // Header
  ssize_t retval = 0;
  const uint8_t start = 0x3E; // ">" character
  retval += write(sbgc.serial, &start, 1);
  retval += write(sbgc.serial, &cmd.cmd_id, 1);
  retval += write(sbgc.serial, &cmd.data_size, 1);

  // Body
  retval += write(sbgc.serial, &cmd.header_checksum, 1);
  retval += write(sbgc.serial, cmd.data, cmd.data_size);
  retval += write(sbgc.serial, &cmd.data_checksum, 1);

  // Flush
  tcflush(sbgc.serial, TCIOFLUSH); // very critical
  usleep(10 * 1000);
  if (retval != (5 + cmd.data_size)) {
    LOG_ERROR("Opps! frame wasn't sent completely!");
  }

  return 0;
}

int sbgc_read(const sbgc_t &sbgc,
              const uint8_t read_length,
              sbgc_frame_t &frame) {
  // Check connection
  if (sbgc.connected == false) {
    LOG_ERROR("Not connected to SBGC!");
    return -1;
  }

  // Send query
  uint8_t buffer[150];
  int16_t nb_bytes = read(sbgc.serial, buffer, read_length);
  if (nb_bytes <= 0 || nb_bytes != read_length) {
    // LOG_ERROR("Failed to read SBGC frame!");
    return -1;
  }

  // Parse sbgc frame
  int retval = sbgc_frame_parse(frame, buffer);
  if (retval == -1) {
    // LOG_ERROR("Failed to parse SBGC frame!");
    return -1;
  }

  return 0;
}

int sbgc_on(const sbgc_t &sbgc) {
  sbgc_frame_t cmd;
  sbgc_frame_build(cmd, CMD_MOTORS_ON);
  return sbgc_send(sbgc, cmd);
}

int sbgc_off(const sbgc_t &sbgc) {
  // Pre-check
  if (sbgc.connected == false) {
    LOG_ERROR("Not connected to SBGC!");
    return -1;
  }

  // Turn off motor control
  uint8_t data[13];
  data[0] = MODE_NO_CONTROL;
  for (int i = 1; i < 13; i++) {
    data[i] = 0;
  }

  // Send frame
  sbgc_frame_t cmd;
  sbgc_frame_build(cmd, CMD_CONTROL, data, 13);
  int retval = sbgc_send(sbgc, cmd);
  if (retval != 0) {
    LOG_ERROR("Failed to turn motor control off!");
  }

  // Turn off motors
  sbgc_frame_build(cmd, CMD_MOTORS_OFF);
  retval = sbgc_send(sbgc, cmd);
  if (retval != 0) {
    LOG_ERROR("Failed to turn motor control off!");
  }

  return 0;
}

int sbgc_reset(sbgc_t &sbgc) {
  // Pre-check
  if (sbgc.connected == false) {
    LOG_ERROR("Not connected to SBGC!");
    return -1;
  }

  if (sbgc_off(sbgc) || sbgc_on(sbgc)) {
    LOG_ERROR("Failed to reset SBGC!");
    return -1;
  }

  return 0;
}

int sbgc_get_board_info(sbgc_t &sbgc) {
  // Pre-check
  if (sbgc.connected == false) {
    LOG_ERROR("Not connected to SBGC!");
    return -1;
  }

  // Request board info
  sbgc_frame_t frame;
  sbgc_frame_build(frame, CMD_BOARD_INFO);
  int retval = sbgc_send(sbgc, frame);
  if (retval == -1) {
    LOG_ERROR("Failed to request SBGC board info!");
    return -1;
  }

  // Obtain board info
  retval = sbgc_read(sbgc, CMD_BOARD_INFO_FRAME_SIZE, frame);
  if (retval == -1) {
    LOG_ERROR("Failed to parse SBGC frame for board info!");
    return -1;
  }

  // Set object board info
  sbgc.board_version = frame.data[0];
  sbgc.firmware_version = (frame.data[2] << 8) | (frame.data[1] & 0xff);
  sbgc.debug_mode = frame.data[3];
  sbgc.board_features = (frame.data[5] << 8) | (frame.data[4] & 0xff);
  sbgc.connection_flags = frame.data[6];

  // Clean up
  free(frame.data);

  return 0;
}

int sbgc_get_realtime_data(sbgc_t &sbgc) {
  // Pre-check
  if (sbgc.connected == false) {
    LOG_ERROR("Not connected to SBGC!");
    return -1;
  }

  // Request real time data
  sbgc_frame_t frame;
  sbgc_frame_build(frame, CMD_REALTIME_DATA_3);
  if (sbgc_send(sbgc, frame) == -1) {
    LOG_ERROR("Failed to request SBGC realtime data!");
    return -1;
  }

  // Obtain real time data
  if (sbgc_read(sbgc, 68, frame) == -1) {
    LOG_ERROR("Failed to parse SBGC frame for realtime data!");
    return -1;
  }

  // Parse real time data
  // Accelerometer and gyroscope
  sbgc.data.accel(0) = S16BIT(frame.data, 1, 0);
  sbgc.data.gyro(0) = S16BIT(frame.data, 3, 2);
  sbgc.data.accel(1) = S16BIT(frame.data, 5, 4);
  sbgc.data.gyro(1) = S16BIT(frame.data, 7, 6);
  sbgc.data.accel(2) = S16BIT(frame.data, 9, 8);
  sbgc.data.gyro(2) = S16BIT(frame.data, 11, 10);

  sbgc.data.accel(0) = (ACC_UNIT) *sbgc.data.accel(0);
  sbgc.data.accel(1) = (ACC_UNIT) *sbgc.data.accel(1);
  sbgc.data.accel(2) = (ACC_UNIT) *sbgc.data.accel(2);

  sbgc.data.gyro(0) = (GYRO_UNIT) *sbgc.data.gyro(0);
  sbgc.data.gyro(1) = (GYRO_UNIT) *sbgc.data.gyro(1);
  sbgc.data.gyro(2) = (GYRO_UNIT) *sbgc.data.gyro(2);

  // Angles
  sbgc.data.camera_angles(0) = S16BIT(frame.data, 33, 32);
  sbgc.data.camera_angles(1) = S16BIT(frame.data, 35, 34);
  sbgc.data.camera_angles(2) = S16BIT(frame.data, 37, 36);

  sbgc.data.frame_angles(0) = S16BIT(frame.data, 39, 38);
  sbgc.data.frame_angles(1) = S16BIT(frame.data, 41, 40);
  sbgc.data.frame_angles(2) = S16BIT(frame.data, 43, 42);

  sbgc.data.rc_angles(0) = S16BIT(frame.data, 45, 44);
  sbgc.data.rc_angles(1) = S16BIT(frame.data, 47, 45);
  sbgc.data.rc_angles(2) = S16BIT(frame.data, 49, 46);

  sbgc.data.camera_angles(0) = (DEG_PER_BIT) *sbgc.data.camera_angles(0);
  sbgc.data.camera_angles(1) = (DEG_PER_BIT) *sbgc.data.camera_angles(1);
  sbgc.data.camera_angles(2) = (DEG_PER_BIT) *sbgc.data.camera_angles(2);

  sbgc.data.frame_angles(0) = (DEG_PER_BIT) *sbgc.data.frame_angles(0);
  sbgc.data.frame_angles(1) = (DEG_PER_BIT) *sbgc.data.frame_angles(1);
  sbgc.data.frame_angles(2) = (DEG_PER_BIT) *sbgc.data.frame_angles(2);

  sbgc.data.rc_angles(0) = (DEG_PER_BIT) *sbgc.data.rc_angles(0);
  sbgc.data.rc_angles(1) = (DEG_PER_BIT) *sbgc.data.rc_angles(1);
  sbgc.data.rc_angles(2) = (DEG_PER_BIT) *sbgc.data.rc_angles(2);

  // misc
  sbgc.data.cycle_time = U16BIT(frame.data, 51, 50);
  sbgc.data.i2c_error_count = U16BIT(frame.data, 53, 52);
  sbgc.data.system_error = U16BIT(frame.data, 15, 14);
  sbgc.data.battery_level = U16BIT(frame.data, 56, 55);

  // sbgc_realtime_data_print(sbgc.data);

  return 0;
}

int sbgc_get_realtime_data4(sbgc_t &sbgc) {
  // Pre-check
  if (sbgc.connected == false) {
    LOG_ERROR("Not connected to SBGC!");
    return -1;
  }

  // Request real time data
  sbgc_frame_t frame;
  sbgc_frame_build(frame, CMD_REALTIME_DATA_4);
  if (sbgc_send(sbgc, frame) == -1) {
    // LOG_ERROR("Failed to request SBGC realtime data!");
    return -1;
  }

  // Obtain real time data
  if (sbgc_read(sbgc, 129, frame) == -1) {
    // LOG_ERROR("Failed to parse SBGC frame for realtime data!" <<
    // std::endl;
    return -1;
  }

  // Parse real time data
  // Accelerometer and gyroscope
  sbgc.data.accel(0) = S16BIT(frame.data, 1, 0);
  sbgc.data.gyro(0) = S16BIT(frame.data, 3, 2);
  sbgc.data.accel(1) = S16BIT(frame.data, 5, 4);
  sbgc.data.gyro(1) = S16BIT(frame.data, 7, 6);
  sbgc.data.accel(2) = S16BIT(frame.data, 9, 8);
  sbgc.data.gyro(2) = S16BIT(frame.data, 11, 10);

  sbgc.data.accel(0) = (ACC_UNIT) *sbgc.data.accel(0);
  sbgc.data.accel(1) = (ACC_UNIT) *sbgc.data.accel(1);
  sbgc.data.accel(2) = (ACC_UNIT) *sbgc.data.accel(2);

  sbgc.data.gyro(0) = (GYRO_UNIT) *sbgc.data.gyro(0);
  sbgc.data.gyro(1) = (GYRO_UNIT) *sbgc.data.gyro(1);
  sbgc.data.gyro(2) = (GYRO_UNIT) *sbgc.data.gyro(2);

  // Angles
  sbgc.data.camera_angles(0) = S16BIT(frame.data, 33, 32);
  sbgc.data.camera_angles(1) = S16BIT(frame.data, 35, 34);
  sbgc.data.camera_angles(2) = S16BIT(frame.data, 37, 36);

  sbgc.data.frame_angles(0) = S16BIT(frame.data, 39, 38);
  sbgc.data.frame_angles(1) = S16BIT(frame.data, 41, 40);
  sbgc.data.frame_angles(2) = S16BIT(frame.data, 43, 42);

  sbgc.data.rc_angles(0) = S16BIT(frame.data, 45, 44);
  sbgc.data.rc_angles(1) = S16BIT(frame.data, 47, 45);
  sbgc.data.rc_angles(2) = S16BIT(frame.data, 49, 46);

  sbgc.data.encoder_angles(0) = S16BIT(frame.data, 64, 63);
  sbgc.data.encoder_angles(1) = S16BIT(frame.data, 66, 65);
  sbgc.data.encoder_angles(2) = S16BIT(frame.data, 68, 67);

  sbgc.data.camera_angles(0) = (DEG_PER_BIT) *sbgc.data.camera_angles(0);
  sbgc.data.camera_angles(1) = (DEG_PER_BIT) *sbgc.data.camera_angles(1);
  sbgc.data.camera_angles(2) = (DEG_PER_BIT) *sbgc.data.camera_angles(2);

  sbgc.data.frame_angles(0) = (DEG_PER_BIT) *sbgc.data.frame_angles(0);
  sbgc.data.frame_angles(1) = (DEG_PER_BIT) *sbgc.data.frame_angles(1);
  sbgc.data.frame_angles(2) = (DEG_PER_BIT) *sbgc.data.frame_angles(2);

  sbgc.data.rc_angles(0) = (DEG_PER_BIT) *sbgc.data.rc_angles(0);
  sbgc.data.rc_angles(1) = (DEG_PER_BIT) *sbgc.data.rc_angles(1);
  sbgc.data.rc_angles(2) = (DEG_PER_BIT) *sbgc.data.rc_angles(2);

  sbgc.data.encoder_angles(0) = (DEG_PER_BIT) *sbgc.data.encoder_angles(0);
  sbgc.data.encoder_angles(1) = (DEG_PER_BIT) *sbgc.data.encoder_angles(1);
  sbgc.data.encoder_angles(2) = (DEG_PER_BIT) *sbgc.data.encoder_angles(2);

  // Misc
  sbgc.data.cycle_time = U16BIT(frame.data, 51, 50);
  sbgc.data.i2c_error_count = U16BIT(frame.data, 53, 52);
  sbgc.data.system_error = U16BIT(frame.data, 15, 14);
  sbgc.data.battery_level = U16BIT(frame.data, 56, 55);

  // sbgc_realtime_data_print(sbgc.data);

  return 0;
}

int sbgc_get_angles_ext(sbgc_t &sbgc) {
  int retval;

  // Request real time data
  sbgc_frame_t frame;
  sbgc_frame_build(frame, CMD_GET_ANGLES_EXT);
  if (sbgc_send(sbgc, frame) == -1) {
    LOG_ERROR("Failed to request SBGC realtime data!");
    return -1;
  }

  // Obtain real time data
  if (sbgc_read(sbgc, 54, frame) == -1) {
    LOG_ERROR("Failed to parse SBGC frame for realtime data!");
    return -1;
  }

  // Roll
  sbgc.data.camera_angles(0) = S16BIT(frame.data, 1, 0);
  sbgc.data.rc_angles(0) = S16BIT(frame.data, 3, 2);
  sbgc.data.encoder_angles(0) = S16BIT(frame.data, 8, 4);

  // Pitch
  sbgc.data.camera_angles(1) = S16BIT(frame.data, 19, 18);
  sbgc.data.rc_angles(1) = S16BIT(frame.data, 21, 20);
  sbgc.data.encoder_angles(1) = S16BIT(frame.data, 26, 22);

  sbgc.data.camera_angles(2) = S16BIT(frame.data, 37, 36);
  sbgc.data.rc_angles(2) = S16BIT(frame.data, 39, 38);
  sbgc.data.encoder_angles(2) = S16BIT(frame.data, 44, 40);

  sbgc.data.camera_angles(0) = (DEG_PER_BIT) *sbgc.data.camera_angles(0);
  sbgc.data.camera_angles(1) = (DEG_PER_BIT) *sbgc.data.camera_angles(1);
  sbgc.data.camera_angles(2) = (DEG_PER_BIT) *sbgc.data.camera_angles(2);

  sbgc.data.rc_angles(0) = (DEG_PER_BIT) *sbgc.data.rc_angles(0);
  sbgc.data.rc_angles(1) = (DEG_PER_BIT) *sbgc.data.rc_angles(1);
  sbgc.data.rc_angles(2) = (DEG_PER_BIT) *sbgc.data.rc_angles(2);

  sbgc.data.encoder_angles(0) = (DEG_PER_BIT) *sbgc.data.encoder_angles(0);
  sbgc.data.encoder_angles(1) = (DEG_PER_BIT) *sbgc.data.encoder_angles(1);
  sbgc.data.encoder_angles(2) = (DEG_PER_BIT) *sbgc.data.encoder_angles(2);

  // sbgc_realtime_data_print(sbgc.data);

  return 0;
}

int sbgc_set_angle(const sbgc_t &sbgc,
                   const double roll,
                   const double pitch,
                   const double yaw) {
  // adjust roll, pitch and yaw
  int16_t roll_adjusted = (int16_t)(roll / DEG_PER_BIT);
  int16_t pitch_adjusted = (int16_t)(pitch / DEG_PER_BIT);
  int16_t yaw_adjusted = (int16_t)(yaw / DEG_PER_BIT);

  // control mode
  uint8_t data[13];
  data[0] = MODE_ANGLE;

  // speed roll
  data[1] = 0;
  data[2] = 0;

  // angle roll
  data[3] = ((roll_adjusted >> 0) & 0xff);
  data[4] = ((roll_adjusted >> 8) & 0xff);

  // speed pitch
  data[5] = 0;
  data[6] = 0;

  // angle pitch
  data[7] = ((pitch_adjusted >> 0) & 0xff);
  data[8] = ((pitch_adjusted >> 8) & 0xff);

  // speed yaw
  data[9] = 0;
  data[10] = 0;

  // angle yaw
  data[11] = ((yaw_adjusted >> 0) & 0xff);
  data[12] = ((yaw_adjusted >> 8) & 0xff);

  // build frame and send
  sbgc_frame_t frame;
  sbgc_frame_build(frame, CMD_CONTROL, data, 13);
  sbgc_send(sbgc, frame);

  return 0;
}

int sbgc_set_speed_angle(const sbgc_t &sbgc,
                         const double roll,
                         const double pitch,
                         const double yaw,
                         const double roll_speed,
                         const double pitch_speed,
                         const double yaw_speed) {
  // Adjust roll, pitch and yaw
  int16_t roll_adjusted = (int16_t)(roll / DEG_PER_BIT);
  int16_t pitch_adjusted = (int16_t)(pitch / DEG_PER_BIT);
  int16_t yaw_adjusted = (int16_t)(yaw / DEG_PER_BIT);
  int16_t roll_speed_adjusted = (int16_t)(roll_speed / DEG_SEC_PER_BIT);
  int16_t pitch_speed_adjusted = (int16_t)(pitch_speed / DEG_SEC_PER_BIT);
  int16_t yaw_speed_adjusted = (int16_t)(yaw_speed / DEG_SEC_PER_BIT);

  // Control mode
  uint8_t data[13];
  data[0] = MODE_SPEED_ANGLE;

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
  sbgc_frame_build(frame, CMD_CONTROL, data, 13);
  sbgc_send(sbgc, frame);

  return 0;
}

} //  namespace prototype
