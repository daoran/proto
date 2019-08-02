#include "proto/driver/ublox.hpp"

namespace proto {

/*****************************************************************************
 * UBX Message
 ****************************************************************************/

ubx_msg_t::ubx_msg_t() {}

ubx_msg_t::ubx_msg_t(const uint8_t *data) { ubx_msg_parse(*this, data); }

ubx_msg_t::ubx_msg_t(const uint8_t msg_class_,
                     const uint8_t msg_id_,
                     const uint16_t length_,
                     const uint8_t *payload_) {
  // Header
  msg_class = msg_class_;
  msg_id = msg_id_;
  payload_length = length_;

  // Payload
  for (size_t i = 0; i < length_; i++) {
    payload[i] = payload_[i];
  }

  // Checksum
  ubx_msg_checksum(msg_class, msg_id, length_, payload_, ck_a, ck_b);
  ok = true;
}

void ubx_msg_parse(ubx_msg_t &msg, const uint8_t *data) {
  // Check SYNC_1 and SYNC_2
  if (data[0] != 0xB5 || data[1] != 0x62) {
    msg.ok = false;
    return;
  }

  // Message class and id
  msg.msg_class = data[2];
  msg.msg_id = data[3];

  // Payload
  msg.payload_length = (data[5] << 8) | (data[4]);
  for (uint16_t i = 0; i < msg.payload_length; i++) {
    msg.payload[i] = data[6 + i];
  }

  // Checksum
  msg.ck_a = data[msg.payload_length + 6];
  msg.ck_b = data[msg.payload_length + 6 + 1];
  msg.ok = ubx_msg_is_valid(msg);
}

void ubx_msg_serialize(const ubx_msg_t &msg,
                       uint8_t *frame,
                       size_t &frame_size) {
  // Form packet frame (header[6] + payload length + checksum[2])
  frame_size = 6 + msg.payload_length + 2;

  // -- Form header
  frame[0] = 0xB5;                             // Sync Char 1
  frame[1] = 0x62;                             // Sync Char 2
  frame[2] = msg.msg_class;                    // Message class
  frame[3] = msg.msg_id;                       // Message id
  frame[4] = msg.payload_length & 0xFF;        // Length
  frame[5] = (msg.payload_length >> 8) & 0xFF; // Length

  // -- Form payload
  for (size_t i = 0; i < msg.payload_length; i++) {
    frame[6 + i] = msg.payload[i];
  }

  // -- Form checksum
  frame[frame_size - 2] = msg.ck_a;
  frame[frame_size - 1] = msg.ck_b;
}

void ubx_msg_checksum(const uint8_t msg_class,
                      const uint8_t msg_id,
                      const uint16_t payload_length,
                      const uint8_t *payload,
                      uint8_t &ck_a,
                      uint8_t &ck_b) {
  ck_a = 0;
  ck_b = 0;

  ck_a = ck_a + msg_class;
  ck_b = ck_b + ck_a;

  ck_a = ck_a + msg_id;
  ck_b = ck_b + ck_a;

  ck_a = ck_a + payload_length & 0x00FF;
  ck_b = ck_b + ck_a;

  ck_a = ck_a + ((payload_length & 0xFF00) >> 8);
  ck_b = ck_b + ck_a;

  for (uint16_t i = 0; i < payload_length; i++) {
    ck_a = ck_a + payload[i];
    ck_b = ck_b + ck_a;
  }
}

void ubx_msg_checksum(const ubx_msg_t &msg, uint8_t &ck_a, uint8_t &ck_b) {
  ubx_msg_checksum(msg.msg_class,
                   msg.msg_id,
                   msg.payload_length,
                   msg.payload,
                   ck_a,
                   ck_b);
}

bool ubx_msg_is_valid(const ubx_msg_t &msg) {
  uint8_t expected_ck_a = 0;
  uint8_t expected_ck_b = 0;
  ubx_msg_checksum(msg, expected_ck_a, expected_ck_b);
  if (expected_ck_a == msg.ck_a && expected_ck_b == msg.ck_b) {
    return true;
  }

  return false;
}

void ubx_msg_print(const ubx_msg_t &msg) {
  printf("msg_class: 0x%02x\n", msg.msg_class);
  printf("msg_id: 0x%02x\n", msg.msg_id);
  printf("payload_length: 0x%02x\n", msg.payload_length);
  for (size_t i = 0; i < msg.payload_length; i++) {
    printf("payload[%zu]: 0x%02x\n", i, msg.payload[i]);
  }
  printf("ck_a: 0x%02x\n", msg.ck_a);
  printf("ck_b: 0x%02x\n", msg.ck_b);
}

/*****************************************************************************
 * UBX Stream Parser
 ****************************************************************************/

ubx_parser_t::ubx_parser_t() {}

void ubx_parser_reset(ubx_parser_t &parser) {
  parser.state = SYNC_1;
  for (size_t i = 0; i < 1024; i++) {
    parser.buf_data[i] = 0;
  }
  parser.buf_pos = 0;
}

int ubx_parser_update(ubx_parser_t &parser, uint8_t data) {
  // Add byte to buffer
  parser.buf_data[parser.buf_pos++] = data;

  // Parse byte
  switch (parser.state) {
  case SYNC_1:
    if (data == 0xB5) {
      parser.state = SYNC_2;
    } else {
      ubx_parser_reset(parser);
    }
    break;
  case SYNC_2:
    if (data == 0x62) {
      parser.state = MSG_CLASS;
    } else {
      ubx_parser_reset(parser);
    }
    break;
  case MSG_CLASS: parser.state = MSG_ID; break;
  case MSG_ID: parser.state = PAYLOAD_LENGTH_LOW; break;
  case PAYLOAD_LENGTH_LOW: parser.state = PAYLOAD_LENGTH_HI; break;
  case PAYLOAD_LENGTH_HI: parser.state = PAYLOAD_DATA; break;
  case PAYLOAD_DATA: {
    uint8_t length_low = parser.buf_data[4];
    uint8_t length_hi = parser.buf_data[5];
    uint16_t payload_length = (length_hi << 8) | (length_low);
    if (parser.buf_pos == 6 + payload_length) {
      parser.state = CK_A;
    }
    if (parser.buf_pos >= 1022) {
      ubx_parser_reset(parser);
      return -2;
    }
    break;
  }
  case CK_A: parser.state = CK_B; break;
  case CK_B:
    parser.msg = ubx_msg_t{parser.buf_data};
    ubx_parser_reset(parser);
    return 1;
  default: FATAL("Invalid Parser State!"); break;
  }

  return 0;
}

/*****************************************************************************
 * RTCM3 Stream Parser
 ****************************************************************************/

rtcm3_parser_t::rtcm3_parser_t() {}

void rtcm3_parser_reset(rtcm3_parser_t &parser) {
  for (size_t i = 0; i < 9046; i++) {
    parser.buf_data[i] = 0;
  }
  parser.buf_pos = 0;
  parser.msg_len = 0;
}

int rtcm3_parser_update(rtcm3_parser_t &parser, uint8_t data) {
  // Add byte to buffer
  parser.buf_data[parser.buf_pos] = data;

  // Parse message
  if (parser.buf_data[0] != 0xD3) {
    rtcm3_parser_reset(parser);
  } else if (parser.buf_pos == 1) {
    // Get the last two bits of this byte. Bits 8 and 9 of 10-bit length
    parser.msg_len = (data & 0x03) << 8;
  } else if (parser.buf_pos == 2) {
    parser.msg_len |= data; // Bits 0-7 of packet length
    parser.msg_len += 6;
    // There are 6 additional bytes of what we presume is
    // header, msgType, CRC, and stuff
  } else if (parser.buf_pos == 3) {
    parser.msg_type = data << 4; // Message Type, most significant 4 bits
  } else if (parser.buf_pos == 4) {
    parser.msg_type |= (data >> 4); // Message Type, bits 0-7
  }
  parser.buf_pos++;

  // Check if end of message
  if (parser.buf_pos == parser.msg_len) {
    return 1;
  }

  return 0;
}

/*****************************************************************************
 * Ublox
 ****************************************************************************/

ublox_t::ublox_t(const std::string &port, const int speed) {
  uart = uart_t{port, speed};
}

ublox_t::ublox_t(const uart_t &uart_) : uart{uart_} { ok = true; }

ublox_t::~ublox_t() {
  if (uart.connection != -1 && uart_disconnect(uart)) {
    LOG_ERROR("Failed to disconnect from serial");
  } else {
    ok = false;
  }
}

int ublox_connect(ublox_t &ublox) {
  if (uart_connect(ublox.uart) != 0) {
    return -1;
  }

  ublox.ok = true;
  return 0;
}

int ubx_write(const ublox_t &ublox,
              uint8_t msg_class,
              uint8_t msg_id,
              uint16_t length,
              uint8_t *payload) {
  // Build and serialize UBX message
  const ubx_msg_t msg{msg_class, msg_id, length, payload};
  uint8_t frame[1024] = {0};
  size_t frame_size = 0;
  ubx_msg_serialize(msg, frame, frame_size);

  // Transmit msg
  if (uart_write(ublox.uart, frame, frame_size) != 0) {
    LOG_ERROR("Failed to send data to UART!");
    return -1;
  }

  return 0;
}

int ubx_poll(const ublox_t &ublox,
             const uint8_t msg_class,
             const uint8_t msg_id,
             uint16_t &payload_length,
             uint8_t *payload,
             const bool expect_ack,
             const int retry) {
  int attempts = 0;
  ubx_parser_t parser;

request:
  // Request
  attempts++;
  if (attempts > retry) {
    payload_length = 0;
    return -1;
  }
  ubx_write(ublox, msg_class, msg_id, payload_length, payload);

  // Arbitrary counter for response timeout
  int counter = 0;
response:
  // Response
  while (counter < 1024) {
    uint8_t data = 0;
    if (uart_read(ublox.uart, &data, 1) == 0) {
      if (ubx_parser_update(parser, data) == 1) {
        break;
      }
    }

    counter++;
  }

  // Check parsed message
  if (parser.msg.ok == false) {
    LOG_WARN("Checksum failed, retrying ...");
    goto request;
  }

  // Try sending the request again?
  if (counter == 1024) {
    goto request;
  }

  // Check message
  bool msg_is_ack = (parser.msg.msg_class == UBX_ACK);
  bool match_class = (parser.msg.msg_class == msg_class);
  bool match_id = (parser.msg.msg_id == msg_id);
  if (!msg_is_ack && match_class && match_id) {
    // Copy payload length and data
    for (uint16_t i = 0; i < parser.msg.payload_length; i++) {
      payload[i] = parser.msg.payload[i];
    }
    payload_length = parser.msg.payload_length;

    // Get another message (hopefully an ACK)
    if (expect_ack) {
      counter = 0;
      goto response;
    } else {
      return 0;
    }
  }
  if (not msg_is_ack and not match_class and not match_id) {
    // Get another message
    goto response;

  } else if (expect_ack && msg_is_ack) {
    // Check the ACK message
    bool match_class = (msg_class == parser.msg.payload[0]);
    bool match_id = (msg_id == parser.msg.payload[1]);
    bool is_ack = (parser.msg.msg_id == UBX_ACK_ACK);
    if (match_class && match_id && is_ack) {
      return 0;
    } else {
      return -2;
    }
  }

  return 0;
}

int ubx_read_ack(const ublox_t &ublox,
                 const uint8_t msg_class,
                 const uint8_t msg_id) {
  ubx_parser_t parser;

  // Get Ack
  int counter = 0; // Arbitrary counter for timeout
  while (counter != 1024) {
    uint8_t data = 0;
    if (uart_read(ublox.uart, &data, 1) != 0) {
      continue;
    }

    if (ubx_parser_update(parser, data) == 1) {
      const bool is_ack_msg = (parser.msg.msg_class == UBX_ACK);
      const uint8_t ack_msg_class = parser.msg.payload[0];
      const uint8_t ack_msg_id = parser.msg.payload[1];
      const bool ack_msg_class_match = (ack_msg_class == msg_class);
      const bool ack_msg_id_match = (ack_msg_id == msg_id);

      if (is_ack_msg && ack_msg_class_match && ack_msg_id_match) {
        break;
      }
    }

    counter++;
  }

  // Try again?
  if (counter == 1024) {
    return 1;
  }

  return (parser.msg.msg_id == UBX_ACK_ACK) ? 0 : -1;
}

int ubx_val_get(const ublox_t &ublox,
                const uint8_t layer,
                const uint32_t key,
                uint32_t &val) {
  // Build message
  uint16_t payload_length = 4 + 4;
  uint8_t payload[1024] = {0};
  payload[0] = 0; // Version
  payload[1] = layer;
  payload[4 + 0] = key >> 0;
  payload[4 + 1] = key >> 8;
  payload[4 + 2] = key >> 16;
  payload[4 + 3] = key >> 24;

  // Poll
  int retval =
      ubx_poll(ublox, UBX_CFG, UBX_CFG_VALGET, payload_length, payload, true);
  if (retval != 0) {
    return -1;
  }

  val = u32bit(payload, 8);
  return 0;
}

int ubx_val_set(const ublox_t &ublox,
                const uint8_t layer,
                const uint32_t key,
                const uint32_t val,
                const uint8_t val_size) {
  const uint32_t bit_masks[4] = {0x000000FF,
                                 0x0000FF00,
                                 0x00FF0000,
                                 0xFF000000};

  // Build message
  uint16_t payload_length = 4 + 4 + val_size;
  uint8_t payload[1024] = {0};
  payload[0] = 0; // Version
  payload[1] = layer;
  payload[2] = 0;

  payload[4 + 0] = (key & bit_masks[0]);
  payload[4 + 1] = (key & bit_masks[1]) >> 8;
  payload[4 + 2] = (key & bit_masks[2]) >> 16;
  payload[4 + 3] = (key & bit_masks[3]) >> 24;

  for (uint8_t i = 0; i < val_size; i++) {
    payload[4 + 4 + i] = ((val & bit_masks[i]) >> (8 * i));
  }

  // Set value
  int attempts = 0;
retry:
  attempts++;
  if (attempts >= 5) {
    LOG_ERROR("Failed to set configuration!");
    return -1;
  }

  ubx_write(ublox, UBX_CFG, UBX_CFG_VALSET, payload_length, payload);
  switch (ubx_read_ack(ublox, UBX_CFG, UBX_CFG_VALSET)) {
  case 0: return 0;
  case 1: goto retry;
  case -1:
  default: LOG_ERROR("Failed to set configuration!"); return -1;
  }
}

void ublox_version(const ublox_t &ublox) {
  uint16_t payload_length = 0;
  uint8_t payload[1024] = {0};
  if (ubx_poll(ublox, UBX_MON, 0x04, payload_length, payload) == 0) {
    printf("SW VERSION: %s\n", payload);
    printf("HW VERSION: %s\n", payload + 30);
    printf("%s\n", payload + 40);
  } else {
    LOG_ERROR("Failed to obtain UBlox version!");
  }
}

int ublox_parse_ubx(ublox_t &ublox, uint8_t data) {
  if (ubx_parser_update(ublox.ubx_parser, data) == 1) {
    const uint8_t msg_class = ublox.ubx_parser.msg.msg_class;
    const uint8_t msg_id = ublox.ubx_parser.msg.msg_id;
    DEBUG("[UBX]\tmsg_class: %d\tmsg_id: %d", msg_class, msg_id);

    // UBX-NAV
    if (msg_class == UBX_NAV) {
      switch (msg_id) {
      case UBX_NAV_DOP:
        if (ublox.nav_dop_cb) {
          ublox.nav_dop_cb(ublox);
        }
        break;
      case UBX_NAV_EOE:
        if (ublox.nav_eoe_cb) {
          ublox.nav_eoe_cb(ublox);
        }
        break;
      case UBX_NAV_HPPOSLLH:
        if (ublox.nav_hpposllh_cb) {
          ublox.nav_hpposllh_cb(ublox);
        }
        break;
      case UBX_NAV_PVT:
        if (ublox.nav_pvt_cb) {
          ublox.nav_pvt_cb(ublox);
        }
        break;
      case UBX_NAV_SVIN:
        if (ublox.nav_svin_cb) {
          ublox.nav_svin_cb(ublox);
        }
        break;
      case UBX_NAV_STATUS:
        if (ublox.nav_status_cb) {
          ublox.nav_status_cb(ublox);
        }
        break;
      case UBX_NAV_VELNED:
        if (ublox.nav_velned_cb) {
          ublox.nav_velned_cb(ublox);
        }
        break;
      default:
        LOG_WARN("Unsupported [msg cls: %d - msg id: %d]", msg_class, msg_id);
        break;
      }
    }

    // UBX-MON
    if (msg_class == UBX_MON) {
      switch (msg_id) {
      case UBX_MON_RF:
        if (ublox.mon_rf_cb) {
          ublox.mon_rf_cb(ublox);
        }
        break;
      default:
        LOG_WARN("Unsupported [msg cls: %d - msg id: %d]", msg_class, msg_id);
        break;
      }
    }

    // UBX-RXM
    if (msg_class == UBX_RXM) {
      switch (msg_id) {
      case UBX_RXM_RTCM:
        if (ublox.rxm_rtcm_cb) {
          ublox.rxm_rtcm_cb(ublox);
        }
        break;
      default:
        LOG_WARN("Unsupported [msg cls: %d - msg id: %d]", msg_class, msg_id);
        break;
      }
    }

    // Reset msg type
    ublox.msg_type = "";

    return 1;
  }

  return 0;
}

void ublox_broadcast_rtcm3(ublox_t &ublox) {
  const uint8_t *msg_data = ublox.rtcm3_parser.buf_data;
  const size_t msg_len = ublox.rtcm3_parser.msg_len;
  const int msg_flags = MSG_DONTWAIT | MSG_NOSIGNAL;

  auto &conns = ublox.conns;
  for (const auto conn : conns) {
    if (send(conn, msg_data, msg_len, msg_flags) == -1) {
      conns.erase(std::remove(conns.begin(), conns.end(), conn), conns.end());
      LOG_ERROR("Rover diconnected!");
    }
  }
}

int ublox_parse_rtcm3(ublox_t &ublox, uint8_t data) {
  if (rtcm3_parser_update(ublox.rtcm3_parser, data)) {
    // Broadcast RTCM3 corrections
    ublox_broadcast_rtcm3(ublox);

    // Debug
    DEBUG("[RTCM3]\tmsg type: %zu\tmsg length: %zu",
          ublox.rtcm3_parser.msg_type,
          ublox.rtcm3_parser.msg_len);

    // Reset parser and msg type
    rtcm3_parser_reset(ublox.rtcm3_parser);
    ublox.msg_type = "";

    return 1;
  }

  return 0;
}

/*****************************************************************************
 * Ublox Base Station
 ****************************************************************************/

int ublox_base_station_config(ublox_t &base) {
  const uint8_t layer = 1; // RAM
  // const uint8_t layer = 2; // BBR
  int retval = 0;
  retval += ubx_val_set(base, layer, CFG_RATE_MEAS, 1000, 2); // 1000ms = 1Hz
  retval += ubx_val_set(base, layer, CFG_USBOUTPROT_NMEA, 0, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_RTCM_3X_TYPE1005_USB, 1, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_RTCM_3X_TYPE1077_USB, 1, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_RTCM_3X_TYPE1087_USB, 1, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_RTCM_3X_TYPE1097_USB, 1, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_RTCM_3X_TYPE1127_USB, 1, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_RTCM_3X_TYPE1230_USB, 1, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_NAV_CLOCK_USB, 0, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_NAV_DOP_USB, 0, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_NAV_EOE_USB, 0, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_NAV_HPPOSEECF_USB, 0, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB, 0, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_NAV_STATUS_USB, 0, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_NAV_SVIN_USB, 0, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_NAV_PVT_USB, 1, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_NAV_VELNED_USB, 0, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_MON_RF_USB, 0, 1);
  retval += ubx_val_set(base, layer, CFG_MSGOUT_UBX_RXM_RTCM_USB, 0, 1);
  retval += ubx_val_set(base, layer, CFG_TMODE_MODE, 1, 1);
  retval += ubx_val_set(base, layer, CFG_TMODE_SVIN_MIN_DUR, 60, 4);
  retval += ubx_val_set(base, layer, CFG_TMODE_SVIN_ACC_LIMIT, 50000, 4);
  if (retval != 0) {
    LOG_ERROR("Failed to configure Ublox into BASE_STATION mode!");
    return -1;
  }

  return 0;
}

void ublox_base_station_loop(ublox_t &base) {
  while (true) {
    // Accept the data packet from client and verification
    struct sockaddr_in client;
    socklen_t len = sizeof(client);
    const int flags = SOCK_NONBLOCK;
    const int connfd =
        accept4(base.sockfd, (struct sockaddr *) &client, &len, flags);
    if (connfd >= 0) {
      std::string ip;
      int port = 0;
      ip_port_info(connfd, ip, port);
      LOG_INFO("Server connected with UBlox client [%s:%d]", ip.c_str(), port);
      base.conns.push_back(connfd);
    }

    // Read byte
    uint8_t data = 0;
    if (uart_read(base.uart, &data, 1) != 0) {
      continue;
    }

    // Determine message type
    if (base.msg_type == "") {
      if (data == 0xB5) {
        base.msg_type = "UBX";
      } else if (data == 0xD3) {
        base.msg_type = "RTCM3";
      }
    }

    // Parse message
    if (base.msg_type == "UBX") {
      ublox_parse_ubx(base, data);
    } else if (base.msg_type == "RTCM3") {
      ublox_parse_rtcm3(base, data);
    }
  }
}

int ublox_base_station_run(ublox_t &base, const int port) {
  // Configure base station
  if (ublox_base_station_config(base) != 0) {
    LOG_ERROR("Failed to configure Ublox into BASE_STATION mode!");
    return -1;
  }

  // Socket create and verification
  base.sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (base.sockfd == -1) {
    LOG_ERROR("Socket creation failed...");
    return -1;
  } else {
    DEBUG("Socket created!");
  }

  // Change server socket into non-blocking state
  fcntl(base.sockfd, F_SETFL, O_NONBLOCK);

  // Socket options
  const int level = SOL_SOCKET;
  const int val = 1;
  const socklen_t len = sizeof(int);
  if (setsockopt(base.sockfd, level, SO_REUSEADDR, &val, len) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEADDR) failed");
  }
  if (setsockopt(base.sockfd, SOL_SOCKET, SO_REUSEPORT, &val, len) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEPORT) failed");
  }

  // Assign IP, PORT
  struct sockaddr_in server;
  bzero(&server, sizeof(server));
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = htonl(INADDR_ANY);
  server.sin_port = htons(port);

  // Bind newly created socket to given IP
  int retval = bind(base.sockfd, (struct sockaddr *) &server, sizeof(server));
  if (retval != 0) {
    LOG_ERROR("Socket bind failed...");
    LOG_ERROR("%s", strerror(errno));
    return -1;
  } else {
    DEBUG("Socket binded!");
  }

  // Server is ready to listen
  if ((listen(base.sockfd, 20)) != 0) {
    LOG_ERROR("Listen failed...");
    return -1;
  } else {
    DEBUG("Server running");
  }

  // Obtain RTCM3 Messages from receiver and transmit them to rover
  ublox_base_station_loop(base);

  // Clean up
  close(base.sockfd);
  base.sockfd = -1;

  return 0;
}

/*****************************************************************************
 * Ublox Rover
 ****************************************************************************/

int ublox_rover_config(ublox_t &rover) {
  // Configure rover
  const uint8_t layer = 1; // RAM
  int retval = 0;
  // retval += ubx_val_set(rover, layer, CFG_RATE_MEAS, 1000, 2);  // 1000ms =
  // 1Hz
  retval += ubx_val_set(rover, layer, CFG_RATE_MEAS, 100, 2); // 100ms = 10Hz
  retval += ubx_val_set(rover, layer, CFG_USBOUTPROT_NMEA, 0, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_NAV_CLOCK_USB, 0, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_NAV_DOP_USB, 1, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_NAV_EOE_USB, 1, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_NAV_HPPOSEECF_USB, 0, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB, 1, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_NAV_STATUS_USB, 1, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_NAV_SVIN_USB, 0, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_NAV_PVT_USB, 1, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_NAV_VELNED_USB, 1, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_MON_RF_USB, 1, 1);
  retval += ubx_val_set(rover, layer, CFG_MSGOUT_UBX_RXM_RTCM_USB, 1, 1);
  retval += ubx_val_set(rover, layer, CFG_TMODE_MODE, 0, 1);
  retval += ubx_val_set(rover, layer, CFG_NAVSPG_DYNMODEL, 6, 1);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

void ublox_rover_loop(ublox_t &rover) {
  const int timeout = 1; // 1ms

  // Setup poll file descriptors
  struct pollfd fds[2];
  // -- UART
  fds[0].fd = rover.uart.connection;
  fds[0].events = POLLIN;
  // -- Server
  fds[1].fd = rover.sockfd;
  fds[1].events = POLLIN;

  // Poll
  while (poll(fds, 2, timeout) >= 0) {
    // Read byte from serial and parse
    if (fds[0].revents & POLLIN) {
      uint8_t data = 0;
      if (uart_read(rover.uart, &data, 1) != 0) {
        continue;
      }
      ublox_parse_ubx(rover, data);
    }

    // Read byte from server (assuming its the RTCM3)
    if (fds[1].fd != -1 && (fds[1].revents & POLLIN)) {
      // Read byte
      uint8_t data = 0;
      if (read(rover.sockfd, &data, 1) != 1) {
        LOG_ERROR("Failed to read RTCM3 byte from server!");
        LOG_ERROR("Ignoring server for now!");
        fds[1].fd = -1;
      }

      // Transmit RTCM3 packet if its ready
      if (rtcm3_parser_update(rover.rtcm3_parser, data)) {
        const uint8_t *msg_data = rover.rtcm3_parser.buf_data;
        const size_t msg_len = rover.rtcm3_parser.msg_len;
        uart_write(rover.uart, msg_data, msg_len);
        rtcm3_parser_reset(rover.rtcm3_parser);
      }
    }
  }
}

int ublox_rover_run(ublox_t &rover,
                    const std::string &base_ip,
                    const int base_port) {
  // Configure rover
  if (ublox_rover_config(rover) != 0) {
    LOG_ERROR("Failed to configure Ublox into ROVER mode!");
    return -1;
  }

  // Create socket
  rover.sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (rover.sockfd == -1) {
    LOG_ERROR("Socket creation failed!");
    return -1;
  } else {
    DEBUG("Created socket!");
  }

  // Assign IP, PORT
  struct sockaddr_in server;
  bzero(&server, sizeof(server));
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(base_ip.c_str());
  server.sin_port = htons(base_port);

  // Connect to server
  const socklen_t len = sizeof(server);
  int retval = connect(rover.sockfd, (struct sockaddr *) &server, len);
  if (retval != 0) {
    LOG_ERROR("Connection with the server failed!");
    return -1;
  } else {
    DEBUG("Connected to the server!");
  }

  // Obtain RTCM3 Messages from server and transmit them to ublox over serial
  ublox_rover_loop(rover);

  // Clean up
  close(rover.sockfd);
  rover.sockfd = -1;

  return 0;
}

} // namespace proto
