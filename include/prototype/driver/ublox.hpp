#ifndef PROTOTYPE_DRIVER_UBLOX_HPP
#define PROTOTYPE_DRIVER_UBLOX_HPP

#include <stdlib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>

#include "prototype/driver/uart.hpp"
#include "prototype/comm/tcp.hpp"

namespace proto {

/**
 * UBX Class IDs
 * -------------
 * NAV 0x01 Navigation Results: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
 * RXM 0x02 Receiver Manager: Satellite Status, RTC Status
 * INF 0x04 Information: Printf-Style Messages, with IDs such as Error, Warning, Notice
 * ACK 0x05 Ack/Nak: Acknowledge or Reject messages to UBX-CFG input messages
 * CFG 0x06 Configuration Input: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
 * UPD 0x09 Firmware Update: Memory/Flash erase/write, Reboot, Flash identification, etc.
 * MON 0x0A Monitoring: Communication Status, CPU Load, Stack Usage, Task Status
 * TIM 0x0D Timing: Time Pulse Output, Time Mark Results
 * MGA 0x13 Multiple GNSS Assistance: Assistance data for various GNSS
 * LOG 0x21 Logging: Log creation, deletion, info and retrieval
 * SEC 0x27 Security Feature
 */
#define UBX_NAV 0x01
#define UBX_RXM 0x02
#define UBX_INF 0x04
#define UBX_ACK 0x05
#define UBX_CFG 0x06
#define UBX_UPD 0x09
#define UBX_MON 0x0A
#define UBX_TIM 0x0D
#define UBX_MGA 0x13
#define UBX_LOG 0x21
#define UBX_SEC 0x27

/**
 * UBX Class CFG
 * -------------
 * ACK-ACK 0x05 0x01 2 Output Message Acknowledged
 * ACK-NAK 0x05 0x00 2 Output Message Not-Acknowledged
 */
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NAK 0x00

/**
 * UBX Class CFG
 * -------------
 * CFG-VALDEL 0x06 0x8C 4 + 4*N Set Deletes values corresponding to...
 * CFG-VALGET 0x06 0x8B 4 + 4*N Poll Request Get Configuration Items
 * CFG-VALSET 0x06 0x8A 4 + 1*N Set Sets values corresponding to provided...
 */
#define UBX_CFG_VALDEL 0x8C
#define UBX_CFG_VALGET 0x8B
#define UBX_CFG_VALSET 0x8A

/**
 * UBX Class NAV Navigation Results Messages
 * -----------------------------------------
 * NAV-CLOCK 0x01 0x22 20 Periodic/Polled Clock Solution
 * NAV-DOP 0x01 0x04 18 Periodic/Polled Dilution of precision
 * NAV-EOE 0x01 0x61 4 Periodic End Of Epoch
 * NAV-GEOFENCE 0x01 0x39 8 + 2*numFe... Periodic/Polled Geofencing status
 * NAV-HPPOSECEF 0x01 0x13 28 Periodic/Polled High Precision Position Solution in ECEF
 * NAV-HPPOSLLH 0x01 0x14 36 Periodic/Polled High Precision Geodetic Position Solution
 * NAV-ODO 0x01 0x09 20 Periodic/Polled Odometer Solution
 * NAV-ORB 0x01 0x34 8 + 6*numSv Periodic/Polled GNSS Orbit Database Info
 * NAV-POSECEF 0x01 0x01 20 Periodic/Polled Position Solution in ECEF
 * NAV-POSLLH 0x01 0x02 28 Periodic/Polled Geodetic Position Solution
 * NAV-PVT 0x01 0x07 92 Periodic/Polled Navigation Position Velocity Time...
 * NAV-RELPOSNED 0x01 0x3C 64 Periodic/Polled Relative Positioning Information in...
 * NAV-RESETODO 0x01 0x10 0 Command Reset odometer
 * NAV-SAT 0x01 0x35 8 + 12*numSvs Periodic/Polled Satellite Information
 * NAV-SIG 0x01 0x43 8 + 16*numSi... Periodic/Polled Signal Information
 * NAV-STATUS 0x01 0x03 16 Periodic/Polled Receiver Navigation Status
 * NAV-SVIN 0x01 0x3B 40 Periodic/Polled Survey-in data
 * NAV-TIMEBDS 0x01 0x24 20 Periodic/Polled BDS Time Solution
 * NAV-TIMEGAL 0x01 0x25 20 Periodic/Polled Galileo Time Solution
 * NAV-TIMEGLO 0x01 0x23 20 Periodic/Polled GLO Time Solution
 * NAV-TIMEGPS 0x01 0x20 16 Periodic/Polled GPS Time Solution
 * NAV-TIMELS 0x01 0x26 24 Periodic/Polled Leap second event information
 * NAV-TIMEUTC 0x01 0x21 20 Periodic/Polled UTC Time Solution
 * NAV-VELECEF 0x01 0x11 20 Periodic/Polled Velocity Solution in ECEF
 * NAV-VELNED 0x01 0x12 36 Periodic/Polled Velocity Solution in NED
 */
#define UBX_NAV_CLOCK 0x22
#define UBX_NAV_DOP 0x04
#define UBX_NAV_EOE 0x61
#define UBX_NAV_GEOFENCE 0x39
#define UBX_NAV_HPPOSECEF 0x13
#define UBX_NAV_HPPOSLLH 0x14
#define UBX_NAV_ODO 0x09
#define UBX_NAV_ORB 0x34
#define UBX_NAV_POSECEF 0x01
#define UBX_NAV_POSLLH 0x02
#define UBX_NAV_PVT 0x07
#define UBX_NAV_RELPOSNED 0x3C
#define UBX_NAV_RESETODO 0x10
#define UBX_NAV_SAT 0x35
#define UBX_NAV_SIG 0x43
#define UBX_NAV_STATUS 0x03
#define UBX_NAV_SVIN 0x3B
#define UBX_NAV_TIMEBDS 0x24
#define UBX_NAV_TIMEGAL 0x25
#define UBX_NAV_TIMEGLO 0x23
#define UBX_NAV_TIMEGPS 0x20
#define UBX_NAV_TIMELS 0x26
#define UBX_NAV_TIMEUTC 0x21
#define UBX_NAV_VELECEF 0x11
#define UBX_NAV_VELNED 0x12

/**
 * UBX Class RXM Receiver Manager Messages
 * ---------------------------------------
 * RXM-MEASX 0x02 0x14 44 + 24*num... Periodic/Polled Satellite Measurements for RRLP
 * RXM-PMREQ 0x02 0x41 8 Command Requests a Power Management task
 * RXM-PMREQ 0x02 0x41 16 Command Requests a Power Management task
 * RXM-RAWX 0x02 0x15 16 + 32*num... Periodic/Polled Multi-GNSS Raw Measurement Data
 * RXM-RLM 0x02 0x59 16 Output Galileo SAR Short-RLM report
 * RXM-RLM 0x02 0x59 28 Output Galileo SAR Long-RLM report
 * RXM-RTCM 0x02 0x32 8 Output RTCM input status
 * RXM-SFRBX 0x02 0x13 8 + 4*numW... Output
 */
#define UBX_RXM_MEASX 0x14
#define UBX_RXM_PMREQ 0x41
#define UBX_RXM_RAWX 0x15
#define UBX_RXM_RLM 0x59
#define UBX_RXM_RTCM 0x32
#define UBX_RXM_SFRBX 0x13

#define CFG_SIGNAL_GPS_ENA 0x1031001f
#define CFG_SIGNAL_GPS_L1CA_ENA 0x10310001
#define CFG_SIGNAL_QZSS_ENA 0x10310024
#define CFG_SIGNAL_BDS_B2_ENA 0x1031000e

#define CFG_RATE_MEAS 0x30210001
#define CFG_UART1_BAUDRATE 0x40520001
#define CFG_USBOUTPROT_NMEA 0x10780002

#define CFG_MSGOUT_RTCM_3X_TYPE1005_USB 0x209102c0
#define CFG_MSGOUT_RTCM_3X_TYPE1077_USB 0x209102cf
#define CFG_MSGOUT_RTCM_3X_TYPE1087_USB 0x209102d4
#define CFG_MSGOUT_RTCM_3X_TYPE1097_USB 0x2091031b
#define CFG_MSGOUT_RTCM_3X_TYPE1127_USB 0x209102d9
#define CFG_MSGOUT_RTCM_3X_TYPE1230_USB 0x20910306

#define CFG_MSGOUT_UBX_NAV_CLOCK_USB 0x20910068
#define CFG_MSGOUT_UBX_NAV_HPPOSEECF_USB 0x20910031
#define CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB 0x20910036
#define CFG_MSGOUT_UBX_NAV_RELPOSNED_USB 0x20910090
#define CFG_MSGOUT_UBX_NAV_SVIN_USB 0x2091008b
#define CFG_MSGOUT_UBX_NAV_STATUS_USB 0x2091001d
#define CFG_MSGOUT_UBX_NAV_PVT_USB 0x20910009
#define CFG_MSGOUT_UBX_RXM_RTCM_USB 0x2091026b

#define CFG_TMODE_MODE 0x20030001
#define CFG_TMODE_SVIN_MIN_DUR 0x40030010
#define CFG_TMODE_SVIN_ACC_LIMIT 0x40030011

#define CFG_NAVSPG_DYNMODEL 0x20110021


/*****************************************************************************
 * UBX Message
 ****************************************************************************/

/**
 * UBX Message
 */
struct ubx_msg_t {
  bool ok = false;

  uint8_t msg_class = 0;
  uint8_t msg_id = 0;
  uint16_t payload_length = 0;
  uint8_t payload[1024] = {0};
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  ubx_msg_t();
  ubx_msg_t(const uint8_t *msg_data);
  ubx_msg_t(const uint8_t msg_class_,
            const uint8_t msg_id_,
            const uint16_t length_,
            const uint8_t *payload_);
};

void ubx_msg_parse(ubx_msg_t &msg, const uint8_t *data);
void ubx_msg_serialize(const ubx_msg_t &msg,
                       uint8_t *frame,
                       size_t &frame_size);
void ubx_msg_checksum(const uint8_t msg_class,
                      const uint8_t msg_id,
                      const uint16_t payload_length,
                      const uint8_t *payload,
                      uint8_t &ck_a,
                      uint8_t &ck_b);
void ubx_msg_checksum(const ubx_msg_t &msg, uint8_t &ck_a, uint8_t &ck_b);
bool ubx_msg_is_valid(const ubx_msg_t &msg);
void ubx_msg_print(const ubx_msg_t &msg);

/**
 * UBX-NAV-SVIN
 */
struct ubx_nav_svin_t{
  uint32_t itow = 0;
  uint32_t dur = 0;
  int32_t mean_x = 0;
  int32_t mean_y = 0;
  int32_t mean_z = 0;
  int8_t mean_xhp = 0;
  int8_t mean_yhp = 0;
  int8_t mean_zhp = 0;
  uint32_t mean_acc = 0;
  uint32_t obs = 0;
  uint8_t valid = 0;
  uint8_t active = 0;

  ubx_nav_svin_t() {}
  ubx_nav_svin_t(const ubx_msg_t &msg) {
    // ubx_msg_print(msg);
    itow = u32bit(msg.payload, 4);
    dur = u32bit(msg.payload, 8);
    mean_x = s32bit(msg.payload, 12);
    mean_y = s32bit(msg.payload, 16);
    mean_z = s32bit(msg.payload, 20);
    mean_xhp = s8bit(msg.payload, 24);
    mean_yhp = s8bit(msg.payload, 25);
    mean_zhp = s8bit(msg.payload, 26);
    mean_acc = u32bit(msg.payload, 28);
    obs = u32bit(msg.payload, 32);
    valid = u8bit(msg.payload, 36);
    active = u8bit(msg.payload, 37);
  }
};

void print_ubx_nav_svin(const ubx_nav_svin_t &msg) {
  printf("[nav-svin] ");
  printf("itow: %d", msg.itow);
  printf("\t");
  printf("dur: %d", msg.dur);
  printf("\t");
  printf("mean_x: %d", msg.mean_x);
  printf("\t");
  printf("mean_y: %d", msg.mean_y);
  printf("\t");
  printf("mean_z: %d", msg.mean_z);
  printf("\t");
  printf("active: 0x%02x", msg.active);
  printf("\t");
  printf("valid: 0x%02x", msg.valid);
  printf("\n");
}

/**
 * UBX-NAV-STATUS
 */
struct ubx_nav_status_t {
  uint32_t itow = 0;
  uint8_t fix = 0;
  uint8_t flags = 0;
  uint8_t fix_status = 0;
  uint8_t flags2 = 0;
  uint32_t ttff = 0;
  uint32_t msss = 0;

  ubx_nav_status_t() {}
  ubx_nav_status_t(const ubx_msg_t &msg) {
    itow = u32bit(msg.payload, 0);
    fix = u8bit(msg.payload, 4);
    flags = u8bit(msg.payload, 5);
    fix_status = u8bit(msg.payload, 6);
    flags2 = u8bit(msg.payload, 7);
    ttff = u32bit(msg.payload, 8);
    msss = u32bit(msg.payload, 12);
  }
};

void print_ubx_nav_status(const ubx_nav_status_t &msg) {
  printf("[nav-status] ");
  printf("itow: %d", msg.itow);

  printf("\t");
  switch(msg.fix) {
  case 0x00:
    printf("fix: no fix");
    break;
  case 0x01:
    printf("fix: dead reckoning only");
    break;
  case 0x02:
    printf("fix: 2D-fix");
    break;
  case 0x03:
    printf("fix: 3D-fix");
    break;
  case 0x04:
    printf("fix: GNSS + dead reckoning combined");
    break;
  case 0x05:
    printf("fix: time only fix");
    break;
  }
  printf("\t");

  printf("flags: %d", msg.flags);
  printf("\t");

  const bool diff_corr = (msg.fix_status & 0b00000001);
  if (diff_corr) {
    printf("diff corr avail?: true");
  } else {
    printf("diff corr avail?: false");
  }
  printf("\t");
  printf("\t");

  const uint8_t map_matching = (msg.fix_status & 0b11000000);
  if (map_matching == 0b00) {
    printf("map matching: none");
  } else if (map_matching == 0b01) {
    printf("map matching: valid but not used");
  } else if (map_matching == 0b10) {
    printf("map matching: valid and used");
  } else if (map_matching == 0b11) {
    printf("map matching: valid and used");
  }
  printf("\t");

  printf("flags2: %d", msg.flags2);
  printf("\t");

  printf("ttff: %d", msg.ttff);
  printf("\t");

  printf("msss: %d", msg.msss);
  printf("\n");
}

/**
 * UBX-NAV-PVT
 */
struct ubx_nav_pvt_t {
  uint32_t itow = 0;
  uint16_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t min = 0;
  uint8_t sec = 0;
  uint8_t valid = 0;
  uint32_t tacc = 0;
  int32_t nano = 0;
  uint8_t fix_type = 0;
  uint8_t flags = 0;
  uint8_t flags2 = 0;
  uint8_t num_sv = 0;
  uint32_t lon = 0;
  uint32_t lat = 0;
  uint32_t height = 0;
  uint32_t hmsl = 0;
  int32_t hacc = 0;
  int32_t vacc = 0;
  int32_t veln = 0;
  int32_t vele = 0;
  int32_t veld = 0;
  int32_t gspeed = 0;
  int32_t headmot = 0;
  uint32_t sacc = 0;
  uint32_t headacc = 0;
  uint16_t pdop = 0;
  int32_t headveh = 0;
  int16_t magdec = 0;
  uint16_t magacc = 0;

  ubx_nav_pvt_t() {}
  ubx_nav_pvt_t(const ubx_msg_t &msg) {
    itow = u32bit(msg.payload, 0);
    year = u16bit(msg.payload, 4);
    month = u8bit(msg.payload, 6);
    day = u8bit(msg.payload, 7);
    hour = u8bit(msg.payload, 8);
    min = u8bit(msg.payload, 9);
    sec = u8bit(msg.payload, 10);
    valid = u8bit(msg.payload, 11);
    tacc = u32bit(msg.payload, 12);
    nano = s32bit(msg.payload, 16);
    fix_type = u8bit(msg.payload, 20);
    flags = u8bit(msg.payload, 21);
    flags2 = u8bit(msg.payload, 22);
    num_sv = u8bit(msg.payload, 23);
    lon = s32bit(msg.payload, 24);
    lat = s32bit(msg.payload, 28);
    height = s32bit(msg.payload, 32);
    hmsl = s32bit(msg.payload, 36);
    hacc = u32bit(msg.payload, 40);
    vacc = u32bit(msg.payload, 44);
    veln = s32bit(msg.payload, 48);
    vele = s32bit(msg.payload, 52);
    veld = s32bit(msg.payload, 56);
    gspeed = s32bit(msg.payload, 60);
    headmot = s32bit(msg.payload, 64);
    sacc = u32bit(msg.payload, 68);
    headacc = u32bit(msg.payload, 72);
    pdop = u16bit(msg.payload, 76);
    headveh = s32bit(msg.payload, 84);
    magdec = s16bit(msg.payload, 88);
    magacc = u16bit(msg.payload, 90);
  }
};

void print_ubx_nav_pvt(const ubx_nav_pvt_t &msg) {
  printf("[nav-hpposllh] ");
  printf("itow: %d", msg.itow);
  printf("\t");
  printf("lon: %d", (int32_t) msg.lon);
  printf("\t");
  printf("lat: %d", (int32_t) msg.lat);
  printf("\t");
  printf("height: %d", msg.height);
  printf("\t");
  switch(msg.fix_type) {
  case 0:
    printf("Fix type: no fix");
    break;
  case 1:
    printf("Fix type: dead reckoning only");
    break;
  case 2:
    printf("Fix type: 2D-fix");
    break;
  case 3:
    printf("Fix type: 3D-fix");
    break;
  case 4:
    printf("Fix type: GNSS + dead reckoning combined");
    break;
  case 5:
    printf("Fix type: time only fix");
    break;
  }
  printf("\n");
}

/**
 * UBX-NAV-HPPOSLLH
 */
struct ubx_nav_hpposllh_t {
  uint8_t version = 0;
  uint32_t itow = 0;
  int32_t lon = 0;
  int32_t lat = 0;
  int32_t height = 0;
  int32_t hmsl = 0;
  int8_t lon_hp = 0;
  int8_t lat_hp = 0;
  int8_t height_hp = 0;
  int8_t hmsl_hp = 0;
  uint32_t hacc = 0;
  uint32_t vacc = 0;

  ubx_nav_hpposllh_t() {}
  ubx_nav_hpposllh_t(const ubx_msg_t &msg) {
    version = u8bit(msg.payload, 0);
    itow = u32bit(msg.payload, 4);
    lon = s32bit(msg.payload, 8);
    lat = s32bit(msg.payload, 12);
    height = s32bit(msg.payload, 16);
    hmsl = s32bit(msg.payload, 20);
    lon_hp = s8bit(msg.payload, 24);
    lat_hp = s8bit(msg.payload, 25);
    height_hp = s8bit(msg.payload, 26);
    hmsl_hp = s8bit(msg.payload, 27);
    hacc = u32bit(msg.payload, 28);
    vacc = u32bit(msg.payload, 32);
  }
};

void print_ubx_nav_hpposllh(const ubx_nav_hpposllh_t &msg) {
  printf("[nav-hpposllh] ");
  printf("itow: %d", msg.itow);
  printf("\t");
  printf("lon: %d", (int32_t) msg.lon);
  printf("\t");
  printf("lat: %d", (int32_t) msg.lat);
  printf("\t");
  printf("height: %d", msg.height);
  printf("\t");
  printf("hmsl: %d", msg.hmsl);
  printf("\t");
  printf("lon_hp: %d", msg.lon_hp);
  printf("\t");
  printf("lat_hp: %d", msg.lat_hp);
  printf("\t");
  printf("height_hp: %d", msg.height_hp);
  printf("\t");
  printf("hmsl_hp: %d", msg.hmsl_hp);
  printf("\t");
  printf("hacc: %d", msg.hacc);
  printf("\t");
  printf("vacc: %d", msg.vacc);
  printf("\n");
}

/**
 * UBX-RXM-RTCM
 */
struct ubx_rxm_rtcm_t {
  uint8_t flags = 0;
  uint16_t sub_type = 0;
  uint16_t ref_station = 0;
  uint16_t msg_type = 0;

  ubx_rxm_rtcm_t() {}
  ubx_rxm_rtcm_t(const ubx_msg_t &msg) {
    flags = msg.payload[1];
    sub_type = u16bit(msg.payload, 2);
    ref_station = u16bit(msg.payload, 4);
    msg_type = u16bit(msg.payload, 6);
  }
};

void print_ubx_rxm_rtcm(const ubx_rxm_rtcm_t &msg) {
  printf("GOT RTCM3 msg type: [%d]", msg.msg_type);
  printf("\t");
  if (msg.flags == 0) {
    printf("RTCM OK!");
  } else {
    printf("RTCM NOT OK!");
  }
  printf("\n");
}



/*****************************************************************************
 * UBX Stream Parser
 ****************************************************************************/

/**
 * UBX Stream Parser States
 */
#define SYNC_1 0
#define SYNC_2 1
#define MSG_CLASS 2
#define MSG_ID 3
#define PAYLOAD_LENGTH_LOW 4
#define PAYLOAD_LENGTH_HI 5
#define PAYLOAD_DATA 6
#define CK_A 7
#define CK_B 8

/**
 * UBX Stream Parser
 */
struct ubx_parser_t {
  int state = SYNC_1;
  uint8_t buf_data[9046] = {0};
  size_t buf_pos = 0;
  ubx_msg_t msg;

  ubx_parser_t();
};

void ubx_parser_reset(ubx_parser_t &parser);
int ubx_parser_update(ubx_parser_t &parser, uint8_t data);



/*****************************************************************************
 * RTCM3 Stream Parser
 ****************************************************************************/

/**
 * RTCM3 Stream Parser
 */
struct rtcm3_parser_t {
  uint8_t buf_data[9046] = {0};
  size_t buf_pos = 0;
  size_t msg_len = 0;
  size_t msg_type = 0;

  rtcm3_parser_t();
};

void rtcm3_parser_reset(rtcm3_parser_t &parser);

/**
 * RTCM 3.2 Frame
 * --------------
 * Byte 0: Always 0xD3
 * Byte 1: 6-bits of zero
 * Byte 2: 10-bits of length of this packet including the first two-ish header
 *         bytes, + 6.
 * byte 3 + 4: Msg type 12 bits
 *
 * Example [Msg type 1087]:
 *
 *   D3 00 7C 43 F0 ...
 *
 * Where 0x7C is the payload size = 124
 * = 124 + 6 [header]
 * = 130 total bytes in this packet
 */
int rtcm3_parser_update(rtcm3_parser_t &parser, uint8_t data);



/*****************************************************************************
 * UBlox
 ****************************************************************************/

/**
 * UBlox
 */
struct ublox_t {
  bool ok = false;

  uart_t uart;
  std::vector<int> conns;
  int server_socket = -1;
  int client_socket = -1;

  std::string msg_type = "";  // Current msg type
  ubx_parser_t ubx_parser;
  rtcm3_parser_t rtcm3_parser;

  ubx_nav_svin_t nav_svin;
  ubx_nav_status_t nav_status;
  ubx_nav_pvt_t nav_pvt;
  ubx_nav_hpposllh_t nav_hpposllh;
  ubx_rxm_rtcm_t rxm_rtcm;

  FILE *hpposllh_data = nullptr;

  ublox_t(const std::string &port="/dev/ttyACM0", const int speed=B57600);
  ublox_t(const uart_t &uart);
  ~ublox_t();
};

int ubx_write(const ublox_t &ublox,
              uint8_t msg_class,
              uint8_t msg_id,
              uint16_t length,
              uint8_t *payload);
int ubx_poll(const ublox_t &ublox,
             const uint8_t msg_class,
             const uint8_t msg_id,
             uint16_t &payload_length,
             uint8_t *payload,
             const bool expect_ack=false,
             const int retry=5);
int ubx_read_ack(const ublox_t &ublox, uint8_t msg_class, uint8_t msg_id);
int ubx_val_get(const ublox_t &ublox,
                const uint8_t layer,
                const uint32_t key,
                uint32_t &val);
int ubx_val_set(const ublox_t &ublox,
                const uint8_t layer,
                const uint32_t key,
                const uint32_t val,
                const uint8_t val_size);

void ublox_version(const ublox_t &ublox);
int ublox_parse_ubx(ublox_t &ublox, uint8_t data);
int ublox_parse_rtcm3(ublox_t &ublox, uint8_t data);



/*****************************************************************************
 * UBlox Base Station
 ****************************************************************************/

/**
 * UBlox Base Station
 */
struct ublox_base_station_t : public ublox_t {
  std::vector<int> conns;
  int sockfd = -1;
  ublox_base_station_t();
};

int ublox_base_station_config(ublox_t &base);
void ublox_base_station_loop(ublox_t &base);
int ublox_base_station_run(ublox_t &base, const int port=8080);



/*****************************************************************************
 * UBlox Rover
 ****************************************************************************/

/**
 * UBlox Rover
 */
struct ublox_rover_t : public ublox_t {
  int sockfd = -1;
  ublox_rover_t();
};

int ublox_rover_config(ublox_t &rover);
void ublox_rover_loop(ublox_t &rover);
int ublox_rover_run(ublox_t &rover,
                    const std::string &base_ip="127.0.0.1",
                    const int base_port=8080);

} //  namespace proto
#endif // PROTOTYPE_DRIVER_UBLOX_HPP
