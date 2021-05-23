#pragma once

void sbus_setup() {
  // The SBUS protocol uses inverted serial logic with:
  // - Baud rate of 100000
  // - 8 data bits
  // - Even parity bit
  // - 2 stop bits
  HardwareSerial *bus = &Serial1;
  bus->begin(100000, SERIAL_8E2);
  bus->flush();
}

uint8_t sbus_read_byte() {
  return Serial1.read();
}

void sbus_read(uint8_t *data, const size_t size) {
  Serial1.readBytes(data, size);
}

void sbus_write_byte(const uint8_t data) {
  Serial1.write(data);
}

void sbus_write(const uint8_t *data, const size_t size) {
  Serial1.write(data, size);
}

void sbus_update(uart_t *uart) {
  // Get sbus frame data
  /* uint8_t frame[25] = {0}; */
  /* frame[0] = sbus_read_byte(); */
  /* if (frame[0] != 0x0F) { */
  /*   uart_printf(uart, "Failed!\n"); */
  /*   return; */
  /* } else { */
  /*   sbus_read(&frame[1], 24); */
  /* } */
  if (Serial1.available()) {
    uart_printf(uart, "%X\n", sbus_read_byte());
  } else {
    uart_printf(uart, "nothing...\n");
  }

  // Parse sbus frame
  // -- Parse flag
  /* uint8_t frame_lost = (frame[23] & (1 << 5)); */
  /* uint8_t failsafe_activated = (frame[23] & (1 << 4)); */
  // -- Parse channel data
  /* uint16_t ch[16] = {0}; */
  /* ch[0] = ((frame[1] | frame[2] << 8) & 0x07FF); */
  /* ch[1] = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF); */
  /* ch[2] = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF); */
  /* ch[3] = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF); */
  /* ch[4] = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF); */
  /* ch[5] = ((frame[7] >> 7 | frame[8] << 1 | frame[8] << 9) & 0x07FF); */
  /* ch[6] = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF); */
  /* ch[7] = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF); */
  /* ch[8] = ((frame[12] | frame[13] << 8) & 0x07FF); */
  /* ch[9] = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF); */
  /* ch[10] = ( (frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF); */
  /* ch[11] = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF); */
  /* ch[12] = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF); */
  /* ch[13] = ( (frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF); */
  /* ch[14] = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF); */
  /* ch[15] = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF); */
  /*  */
  /* char ch_str[10] = {0}; */
  /* itoa(ch[0], ch_str, 10); */

  /* uart_printf(uart, "HERE!\n"); */
  /* uart_printf(uart, "%d, ", ch[0]); */
  /* uart_printf(uart, "%d, ", ch[1]); */
  /* uart_printf(uart, "%d, ", ch[2]); */
  /* uart_printf(uart, "%d\n", ch[3]); */

  /* uart_printf(uart, ch_str); */
  /* uart_printf(uart, "\r\n"); */
}
