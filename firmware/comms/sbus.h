/******************************************************************************
 *                                  SBUS
 *****************************************************************************/

#define SBUS_READY 0

typedef struct sbus_t {
  int8_t status;
  uint32_t connfd;
  uint16_t ch[16];
  uint8_t frame_lost;
  uint8_t failsafe_activated;
} sbus_t;

void sbus_setup(sbus_t *sbus) {
  /* Setup clocks */
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_USART2);

  /* Setup GPIO pin for transmit and receive. */
  // clang-format off
  gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART2_TX);
  gpio_set_mode(GPIOA,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN,
                GPIO_USART2_RX);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  // clang-format on

  /**
   * The SBUS protocol uses inverted serial logic with:
   * - Baud rate of 100000
   * - 8 data bits
   * - Even parity bit
   * - 2 stop bits
   */
  usart_set_baudrate(USART2, 100000);
  usart_set_databits(USART2, 8);
  usart_set_parity(USART2, USART_PARITY_EVEN);
  usart_set_stopbits(USART2, USART_STOPBITS_2);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

  sbus->connfd = USART2;
  sbus->status = SBUS_READY;
}

void sbus_connect(const sbus_t *sbus) { usart_enable(sbus->connfd); }

void sbus_disconnect(const sbus_t *sbus) { usart_disable(sbus->connfd); }

void sbus_read_byte(const sbus_t *sbus, uint8_t *data) {
  *data = usart_recv_blocking(sbus->connfd);
}

void sbus_read(const sbus_t *sbus, uint8_t *data, const size_t size) {
  for (size_t i = 0; i < size; i++) {
    data[i] = usart_recv_blocking(sbus->connfd);
  }
}

void sbus_write_byte(const sbus_t *sbus, const uint8_t data) {
  usart_send_blocking(sbus->connfd, data);
}

void sbus_write(const sbus_t *sbus, const uint8_t *data, const size_t size) {
  for (size_t i = 0; i < size; i++) {
    usart_send_blocking(sbus->connfd, data[i]);
  }
}

void sbus_update(sbus_t *sbus, serial_t *serial) {
  // Get sbus frame data
  uint8_t frame[25];
  sbus_read_byte(sbus, &frame[0]);
  if (frame[0] != 0x0F) {
    return;
  } else {
    sbus_read(sbus, &frame[1], 24);
  }

  // Parse sbus frame
  // -- Parse flag
  sbus->frame_lost = (frame[23] & (1 << 5));
  sbus->failsafe_activated = (frame[23] & (1 << 4));
  // -- Parse channel data
  sbus->ch[0] = ((frame[1] | frame[2] << 8) & 0x07FF);
  sbus->ch[1] = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF);
  sbus->ch[2] = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF);
  sbus->ch[3] = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF);
  sbus->ch[4] = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF);
  sbus->ch[5] = ((frame[7] >> 7 | frame[8] << 1 | frame[8] << 9) & 0x07FF);
  sbus->ch[6] = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF);
  sbus->ch[7] = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF);
  sbus->ch[8] = ((frame[12] | frame[13] << 8) & 0x07FF);
  sbus->ch[9] = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF);
  sbus->ch[10] = ( (frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
  sbus->ch[11] = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF);
  sbus->ch[12] = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF);
  sbus->ch[13] = ( (frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
  sbus->ch[14] = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF);
  sbus->ch[15] = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF);

  char ch_str[10] = {0};
  itoa(sbus->ch[0], ch_str, 10);

  /* serial_print(serial, ch_str); */
  /* serial_print(serial, "\r\n"); */
}
