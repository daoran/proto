#include <Wire.h>
#include <Arduino.h>

class uart_t {
  public:
    Stream *serial_in;
    Print *serial_out;

    uart_t() {
      serial_in = &Serial;
      serial_out = &Serial;
      Serial.begin(115200);
    }

    void read_byte(uint8_t &b) {
      b = serial_in->read();
    }

    void read_bytes(uint8_t *data, const size_t len) {
      serial_in->readBytes(data, len);
    }

    void write_byte(const uint8_t b) {
      serial_out->write(b);
    }

    void write_bytes(const uint8_t *data, const size_t len) {
      serial_out->write(data, len);
    }

    /**
      Simple printf for writing to an Arduino serial port.  Allows specifying
      Serial..Serial3.

      const HardwareSerial&, the serial port to use (Serial..Serial3)
      const char* fmt, the formatting string followed by the data to be formatted

          int d = 65;
          float f = 123.4567;
          char* str = "Hello";
          uart_t::printf(Serial, "<fmt>", d);

      Example:
        uart_t::printf(Serial, "Sensor %d is %o and reads %1f\n", d, d, f) will
        output "Sensor 65 is on and reads 123.5" to the serial port.

      Formatting strings <fmt>
      %B    - binary (d = 0b1000001)
      %b    - binary (d = 1000001)
      %c    - character (s = H)
      %d/%i - integer (d = 65)\
      %f    - float (f = 123.45)
      %3f   - float (f = 123.346) three decimal places specified by %3.
      %s    - char* string (s = Hello)
      %X    - hexidecimal (d = 0x41)
      %x    - hexidecimal (d = 41)
      %%    - escaped percent ("%")
    **/
    void printf(const char* fmt, ...) {
      va_list argv;
      va_start(argv, fmt);

      for (int i = 0; fmt[i] != '\0'; i++) {
        if (fmt[i] == '%') {
          // Look for specification of number of decimal places
          int places = 2;
          if (fmt[i+1] >= '0' && fmt[i+1] <= '9') {
            places = fmt[i+1] - '0';
            i++;
          }

          switch (fmt[++i]) {
            case 'B': serial_out->print("0b");
            case 'b': serial_out->print(va_arg(argv, int), BIN); break;
            case 'c': serial_out->print((char) va_arg(argv, int)); break;
            case 'd':
            case 'i': serial_out->print(va_arg(argv, int), DEC); break;
            case 'f': serial_out->print(va_arg(argv, double), places); break;
            case 'l': serial_out->print(va_arg(argv, long), DEC); break;
            case 's': serial_out->print(va_arg(argv, const char*)); break;
            case 'X':
              serial_out->print("0x");
              serial_out->print(va_arg(argv, int), HEX);
              break;
            case '%': serial_out->print(fmt[i]); break;
            default: serial_out->print("?"); break;
          }
        } else {
          serial_out->print(fmt[i]);
        }
      }
      va_end(argv);
    }
};
