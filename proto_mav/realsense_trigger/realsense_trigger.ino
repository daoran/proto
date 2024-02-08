const int pwm_pin = 0;
const int period_on_us = 100;
const int period_off_us = 10000 - period_on_us;
int led_state = 0;

void setup() {
  pinMode(pwm_pin, OUTPUT);
  pinMode(13, OUTPUT);
  analogWrite(13, 255);
}

void loop() {
  analogWrite(pwm_pin, 255);
  delayMicroseconds(period_on_us);
  analogWrite(pwm_pin, 0);
  delayMicroseconds(period_off_us);

  analogWrite(13, 255 * led_state);
  led_state = !led_state;
}