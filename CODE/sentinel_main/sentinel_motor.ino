void setMotor(int L, int R) {
  if (L > 0) {
    gpio_write_bit(GPIOB, 8, LOW);
  }
  else {
    L = L + 65535;
    gpio_write_bit(GPIOB, 8, HIGH);
  }
  pwmWrite(pwmL, L);
  if (R > 0) {
    gpio_write_bit(GPIOA, 8, LOW);
  }
  else {
    R = R + 65535;
    gpio_write_bit(GPIOA, 8, HIGH);
  }
  pwmWrite(pwmR, R);
}

void initMotor() {
  //timer.setPeriod(1020);
  //timer.refresh();
  pinMode(pwmL, PWM);
  pinMode(pwmR, PWM);
  pinMode(dirL, OUTPUT);
  pinMode(dirR, OUTPUT);
}
