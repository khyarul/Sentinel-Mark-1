//encoder 22 PPR

void initEncoder() {
  pinMode(PA14, INPUT_PULLDOWN); //line 14 PA14
  pinMode(PA15, INPUT_PULLDOWN); //line 15 PA15
  pinMode(PB15, INPUT_PULLDOWN); //line 15 PB15
  pinMode(PB14, INPUT_PULLDOWN); //line 14 PB14
  enc_ON();
}

void enc_ON() {
  attachInterrupt(PA14, enL, CHANGE);
  //attachInterrupt(20, enL, CHANGE);
  attachInterrupt(PB15, enR, CHANGE);
  //attachInterrupt(29, enR, CHANGE);
}

void enc_OFF() {
  detachInterrupt(PA14);
  //attachInterrupt(20, enL, CHANGE);
  detachInterrupt(PB15);
  //attachInterrupt(29, enR, CHANGE);
}

void enL() {
  int8 en1L = gpio_read_bit(GPIOA, 14) ? HIGH : LOW;
  int8 en2L = gpio_read_bit(GPIOA, 15) ? HIGH : LOW;
  if (en1L != last_en1L) {
    if (en2L != en1L) {
      lPos--;
    }
    else {
      lPos++;
    }
  }
  last_en1L = en1L;
}

void enR() {
  int8 en1R = gpio_read_bit(GPIOB, 15) ? HIGH : LOW;
  int8 en2R = gpio_read_bit(GPIOB, 14) ? HIGH : LOW;
  if (en1R != last_en1R) {
    if (en2R != en1R) {
      rPos--;
    }
    else {
      rPos++;
    }
  }
  last_en1R = en1R;
}
