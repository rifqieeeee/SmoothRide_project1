int rem1 = A2;
int rem2 = A3;
int rem3 = A4;
int rem4 = A5;
int rem1_read, rem2_read, rem3_read, rem4_read;

#define AIN1 10
#define AIN2 11
#define BIN1 12
#define BIN2 13

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void loop() {
  rem1_read = analogRead(rem1);
  rem2_read = analogRead(rem2);
  rem3_read = analogRead(rem3);
  rem4_read = analogRead(rem4);

  if (rem2_read > 200 && rem4_read > 200) {
    maju();
  } else if (rem1_read > 200 && rem3_read > 200) {
    mundur();
  } else if (rem3_read > 200 || rem4_read > 200) {
    kiri();
  } else if (rem1_read > 200 || rem2_read > 200) {
    kanan();
  } else {
    stop();
  }
}

void maju() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void mundur() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void kiri() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void kanan() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void stop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
