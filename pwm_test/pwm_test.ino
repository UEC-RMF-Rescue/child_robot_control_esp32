// if wanted to use micro ros, board manager must be less than 2.0.2

#define VCC 4
#define MIN1 12
#define MIN2 13
#define ENA 19
#define ENB 20

void setup() {
  pinMode(VCC, OUTPUT);
  pinMode(ENA, INPUT);
  pinMode(ENB, INPUT);

  ledcAttach(MIN1, 50000, 8);
  ledcAttach(MIN2, 50000, 8);

  digitalWrite(VCC, HIGH);
}

void loop() {
  ledcWrite(MIN1, 170);
  ledcWrite(MIN2, 0);
  delay(3000);
  ledcWrite(MIN1, 256);
  ledcWrite(MIN2, 256);
  delay(1000);
  ledcWrite(MIN1, 0);
  ledcWrite(MIN2, 170);
  delay(3000);
  ledcWrite(MIN1, 256);
  ledcWrite(MIN2, 256);
  delay(1000);

  ledcWrite(MIN1, 80);
  digitalWrite(MIN2, LOW);
  delay(3000);
  digitalWrite(MIN1, HIGH);
  digitalWrite(MIN2, HIGH);
  delay(1000);

  ledcWrite(MIN1, 128);
  digitalWrite(MIN2, LOW);
  delay(3000);
  digitalWrite(MIN1, HIGH);
  digitalWrite(MIN2, HIGH);
  delay(1000);
}
