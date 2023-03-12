void setup() {
  // put your setup code here, to run once:
  #define RIGHT_FWD  2
  #define RIGHT_RWD  3
  #define LEFT_FWD   14
  #define LEFT_RWD   15 
  
  
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_RWD, OUTPUT);
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_RWD, OUTPUT);

  delay(3000);

  // right forward
  analogWrite(RIGHT_FWD, 150);
  analogWrite(RIGHT_RWD, 0);
  delay(1000);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 0);
  delay(1000);

  // right rewind
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 150);
  delay(1000);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 0);
  delay(1000);

  // left forward
  analogWrite(LEFT_FWD, 150);
  analogWrite(LEFT_RWD, 0);
  delay(1000);
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_RWD, 0);
  delay(1000);
  
  // left forward
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_RWD, 150);
  delay(1000);
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_RWD, 0);
  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 0);
  analogWrite(LEFT_FWD,  0);
  analogWrite(LEFT_RWD,  0);

}
