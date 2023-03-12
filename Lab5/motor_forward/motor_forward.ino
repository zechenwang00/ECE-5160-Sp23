void setup() {
  // put your setup code here, to run once:
  #define RIGHT_FWD  15
  #define RIGHT_RWD  14
  #define LEFT_FWD   3
  #define LEFT_RWD   2 
  
  
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_RWD, OUTPUT);
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_RWD, OUTPUT);

  delay(6000);

  // forward
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 150);
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_RWD, 150);
  delay(1000);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 0);
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_RWD, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 0);
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_RWD, 0);

}
