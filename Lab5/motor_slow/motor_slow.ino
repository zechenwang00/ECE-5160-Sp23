void setup() {
  // put your setup code here, to run once:
  #define RIGHT_FWD  15
  #define RIGHT_RWD  14
  #define LEFT_FWD   3
  #define LEFT_RWD   2 
  #define OFFSET     20
  
  
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_RWD, OUTPUT);
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_RWD, OUTPUT);

  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 0);
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_RWD, 0);
  delay(6000);

  // forward
  analogWrite(RIGHT_FWD, 50 + OFFSET);
  analogWrite(LEFT_FWD, 50);
  delay(2000);

  // forward
  analogWrite(RIGHT_FWD, 40);
  analogWrite(LEFT_FWD, 40);
  delay(2000);

  // forward
  analogWrite(RIGHT_FWD, 30);
  analogWrite(LEFT_FWD, 30);
  delay(2000);


  // stop
  analogWrite(RIGHT_FWD, 0);
  analogWrite(LEFT_FWD, 0);
  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 0);
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_RWD, 0);

}
