void setup() {
  //start serial connection
  Serial.begin(9600);
  //read value
  pinMode(4, INPUT);
  pinMode(3, INPUT);
  pinMode(2, INPUT);
  // LED lights
  pinMode(13, OUTPUT);
  // FIRST ONE
  pinMode(12, OUTPUT);
  // SECOND ONE
  pinMode(11, OUTPUT);
  // THRID ONE  
}

void loop() {
  //read the input number
  int Val1 = digitalRead(4);
  int Val2 = digitalRead(3);
  int Val3 = digitalRead(2);
  //print
  Serial.print(Val1);
  Serial.print(Val2);
  Serial.print(Val3);
  Serial.println('x');
  // LED1
  if (Val1 == HIGH)  {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  } 
  // LED2
  if (Val2 == HIGH)  {
    digitalWrite(12, HIGH);
  } else {
    digitalWrite(12, LOW);
  } 
  // LED3
  if (Val3 == HIGH)  {
    digitalWrite(11, HIGH);
  } else {
    digitalWrite(11, LOW);
  } 
  //Serial.print('a');
  /*
  if (Serial.available() > 1) {
    int input = Serial.read();
    //if is valid input
    //Serial.print('b');
    int Val = 0;
    if ((input >= '0') && (input <= '7')) {
      Val = input;
    }
    //convert to seperate bits
    int Val1 = bitRead(Val,2);
    int Val2 = bitRead(Val,1);
    int Val3 = bitRead(Val,0); 
    //print out the value
    Serial.print(Val1);
    Serial.print(Val2);
    Serial.print(Val3);
    Serial.println('x');
    // LED1
    if (Val1 == HIGH)  {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    } 
    // LED2
    if (Val2 == HIGH)  {
      digitalWrite(12, HIGH);
    } else {
      digitalWrite(12, LOW);
    } 
    // LED3
    if (Val3 == HIGH)  {
      digitalWrite(8, HIGH);
    } else {
      digitalWrite(8, LOW);
    } 
  }*/
  
}
