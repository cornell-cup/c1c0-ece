void setup() {
  //start serial connection
  Serial.begin(9600);
  //configure pin 2 as an input and enable the internal pull-up resistor
  //pinMode(6, INPUT);
  //pinMode(5, INPUT);
  //pinMode(4, INPUT);
  // reading voltage
  pinMode(13, OUTPUT);
  // FIRST ONE
  pinMode(12, OUTPUT);
  // SECOND ONE
  pinMode(8, OUTPUT);
  // THRID ONE  
}

void loop() {
  //read the input number
  //Serial.print('a');
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
  }
}
