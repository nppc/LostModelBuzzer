#define OUT_PIN  10
String inString = "";
unsigned int intNum;

void setup() {
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, HIGH);
  Serial.begin(9600);
  Serial.println("Set Serial window to 'Newiine' mode...");
  
}

void loop() {
  //wait for text numeric value
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if(isDigit(inChar)) {
      inString += (char)inChar;
    }
    if(inChar=='\n') {
      //Data is ready, lets send it...
      intNum = inString.toInt();
      Serial.print(intNum);
      sendNum(intNum);
      Serial.println(" this number is sent.");
      inString = "";
    }
    }
}

// send low bit first
void sendNum(unsigned int nb) {
  digitalWrite(OUT_PIN, HIGH);
  delay(50);
  byte bitLen;
  for(byte i=0;i<16;i++) {
    if((nb >> i) & 1)
        bitLen=9;
    else
        bitLen=4;
    digitalWrite(OUT_PIN, LOW);
    delay(1);
    digitalWrite(OUT_PIN, HIGH);
    delay(bitLen);
  }
  digitalWrite(OUT_PIN, LOW);
  delay(1);
  digitalWrite(OUT_PIN, HIGH);
}
