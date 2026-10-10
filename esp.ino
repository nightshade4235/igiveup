/*
  ROBOT M - ESP32-S3-DevKitC-1 motor/encoder slave

  Matches robot-m-schematic.pdf REV A / 06 OCT 2026.

  Raspberry Pi responsibilities:
    - navigation, route, sensors, camera, distance and duration calculation
    - sends complete movement commands over UART

  ESP32-S3 responsibilities:
    - validate and parse Pi UART commands
    - turn Pi direction/speed into L298N inputs
    - run the paired left/right motors for the supplied duration
    - count all eight encoder channels
    - return encoder counts/status to the Pi

  NEW DRIVER MAP: two L298N modules, not TB6612FNG.
  The schematic ties the two left ENA/ENB inputs together and the two right
  ENA/ENB inputs together. Therefore each side has one PWM speed and one
  direction pair. Front/rear motors on the same side receive the same command.

  Schematic motor-control GPIOs:
    GPIO4  -> left L298N ENA + ENB, L_PWM
    GPIO5  -> left L298N IN1 + IN3, L_DIR1
    GPIO6  -> left L298N IN2 + IN4, L_DIR2
    GPIO7  -> right L298N ENA + ENB, R_PWM
    GPIO8  -> right L298N IN1 + IN3, R_DIR1
    GPIO9  -> right L298N IN2 + IN4, R_DIR2

  Schematic encoder GPIOs:
    LF C1 GPIO10, LF C2 GPIO11
    LR C1 GPIO12, LR C2 GPIO13
    RF C1 GPIO14, RF C2 GPIO15
    RR C1 GPIO16, RR C2 GPIO17

  UART WARNING:
    The attached schematic does not show a Pi UART connection or assign UART
    pins. Set UART_RX_PIN/UART_TX_PIN to the actual wires chosen by the team.
    The defaults below are only placeholders and must be confirmed physically.

  Suggested UART1 placeholder (NOT confirmed by schematic):
    ESP32-S3 GPIO18 RX <- Pi TX
    ESP32-S3 GPIO21 TX -> Pi RX through a 3.3 V-safe connection
    ESP32-S3 GND       <- Pi GND

  Protocol from the Pi:
    <P,PING,checksum>
    <S,sequence,checksum>
    <V,sequence,F|B|L|R|S,speed,duration_ms,distance_mm,checksum>

  Reply protocol:
    <A,sequence,PONG|START|STOP,checksum>
    <C,sequence,DONE,checksum>
    <E,sequence,lf_c1,lf_c2,lr_c1,lr_c2,rf_c1,rf_c2,rr_c1,rr_c2,checksum>
    <F,sequence,reason,checksum>

  Encoder supply warning:
    The schematic calls for +3V3_ENC. Confirm encoder outputs are 3.3 V safe.
    For 5 V push-pull encoder outputs, use level translators. Do not connect
    5 V directly to ESP32-S3 GPIOs.
*/

#include <Arduino.h>

// ------------------------ UART: confirm before wiring ----------------------
HardwareSerial PiSerial(1);
const int UART_RX_PIN = 18;  // PLACEHOLDER: actual Pi->ESP32 RX wire required
const int UART_TX_PIN = 21;  // PLACEHOLDER: actual ESP32->Pi TX wire required
const long UART_BAUD = 115200;

// ---------------------------- L298N controls -------------------------------
const int LEFT_PWM  = 4;
const int LEFT_DIR1 = 5;
const int LEFT_DIR2 = 6;
const int RIGHT_PWM  = 7;
const int RIGHT_DIR1 = 8;
const int RIGHT_DIR2 = 9;

// --------------------------- encoder inputs --------------------------------
// C1/C2 are kept separate and reported separately.
const int ENC_LF_C1 = 10;
const int ENC_LF_C2 = 11;
const int ENC_LR_C1 = 12;
const int ENC_LR_C2 = 13;
const int ENC_RF_C1 = 14;
const int ENC_RF_C2 = 15;
const int ENC_RR_C1 = 16;
const int ENC_RR_C2 = 17;

const int ENCODER_PIN[8] = {
  ENC_LF_C1, ENC_LF_C2, ENC_LR_C1, ENC_LR_C2,
  ENC_RF_C1, ENC_RF_C2, ENC_RR_C1, ENC_RR_C2
};

volatile long encoderCount[8] = {0,0,0,0,0,0,0,0};
volatile uint8_t encoderLast[8] = {0,0,0,0,0,0,0,0};

// Set a channel true if that encoder's direction is opposite to the others.
const bool ENCODER_INVERTED[8] = {false,false,false,false,false,false,false,false};

const unsigned long MAX_COMMAND_MS = 30000UL;
const unsigned long ENCODER_REPORT_MS = 100;
unsigned long lastEncoderReport = 0;
unsigned long stopAt = 0;
long activeSequence = 0;
bool commandActive = false;
String rxLine;

void IRAM_ATTR isrLF_C1() { uint8_t v=digitalRead(ENC_LF_C1); if(v!=encoderLast[0]){encoderCount[0]+=ENCODER_INVERTED[0]?-1:1;encoderLast[0]=v;} }
void IRAM_ATTR isrLF_C2() { uint8_t v=digitalRead(ENC_LF_C2); if(v!=encoderLast[1]){encoderCount[1]+=ENCODER_INVERTED[1]?-1:1;encoderLast[1]=v;} }
void IRAM_ATTR isrLR_C1() { uint8_t v=digitalRead(ENC_LR_C1); if(v!=encoderLast[2]){encoderCount[2]+=ENCODER_INVERTED[2]?-1:1;encoderLast[2]=v;} }
void IRAM_ATTR isrLR_C2() { uint8_t v=digitalRead(ENC_LR_C2); if(v!=encoderLast[3]){encoderCount[3]+=ENCODER_INVERTED[3]?-1:1;encoderLast[3]=v;} }
void IRAM_ATTR isrRF_C1() { uint8_t v=digitalRead(ENC_RF_C1); if(v!=encoderLast[4]){encoderCount[4]+=ENCODER_INVERTED[4]?-1:1;encoderLast[4]=v;} }
void IRAM_ATTR isrRF_C2() { uint8_t v=digitalRead(ENC_RF_C2); if(v!=encoderLast[5]){encoderCount[5]+=ENCODER_INVERTED[5]?-1:1;encoderLast[5]=v;} }
void IRAM_ATTR isrRR_C1() { uint8_t v=digitalRead(ENC_RR_C1); if(v!=encoderLast[6]){encoderCount[6]+=ENCODER_INVERTED[6]?-1:1;encoderLast[6]=v;} }
void IRAM_ATTR isrRR_C2() { uint8_t v=digitalRead(ENC_RR_C2); if(v!=encoderLast[7]){encoderCount[7]+=ENCODER_INVERTED[7]?-1:1;encoderLast[7]=v;} }

uint8_t xorChecksum(const String &body) {
  uint8_t result=0;
  for(size_t i=0;i<body.length();++i) result^=(uint8_t)body[i];
  return result;
}

String checksumText(const String &body) {
  char out[3];
  snprintf(out,sizeof(out),"%02X",xorChecksum(body));
  return String(out);
}

void sendFrame(const String &body) {
  PiSerial.print('<'); PiSerial.print(body); PiSerial.print(',');
  PiSerial.print(checksumText(body)); PiSerial.print(">\n");
}

bool decodeFrame(const String &line, String &body) {
  if(line.length()<6 || line[0]!='<' || line[line.length()-1]!='>') return false;
  String inside=line.substring(1,line.length()-1);
  int comma=inside.lastIndexOf(',');
  if(comma<1) return false;
  body=inside.substring(0,comma);
  String supplied=inside.substring(comma+1); supplied.toUpperCase();
  return supplied==checksumText(body);
}

int splitFields(const String &body, String fields[], int limit) {
  int count=0,start=0;
  while(count<limit) {
    int comma=body.indexOf(',',start);
    if(comma<0){fields[count++]=body.substring(start);break;}
    fields[count++]=body.substring(start,comma); start=comma+1;
  }
  return count;
}

void setLeft(float command) {
  command=constrain(command,-1.0f,1.0f);
  int duty=(int)(fabs(command)*255.0f);
  if(command>0.02f){digitalWrite(LEFT_DIR1,HIGH);digitalWrite(LEFT_DIR2,LOW);}
  else if(command<-0.02f){digitalWrite(LEFT_DIR1,LOW);digitalWrite(LEFT_DIR2,HIGH);}
  else {digitalWrite(LEFT_DIR1,LOW);digitalWrite(LEFT_DIR2,LOW);duty=0;}
  analogWrite(LEFT_PWM,duty);
}

void setRight(float command) {
  command=constrain(command,-1.0f,1.0f);
  int duty=(int)(fabs(command)*255.0f);
  if(command>0.02f){digitalWrite(RIGHT_DIR1,HIGH);digitalWrite(RIGHT_DIR2,LOW);}
  else if(command<-0.02f){digitalWrite(RIGHT_DIR1,LOW);digitalWrite(RIGHT_DIR2,HIGH);}
  else {digitalWrite(RIGHT_DIR1,LOW);digitalWrite(RIGHT_DIR2,LOW);duty=0;}
  analogWrite(RIGHT_PWM,duty);
}

void stopMotors() {
  setLeft(0); setRight(0); commandActive=false;
}

void executeCommand(long sequence,char direction,float speed,unsigned long durationMs,long distanceMm) {
  if(speed<0.0f || speed>1.0f || durationMs>MAX_COMMAND_MS){sendFrame(String("F,")+sequence+",BAD_VALUE");return;}
  activeSequence=sequence;
  if(direction=='S' || speed==0.0f || durationMs==0){stopMotors();sendFrame(String("A,")+sequence+",STOP");return;}
  float left=0,right=0;
  if(direction=='F'){left=speed;right=speed;}
  else if(direction=='B'){left=-speed;right=-speed;}
  else if(direction=='L'){left=-speed;right=speed;}
  else if(direction=='R'){left=speed;right=-speed;}
  else {sendFrame(String("F,")+sequence+",BAD_DIRECTION");return;}
  setLeft(left); setRight(right);
  stopAt=millis()+durationMs; commandActive=true;
  (void)distanceMm; // distance was planned by Pi
  sendFrame(String("A,")+sequence+",START");
}

void processBody(const String &body) {
  String f[8]; int n=splitFields(body,f,8); if(n<1)return;
  if(f[0]=="P" && n==2 && f[1]=="PING"){sendFrame("A,0,PONG");return;}
  if(f[0]=="S"){long s=n>1?f[1].toInt():0;stopMotors();sendFrame(String("A,")+s+",STOP");return;}
  if(f[0]=="V" && n==6){
    long s=f[1].toInt(); char d=f[2].length()?f[2][0]:'?';
    executeCommand(s,d,f[3].toFloat(),(unsigned long)f[4].toInt(),f[5].toInt()); return;
  }
  sendFrame("F,0,BAD_FRAME");
}

void readUart() {
  while(PiSerial.available()) {
    char c=(char)PiSerial.read();
    if(c=='\n'){String body;if(decodeFrame(rxLine,body))processBody(body);else if(rxLine.length())sendFrame("F,0,CHECKSUM");rxLine="";}
    else if(c!='\r'){if(rxLine.length()<220)rxLine+=c;else rxLine="";}
  }
}

void sendEncoderReport() {
  long c[8]; noInterrupts(); for(int i=0;i<8;++i)c[i]=encoderCount[i]; interrupts();
  static long seq=0;
  sendFrame(String("E,")+(++seq)+","+c[0]+","+c[1]+","+c[2]+","+c[3]+","+c[4]+","+c[5]+","+c[6]+","+c[7]);
}

void setup() {
  Serial.begin(115200); // USB debug
  PiSerial.begin(UART_BAUD,SERIAL_8N1,UART_RX_PIN,UART_TX_PIN);

  pinMode(LEFT_PWM,OUTPUT); pinMode(LEFT_DIR1,OUTPUT); pinMode(LEFT_DIR2,OUTPUT);
  pinMode(RIGHT_PWM,OUTPUT); pinMode(RIGHT_DIR1,OUTPUT); pinMode(RIGHT_DIR2,OUTPUT);
  setLeft(0); setRight(0);

  for(int i=0;i<8;++i){pinMode(ENCODER_PIN[i],INPUT);encoderLast[i]=digitalRead(ENCODER_PIN[i]);}
  attachInterrupt(digitalPinToInterrupt(ENC_LF_C1),isrLF_C1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LF_C2),isrLF_C2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LR_C1),isrLR_C1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LR_C2),isrLR_C2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RF_C1),isrRF_C1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RF_C2),isrRF_C2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_C1),isrRR_C1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_C2),isrRR_C2,CHANGE);
  Serial.println("ESP32-S3 L298N executor ready; motors stopped.");
}

void loop() {
  readUart();
  if(commandActive && (long)(millis()-stopAt)>=0){stopMotors();sendFrame(String("C,")+activeSequence+",DONE");}
  if(millis()-lastEncoderReport>=ENCODER_REPORT_MS){lastEncoderReport=millis();sendEncoderReport();}
}
