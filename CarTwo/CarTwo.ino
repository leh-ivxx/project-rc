#include "BluetoothSerial.h"
#include <ESP32Servo.h> 


String device_name = "RC-7960";


BluetoothSerial SerialBT;
Servo servoCh1; // will generate 4 servo PWM signals
Servo servoCh2; // will generate 4 servo PWM signals
Servo servoCh3; // will generate 4 servo PWM signals

int chPin1 =  13; // ESP pins: 0 2 12 14
int chPin2 =  12; 
int chPin3 =  2; 
int chVal1 = 1500;
int chVal2 = 1500;
int chVal3;

int usMin = 700; // min pulse width
int usMax = 2300; // max pulse width


char cmd[56]; // stores the command chars received from RoboRemo
int cmdIndex;
unsigned long lastCmdTime = 1000;
unsigned long aliveSentTime = 0;

#ifdef BTS
const int L_PWM = 25;    // I/O channel setup ESP32 pin 
const int R_PWM = 26; 
const int LR_EN = 27;    // I/O pin for BST l_EN & R_EN (enable)
#endif


/*
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14;
int value=0;
// Motor 2
int motor2Pin1 = 33; 
int motor2Pin2 = 25; 
int enable2Pin = 32;
*/
// Setting PWM properties
const int freq = 1000;
const int res = 8;

int dutyCycle = 0;
int value=0;

boolean cmdStartsWith(const char *st) { // checks if cmd starts with st
  for(int i=0; ; i++) {
    if(st[i]==0) return true;
    if(cmd[i]==0) return false;
    if(cmd[i]!=st[i]) return false;;
  }
  return false;
}


void debug(String st) {
  SerialBT.print("debug " + st + "\n");
}

void exeCmd() { // executes the command from cmd
  lastCmdTime = millis();

  if( cmdStartsWith("mo") ) {
    value = (int)atof(cmd+3);
    dutyCycle = constrain(value, 0, 255);
    if (value == 0) {
    // dutyCycle=0;
      handleStop();
   //   digitalWrite(LR_EN, LOW); 
     // digitalWrite(motor1Pin2, LOW); 
 //     digitalWrite(motor2Pin1, LOW);
  //    digitalWrite(motor2Pin2, LOW);   
    }
    
   }
if( cmdStartsWith("s 1") ) {
  //if(cmd[0]=='s' && cmd[1]==' '&& cmd[2]=='1') {
   handleStop();}
if( cmdStartsWith("r 1") ) {
   handleReverse(dutyCycle);}
if( cmdStartsWith("g 1") ) {
   handleForward(dutyCycle);}

if( cmdStartsWith("ch") ) {
      chVal1 = (int)atof(cmd+3);
      chVal2 = chVal1;
    /*  if(!servoCh1.attached()) {
        servoCh1.attach(chPin1, usMin, usMax);
      }   
          if(!servoCh2.attached()) {
        servoCh2.attach(chPin2, usMin, usMax);
      }*/
      servoCh1.writeMicroseconds(chVal1);
      servoCh2.writeMicroseconds(chVal2);
    }
    if( cmdStartsWith("ci") ) {
     int chVal3 = (int)atof(cmd+3);
      //if(!servoCh3.attached()) {
      //  servoCh3.attach(chPin3, usMin, usMax);
    //  }   
      servoCh3.writeMicroseconds(chVal3);
    }
  
  // invert channel:
  // example: set RoboRemo slider id to "ci0", set min -2000 and set max -1000
  
  
  
}
void handleForward(int duty1) {
//  Serial.println("Forward");
ledcWrite(R_PWM, 0);
ledcWrite(L_PWM, duty1);
//  digitalWrite(motor1Pin1, LOW);
//  digitalWrite(motor1Pin2, HIGH); 
digitalWrite(LR_EN, HIGH);
///  digitalWrite(motor2Pin2, HIGH);
}
void handleStop() {
 // Serial.println("Stop");
ledcWrite(R_PWM, 0);
ledcWrite(L_PWM, 0);
digitalWrite(LR_EN, LOW); 
 // digitalWrite(motor1Pin2, LOW); 
//  digitalWrite(motor2Pin1, LOW);
//  digitalWrite(motor2Pin2, LOW);   
 
}

void handleReverse(int duty2) {
  //Serial.println("Reverse");
ledcWrite(R_PWM, duty2);
ledcWrite(L_PWM, 0);
digitalWrite(LR_EN, HIGH);  
/*  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);     */     
 // server.send(200);
}

void setupBTS(){
 ledcAttach(R_PWM, freq, res);
 ledcAttach(L_PWM, freq, res);
  // Initialize PWM with 0 duty cycle
 ledcWrite(R_PWM, 0);
 ledcWrite(L_PWM, 0);
 pinMode(LR_EN,OUTPUT);
 digitalWrite(LR_EN,LOW);
}

void setupServo(){
 //for(int i=0; i<chCount; i++) {
    // attach channels to pins
    servoCh1.attach(chPin1, usMin, usMax);
    servoCh2.attach(chPin2, usMin, usMax);
    servoCh3.attach(chPin3, usMin, usMax);
    // initial value = middle
    chVal1 = (usMin + usMax)/2;
   // chVal2 = (usMin + usMax)/2;
    servoCh1.writeMicroseconds( chVal1 );
    servoCh2.writeMicroseconds( chVal1 );
    servoCh3.writeMicroseconds( chVal1 );
    delay(200);
    servoCh1.writeMicroseconds( usMin );
    servoCh2.writeMicroseconds( usMin );
    servoCh3.writeMicroseconds( usMin );
    delay(200);
    servoCh1.writeMicroseconds( usMax );
    servoCh2.writeMicroseconds( usMax );
    servoCh3.writeMicroseconds( usMax );
    delay(200);
    servoCh1.writeMicroseconds( chVal1 );
    servoCh2.writeMicroseconds( chVal1 );
    servoCh3.writeMicroseconds( chVal1 );
    delay(200);
  //  servoCh3.writeMicroseconds( usMin );
}
void setup() {
  //delay(1000);
   Serial.begin(115200);
  SerialBT.begin(device_name);  //Bluetooth device name
 setupBTS();
 setupServo();
//  pinMode(R_PWM, OUTPUT);
//  pinMode(motor1Pin2, OUTPUT);
  //  pinMode(18, OUTPUT);
 // pinMode(19, OUTPUT);
  //pinMode(motor2Pin1, OUTPUT);
 // pinMode(motor2Pin2, OUTPUT);
  // Configure PWM Pins
 // Attach BST R_PWM

  
  cmdIndex = 0;
}


void loop() {

  // if contact lost for more than half second
  if(millis() - lastCmdTime > 800) {  
   //  digitalWrite(motor1Pin1, LOW); 
//  digitalWrite(motor1Pin2, LOW); 
 // digitalWrite(motor2Pin1, LOW);
    digitalWrite(LR_EN, LOW);   
    servoCh1.writeMicroseconds( (usMin + usMax)/2 );
      
    servoCh2.writeMicroseconds( (usMin + usMax)/2 );
  //  LR_EN
    }
  // here we have a connected client
  if(SerialBT.available()) {
    char c = (char)SerialBT.read(); // read char from client (RoboRemo app)
    if(c=='\n') { // if it is command ending
      cmd[cmdIndex] = 0;
      exeCmd();  // execute the command
      cmdIndex = 0; // reset the cmdIndex
    } else {      
      cmd[cmdIndex] = c; // add to the cmd buffer
      if(cmdIndex<55) cmdIndex++;
    }
  } 

  
}
