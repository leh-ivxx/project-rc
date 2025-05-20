#include "BluetoothSerial.h"
#include <ESP32Servo.h> 


String device_name = "RcCar420";


BluetoothSerial SerialBT;

Servo servoCh; // will generate 4 servo PWM signals
int chPin =  2; // ESP pins: GPIO 0, 2, 14, 12
int chVal = 1500;
int usMin = 700; // min pulse width
int usMax = 2300; // max pulse width





char cmd[100]; // stores the command chars received from RoboRemo
int cmdIndex;
unsigned long lastCmdTime = 60000;
unsigned long aliveSentTime = 0;

int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14;
int value=0;
// Motor 2
int motor2Pin1 = 33; 
int motor2Pin2 = 25; 
int enable2Pin = 32;

// Setting PWM properties
const int freq = 30000;
const int resolution = 8;
int dutyCycle = 0;


boolean cmdStartsWith(const char *st) { // checks if cmd starts with st
  for(int i=0; ; i++) {
    if(st[i]==0) return true;
    if(cmd[i]==0) return false;
    if(cmd[i]!=st[i]) return false;;
  }
  return false;
}


void exeCmd() { // executes the command from cmd

  lastCmdTime = millis();

  // example: set RoboRemo slider id to "ch0", set min 1000 and set max 2000
   if( cmdStartsWith("mo") ) {
      value = (int)atof(cmd+3);
 if (value == 0) {
      ledcWrite(enable1Pin, 0);
      ledcWrite(enable2Pin, 0);
      digitalWrite(motor1Pin1, LOW); 
      digitalWrite(motor1Pin2, LOW); 
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, LOW);   
    } else { 
      dutyCycle = map(value, 25, 100, 200, 255);
      ledcWrite(enable1Pin, dutyCycle);
      ledcWrite(enable2Pin, dutyCycle);
      Serial.println("Motor speed set to " + String(value));
    }
   }
     if(cmd[0]=='s' && cmd[1]==' '&& cmd[2]=='1') {
   handleStop();}
  if(cmd[0]=='r' && cmd[1]==' ' && cmd[2]=='1') {
   handleReverse();}
if(cmd[0]=='g' && cmd[1]==' ' && cmd[2]=='1') {
   handleForward();}



  if( cmdStartsWith("ch") ) {
      chVal = (int)atof(cmd+3);
      if(!servoCh.attached()) {
        servoCh.attach(chPin, usMin, usMax);
      }   
      servoCh.writeMicroseconds(chVal);
    }
    if( cmdStartsWith("le") ) {
      chVal = (int)atof(cmd+3);
      if(!servoCh.attached()) {
        servoCh.attach(chPin, usMin, usMax);
      }   
      servoCh.writeMicroseconds(chVal);
    }
  
  // invert channel:
  // example: set RoboRemo slider id to "ci0", set min -2000 and set max -1000
  
  if( cmdStartsWith("ci") ) {
    
      chVal = -(int)atof(cmd+4);
      if(!servoCh.attached()) {
        servoCh.attach(chPin, usMin, usMax);
      }   
      servoCh.writeMicroseconds(chVal);
    }
  
  
  // use accelerometer:
  // example: set RoboRemo acc y id to "ca1"
  
  if( cmdStartsWith("ca") ) {
      chVal = (usMax+usMin)/2 + (int)( atof(cmd+3)*51 ); // 9.8*51 = 500 => 1000 .. 2000
      if(!servoCh.attached()) {
        servoCh.attach(chPin, usMin, usMax);
      }   
      servoCh.writeMicroseconds(chVal);
    }
  
  
  // invert accelerometer:
  // example: set RoboRemo acc y id to "cb1"
  
  if( cmdStartsWith("cb") ) {
    
      chVal = (usMax+usMin)/2 - (int)( atof(cmd+3)*51 ); // 9.8*51 = 500 => 1000 .. 2000
      if(!servoCh.attached()) {
        servoCh.attach(chPin, usMin, usMax);
      }   
      servoCh.writeMicroseconds(chVal);
    }
  
  
  
  
}
void handleForward() {
//  Serial.println("Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}
void handleStop() {
 // Serial.println("Stop");
  digitalWrite(motor1Pin1, LOW); 
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);   
 
}

void handleReverse() {
  //Serial.println("Reverse");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);          
 // server.send(200);
}
void setup() {
  delay(1000);
   // Serial.begin(115200);
  SerialBT.begin(device_name);  //Bluetooth device name

  //for(int i=0; i<chCount; i++) {
    // attach channels to pins
    servoCh.attach(chPin, usMin, usMax);
    // initial value = middle
    chVal = (usMin + usMax)/2;
    // update
    servoCh.writeMicroseconds( chVal );
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
    pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  // Configure PWM Pins
  ledcAttach(enable1Pin, freq, resolution);
  ledcAttach(enable2Pin, freq, resolution);
  // Initialize PWM with 0 duty cycle
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);
  cmdIndex = 0;
}


void loop() {

  // if contact lost for more than half second
  if(millis() - lastCmdTime > 500) {  
     digitalWrite(motor1Pin1, LOW); 
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);   
      servoCh.writeMicroseconds( (usMin + usMax)/2 );
      servoCh.detach(); // stop PWM signals
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
      if(cmdIndex<99) cmdIndex++;
    }
  } 
/*
  if(millis() - aliveSentTime > 500) { // every 500ms
    client.write("alive 1\n");
    // send the alibe signal, so the "connected" LED in RoboRemo will stay ON
    // (the LED must have the id set to "alive")
    
    aliveSentTime = millis();
    // if the connection is lost, the RoboRemo will not receive the alive signal anymore,
    // and the LED will turn off (because it has the "on timeout" set to 700 (ms) )
  }*/

}
