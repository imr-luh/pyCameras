

#include <Wire.h>
#define SLAVE_ADDRESS 0x05
#include <TimerOne.h>

int receivedSignal;
byte data[10];

const int TriggerProjektorPin = 8;
const int InterruptPin = 3;
static unsigned long interrupt_time = millis();
static unsigned long loop_time = millis();
static unsigned long time_since = 0;

int Intervall = 10 ;
int ExposureTime = 900 ;
int Frames = 8;
int FirstDelay = 10;
int LED = 13;
int LED2 = 12;
bool light = false;
bool expo = false;
bool trigger = false;


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);


Wire.begin(SLAVE_ADDRESS);
Wire.onReceive(receiveEvent);
Wire.onRequest(sendData);


pinMode(LED, OUTPUT);
pinMode(LED2, OUTPUT);


pinMode(InterruptPin, INPUT);
pinMode(TriggerProjektorPin, OUTPUT);
attachInterrupt(digitalPinToInterrupt(InterruptPin), funcionInterrupcion, FALLING);

Serial.println("Bereit");


}

void loop() {
  loop_time = millis();
  time_since = loop_time-interrupt_time;
  //Serial.println(time_since);
  if (time_since < ExposureTime && expo == false && light == true)
  {
  Serial.println("LED LOW");
  digitalWrite(LED, LOW); // LED anschalten
  digitalWrite(LED2, LOW);
  light = false;
  }
  
  if (time_since > ExposureTime && light == false && trigger == true)
  {
  Serial.println("LED HIGH");
  digitalWrite(LED, HIGH); // LED anschalten
  digitalWrite(LED2, HIGH);
  light = true;
  expo = true;
  }

}
void receiveEvent(int howMany)
{
  while (Wire.available()){
  receivedSignal = Wire.read();
  Serial.println(receivedSignal);
  if (receivedSignal == 0)
{
  Serial.println("Trigger mode off");
  trigger = false;
    digitalWrite(LED, LOW); // LED anschalten
  digitalWrite(LED2, LOW);
}
else
{
  Serial.println("Trigger mode is on");
  trigger = true;
}

  
  }

}
void sendData() {
  int FrameTime = 1 ;
  Wire.write(FrameTime);
}

void funcionInterrupcion() {
 interrupt_time = millis();
 expo = false;
}
  
    
