#include "Arduino.h"
#include "digitalWriteFast.h"
#include "TimerOne.h"
#include "ros.h"
#include "std_msgs/Int16.h"

#define c_LeftEncoderInterruptA 0
#define c_LeftEncoderInterruptB 1
#define c_LeftEncoderPinA 2
#define c_LeftEncoderPinB 3
#define LeftEncoderIsReversed

// Measured by wheeling 1m and counting ticks in 1m
#define meter_conversion_rate 0.004352872 // meters/tick
#define mile_conversion_rate 0.009737098 // meters/tick

unsigned short dt = 0;
unsigned long reset_time = 0;

volatile bool _LeftEncoderASet;
volatile bool _LeftEncoderBSet;
volatile bool _LeftEncoderAPrev;
volatile bool _LeftEncoderBPrev;
volatile int _LeftEncoderTicks = 0;
volatile int ticks_temp = 0;

boolean publish_ready = false;

ros::NodeHandle nh;
std_msgs::Int16 ticks;
ros::Publisher pub_ticks("/wheel_decoder/ticks", &ticks);

void setup()
{
  pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_LeftEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_LeftEncoderInterruptA, HandleMotorInterrupt, CHANGE);
  attachInterrupt(c_LeftEncoderInterruptB, HandleMotorInterrupt, CHANGE);

  nh.initNode();
  nh.advertise(pub_ticks);
  
  Timer1.initialize(50000);
  Timer1.attachInterrupt(update_ros);
}

void update_ros(){
  publish_ready = true;
}

void loop()
{ 
  if (publish_ready){
    cli();
    ticks_temp = _LeftEncoderTicks;
    _LeftEncoderTicks = 0;
    sei();
    ticks.data = ticks_temp;
    publish_ready = false;
    pub_ticks.publish(&ticks);
    nh.spinOnce();
  }
}

void HandleMotorInterrupt(){
  cli();
  _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);
  _LeftEncoderASet = digitalReadFast(c_LeftEncoderPinA);
  if(_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && _LeftEncoderBSet) _LeftEncoderTicks+=1;
    if(_LeftEncoderASet && !_LeftEncoderBSet) _LeftEncoderTicks-=1;
  }else if(!_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && !_LeftEncoderBSet) _LeftEncoderTicks+=1;
    if(_LeftEncoderASet && _LeftEncoderBSet) _LeftEncoderTicks-=1;
  }else if(!_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && !_LeftEncoderBSet) _LeftEncoderTicks+=1;
    if(!_LeftEncoderASet && _LeftEncoderBSet) _LeftEncoderTicks-=1;
  }else if(_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && _LeftEncoderBSet) _LeftEncoderTicks+=1;
    if(!_LeftEncoderASet && !_LeftEncoderBSet) _LeftEncoderTicks-=1;
  }
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
  sei();
}
