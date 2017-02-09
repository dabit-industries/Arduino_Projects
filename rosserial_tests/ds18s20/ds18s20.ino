#include <ros.h>
#include <std_msgs/Float32.h>
#include "OneWire.h"
#include "DallasTemperature.h"

boolean no_ds18s20 = false;

// one-wire stuff
OneWire ds18s20_onewire(23);
DallasTemperature ds18s20(&ds18s20_onewire);
DeviceAddress insideThermometer = { 0x28, 0xB3, 0x89, 0x5F, 0x06, 0x00, 0x00, 0xEB };
DeviceAddress outsideThermometer = { 0x28, 0x5B, 0x99, 0x5F, 0x06, 0x00, 0x00, 0x1F };

// ros stuff
ros::NodeHandle nh;
std_msgs::Float32 insideTemp;
std_msgs::Float32 outsideTemp;
ros::Publisher pub_insideTemp("/sensors/ds18s20_inside", &insideTemp);
ros::Publisher pub_outsideTemp("/sensors/ds18s20_outside", &outsideTemp);

void setup(void)
{
  nh.initNode();
  nh.advertise(pub_insideTemp);
  nh.advertise(pub_outsideTemp);
  if (!ds18s20.begin()) { no_ds18s20 = true; }
  ds18s20.setResolution(insideThermometer, 10);
  ds18s20.setResolution(outsideThermometer, 10);
}

void loop(void)
{ 
  if (no_ds18s20){
    insideTemp.data = 9001;
    outsideTemp.data = 9001;
  } else {
    ds18s20.requestTemperatures();
    insideTemp.data = ds18s20.getTempC(insideThermometer);
    outsideTemp.data = ds18s20.getTempC(outsideThermometer);
    if (insideTemp.data == -127.0) { insideTemp.data = 9001; }
    if (outsideTemp.data == -127.0) { outsideTemp.data = 9001; }
  }
  pub_insideTemp.publish(&insideTemp);
  pub_outsideTemp.publish(&outsideTemp);
  nh.spinOnce();
  delay(1000);
}

