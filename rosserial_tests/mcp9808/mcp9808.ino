#include <ros.h>
#include <std_msgs/Float32.h>
#include "Adafruit_MCP9808.h"

boolean no_mcp9808 = false;
Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();

ros::NodeHandle nh;
std_msgs::Float32 mcp9808_readings;
ros::Publisher pub_mcp9808("/sensors/mcp9808/tempc", &mcp9808_readings);

void setup(){
  nh.initNode();
  nh.advertise(pub_mcp9808);
  if (!mcp9808.begin()) {
    no_mcp9808 = true;
  }
}

void loop(){
  //nh.spinOnce();
  if (no_mcp9808){
    mcp9808_readings.data = -127.0;
  } else {
    mcp9808_readings.data = mcp9808.readTempC();
  }
  pub_mcp9808.publish(&mcp9808_readings);
  nh.spinOnce();
  delay(1000);
  //to do stuff or not to do stuff
}
