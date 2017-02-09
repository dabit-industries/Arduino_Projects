#include <ros.h>
#include <std_msgs/Float32.h>
#include "Adafruit_AM2315.h"

boolean no_am2315 = false;
Adafruit_AM2315 am2315;

ros::NodeHandle nh;
std_msgs::Float32 am2315_tempc;
std_msgs::Float32 am2315_humidity;
ros::Publisher pub_am2315_tempc("/sensors/am2315/tempc", &am2315_tempc);
ros::Publisher pub_am2315_humidity("/sensors/am2315/humidity", &am2315_humidity);

void setup() {
  nh.initNode();
  nh.advertise(pub_am2315_tempc);
  nh.advertise(pub_am2315_humidity);
  if (!am2315.begin()) {
     no_am2315 = true;
  }
}

void loop() {
  if (no_am2315) {
    am2315_tempc.data = 9001;
    am2315_humidity.data = 9001;
  } else {
    am2315_tempc.data = am2315.readTemperature();
    am2315_humidity.data = am2315.readHumidity();
  }
  pub_am2315_tempc.publish(&am2315_tempc);
  pub_am2315_humidity.publish(&am2315_humidity);
  nh.spinOnce();
  delay(1000);
}
