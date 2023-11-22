#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "math.h"

ros::NodeHandle nh;

void teleop(const geometry_msgs::Twist& msg) {
  
  float a = msg.linear.x;
  float c = msg.angular.z;

  if (a > 0 && c == 0) { //straight
    Serial.print("straight");
    analogWrite(9, 255);
    analogWrite(10, 255);
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
  }
  if (a < 0 && c == 0) { //backward
    Serial.print("back");
    analogWrite(9, 255);
    analogWrite(10, 255);
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
  }
  if(a == 0 && c == 0) { //stop
    Serial.print("stop");
    analogWrite(9, 0);
    analogWrite(10, 0);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
  }
  if (a == 0 && c > 0) { //right
    Serial.print("right");
    analogWrite(9, 255);
    analogWrite(10, 255);
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
  }
  if(a == 0 && c < 0) { //left
    Serial.print("left");
    analogWrite(9, 255);
    analogWrite(10, 255);
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
  }
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", teleop); 


void setup(){
pinMode(10, OUTPUT); //ENB
pinMode(8, OUTPUT); //IN 4
pinMode(7, OUTPUT); //IN 3 
pinMode(6, OUTPUT); //IN 2
pinMode(5, OUTPUT); //IN 1
pinMode(9, OUTPUT); //ENA 

Serial.begin(57600);
nh.initNode();
nh.subscribe(sub);
}

void loop() {
nh.spinOnce();
}
