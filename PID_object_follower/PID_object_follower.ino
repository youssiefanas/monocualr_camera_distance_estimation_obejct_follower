#include <ros.h>
#include <std_msgs/String.h>
#include<std_msgs/Float32.h>

#define in1 5
#define in2 6
#define in3 10
#define in4 11  //motor pins
#define enA 9
#define enB 3
#define Kp 6
#define Ki 0
#define Kd 00
#define desired_dist 80

ros::NodeHandle nh;

std_msgs::Float32 ditance_msg;
std_msgs::Float32 pid_msg;

void objectCallback(const std_msgs::String& msg)
{
  

}
void distanceCallback(const std_msgs::Float32& msg)
{
 ditance_msg.data = msg.data;
 if (ditance_msg.data  > desired_dist){
moveBackward();
//analogWrite(enA,73);
//analogWrite(enB, 60);
analogWrite(enA, pid(desired_dist, ditance_msg.data ));
analogWrite(enB, 0.85*pid(desired_dist, ditance_msg.data ));
}

 else if (ditance_msg.data  < desired_dist){

moveForward();
analogWrite(enA, pid(desired_dist, ditance_msg.data ));
analogWrite(enB, 0.85*pid(desired_dist, ditance_msg.data ));
//analogWrite(enA,73);
//analogWrite(enB, 60);

}
else if (ditance_msg.data  == desired_dist){
stop();
}
else{
  stop();
}
pid_msg.data = pid(desired_dist, ditance_msg.data);

 
}

ros::Subscriber<std_msgs::String> object_sub("/object_detected", &objectCallback);
ros::Publisher etreadings("/etreadings",&ditance_msg);
ros::Publisher pid_pub("/pid_val",&ditance_msg);

ros::Subscriber<std_msgs::Float32> distance_sub("/object_dist", &distanceCallback);


void setup() {
Serial.begin(57600);

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    
  nh.initNode();
  nh.advertise(etreadings);
 nh.subscribe(object_sub);
  nh.advertise(pid_pub);
  nh.subscribe(distance_sub);
}

void loop() {
  
etreadings.publish(&ditance_msg);
 
pid_pub.publish(&pid_msg);
nh.spinOnce();
delay(200);

}


void moveForward()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void moveBackward()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


float pid(float setpoint, float actual_position)
{
  float error = setpoint - actual_position;
    float previous_error = 0;
    float integral = 0;
    float derivative = 0;
    float dt = 0.1;
    float output = 0;
    float motor_val, contr_output;
    integral = integral + error*dt;
    derivative = (error - previous_error)/dt;
    previous_error = error;
    output = Kp*abs(error) + Ki*integral + Kd*derivative;
    contr_output = constrain(output,0,100);
         motor_val = map(output,0,100,0,75);
         motor_val = constrain(motor_val,0,75);

    return motor_val;
}
