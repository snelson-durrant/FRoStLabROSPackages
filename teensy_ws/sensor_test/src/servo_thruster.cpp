#include <Arduino.h>
#include <Servo.h>

Servo my_servo1;
Servo my_servo2;
Servo my_servo3;
Servo thruster;

int servo_pin1 = 9;
int servo_pin2 = 10;
int servo_pin3 = 11;
int thruster_pin = 12;
int default_pos_servo = 90;
int default_pos_thruster = 1500;
int servo1_pos = 0;
int servo2_pos = 0;
int servo3_pos = 0;
int thruster_pos = 0;

void setup_servo() {

    pinMode(servo_pin1, OUTPUT);
	pinMode(servo_pin2, OUTPUT);
	pinMode(servo_pin3, OUTPUT);
	pinMode(thruster_pin, OUTPUT);

	my_servo1.attach(servo_pin1);
	my_servo2.attach(servo_pin2);
	my_servo3.attach(servo_pin3);
    thruster.attach(thruster_pin);

	my_servo1.write(default_pos_servo);
	my_servo2.write(default_pos_servo);
	my_servo3.write(default_pos_servo);
    thruster.writeMicroseconds(default_pos_thruster);
	delay(7000);


}

void loop_servo() {

    my_servo1.write(servo1_pos);
	my_servo2.write(servo2_pos);
	my_servo3.write(servo3_pos);
	//int thrusterValue = map(0, 100, 1500, 2000);
	thruster.writeMicroseconds(thruster_pos);



}