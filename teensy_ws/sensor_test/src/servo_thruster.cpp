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

int count = 0;

void tick_sm() {

        if (count < 4) {
                my_servo1.write(90);
                my_servo2.write(60);
                my_servo3.write(120);
                thruster.writeMicroseconds(1600);
                count++;
        } else if (count < 11) {
                my_servo1.write(45);
                my_servo2.write(90);
                my_servo3.write(90);
                thruster.writeMicroseconds(1600);
                count++;
        } else {
                my_servo1.write(90);
                my_servo2.write(90);
                my_servo3.write(90);
                thruster.writeMicroseconds(1500);
        }
}

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

	tick_sm();
	delay(1000);
}

