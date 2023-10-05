#ifndef PID
#define PID

#include <Arduino.h>
#define INTEGRAL_CAP 3  //3 times the max output

class PID_Control
{
private:
    float integral = 0;
    float error = 0;
    float kp = 0;
    float ki = 0;
    int min_output;
    int max_output;
    int bias;

public:
    
    void begin(float p, float i, int min, int max, int adjust = 0){
        kp = p;
        ki = i;
        min_output = min;
        max_output = max;
        bias = adjust;
        Serial.println(p);
        

    }

    float compute(float goal, float currentVal){
        error = goal - currentVal;
        Serial.print("Error: ");
        Serial.println(error);
        integral = integral + error;
        if(integral > ((max_output - bias)*INTEGRAL_CAP)){
            integral = max_output - bias;
        }
        if(integral < ((min_output - bias)*INTEGRAL_CAP)){
            integral = min_output - bias;           //if the min output was 45 but the bias is 90 then the integral cap would be -45
        }
        Serial.print("Integral");
        Serial.println(integral);
        float pidVal = error*kp + integral*ki;
        Serial.print("PID value");
        Serial.println(pidVal);
        float output = bias + pidVal;
        if(output > max_output){
            output = max_output;
        }
        else if(output < min_output){
            output = min_output;
        }
        return output;
    }
};



#endif