#ifndef PID
#define PID

#include <Arduino.h>
#define INTEGRAL_CAP 5  //4 times the max output
#define INTEGRAL_ARRAY_SIZE 20 //How big the memory of the integral term

class PID_Control
{
private:
    float integral = 0;
    float integralArray[INTEGRAL_ARRAY_SIZE];
    int integralIndex = 0;
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
        // Initialize the integral array with zeros
        for (int i = 0; i < INTEGRAL_ARRAY_SIZE; i++) {
            integralArray[i] = 0;
        }
    }

    float compute(float goal, float currentVal){
        error = goal - currentVal;
        Serial.print("Error: ");
        Serial.println(error);

        // Update the integral array, replacing the oldest value
        integralArray[integralIndex] = error;
        integralIndex = (integralIndex + 1) % INTEGRAL_ARRAY_SIZE;

        //Have the integral be the sum of the error of the last 15 values
        float integral = 0;
        for (int i = 0; i < INTEGRAL_ARRAY_SIZE; i++) {
            integral += integralArray[i];
        }
        //Cap the integral term 
        if(integral > ((max_output - bias)*INTEGRAL_CAP)){
            integral = (max_output - bias)*INTEGRAL_CAP;
        }
        if(integral < ((min_output - bias)*INTEGRAL_CAP)){
            integral = (min_output - bias)*INTEGRAL_CAP;           //if the min output was 45 but the bias is 90 then the integral cap would be -45
        }
        Serial.print("Integral");
        Serial.println(integral);
        float pidVal = error*kp + integral*ki;
        Serial.print("PID value");
        Serial.println(pidVal);
        float output = bias + pidVal;
        
        //Clamp the output to not allow it to exceed the limit
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