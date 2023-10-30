#ifndef PID_CONTROL
#define PID_CONTROL

#define INTEGRAL_CAP 5         // 4 times the max output
#define INTEGRAL_ARRAY_SIZE 20 // How big the memory of the integral term

class PID_Control {

public:
  PID_Control(float p, float i, int min, int max, int adjust = 0);
  float compute(float goal, float currentVal);

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
};

#endif // PID_CONTROL