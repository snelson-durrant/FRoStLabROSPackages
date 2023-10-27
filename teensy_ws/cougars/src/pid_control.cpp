#include "pid_control.hpp"

PID_Control::PID_Control(float p, float i, int min, int max, int adjust = 0) {
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

float PID_Control::compute(float goal, float currentVal) {
  error = goal - currentVal;
  // Serial5.print("Error: ");
  // Serial5.println(error);

  // Update the integral array, replacing the oldest value
  integralArray[integralIndex] = error;
  integralIndex = (integralIndex + 1) % INTEGRAL_ARRAY_SIZE;

  // Have the integral be the sum of the error of the last 15 values
  float integral = 0;
  for (int i = 0; i < INTEGRAL_ARRAY_SIZE; i++) {
    integral += integralArray[i];
  }

  // Cap the integral term
  if (integral > ((max_output - bias) * INTEGRAL_CAP)) {
    integral = (max_output - bias) * INTEGRAL_CAP;
  }
  if (integral < ((min_output - bias) * INTEGRAL_CAP)) {
    integral = (min_output - bias) *
               INTEGRAL_CAP; // if the min output was 45 but the bias is 90
                             // then the integral cap would be -45
  }

  // Serial5.print("Integral");
  // Serial5.println(integral);
  float pidVal = error * kp + integral * ki;
  // Serial5.print("PID value");
  // Serial5.println(pidVal);
  float output = bias + pidVal;

  // Clamp the output to not allow it to exceed the limit
  if (output > max_output) {
    output = max_output;
  } else if (output < min_output) {
    output = min_output;
  }
  return output;
}