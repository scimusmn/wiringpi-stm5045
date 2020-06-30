#include <wiringPi.h>
#include "stm5045.hpp"

const unsigned int pulse = 3;
const unsigned int direction = 2;
const unsigned int enable = 0;

int main() {
  wiringPiSetup();

  stm5045 motor(pulse, direction, enable, 6400, 0);
  motor.setTargetVelocity(-2*PI);
  motor.setAcceleration(1);
  motor.moving = true;
  motor.accelerating = true;
  
  for(;;) {
    motor.update();
  }

  return 0;
}
