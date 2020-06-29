#include <wiringPi.h>
#include "stm5045.hpp"

const unsigned int pulse = 3;
const unsigned int direction = 2;
const unsigned int enable = 0;

int main() {
  wiringPiSetup();

  stm5045 motor(pulse, direction, enable, 6400, 2*PI);
  motor.setTargetVelocity(0);
  motor.setAcceleration(2*PI);
  motor.moving = true;
  motor.accelerating = true;
  
  for(;;) {
    motor.update();
  }

  return 0;
}
