/** @file stm5045.hpp */

#ifndef STM5045_H
#define STM5045_H

#include <chrono>
#include <iostream>

#define PI 3.141592

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** A little class to make dealing with wiringPi pins a bit nicer. */
class outputPin {
private:
  unsigned int m_pinNumber;
  unsigned int m_pinState;

public:
  /** (constructor)
   *
   * This function calls pinMode() so you _must_ call wiringPiSetup() 
   * before this constructor.
   *
   * @param pinNumber the wiringPi pin number the object should control
   * @param initialState the desired initial state of the pin (HIGH or LOW)
   */
  outputPin(unsigned int pinNumber, unsigned int initialState) {
    m_pinNumber = pinNumber;
    m_pinState = initialState;
    pinMode(pinNumber, OUTPUT);
    digitalWrite(m_pinNumber, m_pinState);
  }

  /** Set the pin state
   *
   * @param state the desired pin state (HIGH or LOW)
   */
  void set(unsigned int state) {
    m_pinState = state;
    digitalWrite(m_pinNumber, m_pinState);
  }

  /** Toggle the pin
   *
   * If the pin is HIGH go LOW, and vice versa.
   */
  void toggle() {
    if (m_pinState != 0) {
      m_pinState = 0;
    }
    else {
      m_pinState = 1;
    }

    digitalWrite(m_pinNumber, m_pinState);
  }
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class stm5045 {
private:
  outputPin m_pulse, m_direction, m_enable;
  
  std::chrono::time_point<std::chrono::steady_clock> m_prevTime;
  std::chrono::duration<double> m_pulseSpacing, m_pulseTime;

  unsigned int m_pulsesPerRevolution;

  float m_angularVelocity;
  float m_targetAngularVelocity;
  float m_angularAcceleration;

  void updatePulseSpacing() {
    m_pulseSpacing = (PI/(m_angularVelocity * m_pulsesPerRevolution)) * std::chrono::seconds(1);
  }

public:
  bool moving, accelerating;
  
  stm5045(unsigned int pulsePin,
          unsigned int directionPin,
          unsigned int enablePin,
          unsigned int pulsesPerRevolution,
          float angularVelocity)
    : m_pulse(pulsePin, 0),
      m_direction(directionPin, 0),
      m_enable(enablePin, 1),
      m_pulseTime(0)
  {
    m_pulsesPerRevolution = pulsesPerRevolution;
    m_angularVelocity = angularVelocity;
    updatePulseSpacing();
    m_prevTime = std::chrono::steady_clock::now();
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  void setVelocity(float angularVelocity) {
    m_angularVelocity = angularVelocity;
    updatePulseSpacing();
  }

  void setTargetVelocity(float targetAngularVelocity) {
    m_targetAngularVelocity = targetAngularVelocity;
  }

  void setAcceleration(float angularAcceleration) {
    m_angularAcceleration = angularAcceleration;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  void step() {
    m_pulse.toggle();
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  void update() {
    std::chrono::time_point<std::chrono::steady_clock> currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = currentTime - m_prevTime;
    
    if (moving) {
      m_pulseTime += dt;

      if (m_pulseTime > m_pulseSpacing) {
        m_pulseTime -= m_pulseSpacing;
        m_pulse.toggle();
      }
      if (accelerating) {
        float velocityStep = m_angularAcceleration * dt.count();
        if (m_angularVelocity < m_targetAngularVelocity) {
          m_angularVelocity += velocityStep;
        }
        else {
          m_angularVelocity -= velocityStep;
        }
        
        if (abs(m_angularVelocity - m_targetAngularVelocity) < velocityStep) {
          m_angularVelocity = m_targetAngularVelocity;
          accelerating = false;
        }
        updatePulseSpacing();
      }
    }

    m_prevTime = currentTime;
  }
};
  
#endif
