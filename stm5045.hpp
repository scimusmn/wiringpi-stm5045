/** @file stm5045.hpp */

#ifndef STM5045_H
#define STM5045_H

#include <chrono>
#include <cmath>
#include <iostream>

#define PI 3.141592

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** @brief A little class to make dealing with wiringPi pins a bit nicer. 
*/
class outputPin {
private:
  unsigned int m_pinNumber;
  unsigned int m_pinState;

public:
  /** @brief (constructor)
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

  /** @brief Set the pin state
   *
   * @param state the desired pin state (HIGH or LOW)
   */
  void set(unsigned int state) {
    m_pinState = state;
    digitalWrite(m_pinNumber, m_pinState);
  }

  /** @brief Get current pin state
   *
   * @return Current pin state (HIGH or LOW)
   */
  int getState() {
    return m_pinState;
  }

  /** @brief Toggle the pin
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

/** @brief Class for operating an ST-M5045 driver
 */
class stm5045 {
private:
  outputPin m_pulse, m_direction, m_enable;
  
  std::chrono::time_point<std::chrono::steady_clock> m_prevTime;
  std::chrono::duration<double> m_pulseSpacing, m_pulseTime;

  unsigned int m_pulsesPerRevolution;

  float m_angle;
  int m_steps;
  float m_angularVelocity;
  float m_targetAngularVelocity;
  float m_angularAcceleration;

  void updatePulseSpacing() {
    m_pulseSpacing = (PI/(abs(m_angularVelocity) * m_pulsesPerRevolution)) * std::chrono::seconds(1);
  }

public:
  /** @brief Whether or not the motor is currently moving.
   *
   * Set to TRUE to move; set to FALSE to stop.
   */
  bool moving;

  /** @brief Whether or not the motor is currently accelerating.
   *
   * After setting move to TRUE, set this to TRUE as well to force acceleration to the target velocity.
   */
  bool accelerating;

  /** @brief (constructor)
   *
   * This constructor initializes outputPin objects, which requires you to call wiringPiSetup() before
   * calling this.
   *
   * @param pulsePin The wiringPi number of the pin connected to the PUL signal
   * @param directionPin The wiringPi number of the pin connected to the DIR signal
   * @param enablePin The wiringPi number of the pin connected to the ENA signal
   * @param pulsesPerRevolution the number of pulses per shaft revolution
   * @param angularVelocity The inital angular velocity of the motor
   */
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

  /** @brief Set the current steps
   *
   * This function is typically used for zeroing.
   *
   * @param steps What to set steps to.
   */
  void setSteps(int steps = 0) {
    m_steps = steps;
  }

  /** @brief Set the current angle
   *
   * This function is typically used for zeroing.
   *
   * @param angle What to set the current angle to
   */
  void setAngle(float angle = 0) {
    m_angle = angle;
  }

  /** @brief Set the motor angular velocity
   *
   * This function is immediate, and will not accelerate to the supplied angular velocity.
   * Positive angular velocity is counter-clockwise; negative is clockwise.
   *
   * @param angularVelocity The new angular velocity in radians per second
   */
  void setVelocity(float angularVelocity) {
    m_angularVelocity = angularVelocity;
    if (m_angularVelocity < 0) {
      m_direction.set(1);
    }
    else {
      m_direction.set(0);
    }
    updatePulseSpacing();
  }

  /** @brief Set the motor target angular velocity
   *
   * The target angular velocity is the speed (in radians per second) 
   * the motor will spin up or down if accelerating is set to TRUE.
   * Positive angular velocity is counter-clockwise; negative is clockwise.
   *
   * @param targetAngularVelocity the target angular velocity
   */
  void setTargetVelocity(float targetAngularVelocity) {
    m_targetAngularVelocity = targetAngularVelocity;
  }

  /** @brief Set the motor angular acceleration
   *
   * This is just the absolute value of the acceleration; if a positive or negative change is
   * required, the sign is flipped automatically.
   *
   * @param angularAcceleration the absolute value of the motor's acceleration
   */
  void setAcceleration(float angularAcceleration) {
    m_angularAcceleration = angularAcceleration;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /** @brief Advance one step */
  void step() {
    m_pulse.toggle();
    int sign = 1;
    if (m_direction.getState() != 0) {
      sign = -1;
    }

    m_steps += sign;
    m_angle += sign * 2*PI/m_pulsesPerRevolution;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /** @brief Get current steps
   *
   * @return Current steps away from zero (positive for net counter-clockwise movement,
   * negative for net clockwise movement).
   */
  float getSteps() {
    return m_steps;
  }

  /** @brief Get the current angle
   *
   * @return The current angle in radians.
   */
  float getAngle() {
    return m_angle;
  }

  /** @brief Get the current angular velocity
   *
   * @return The current angular velocity in radians per second
   */
  float getVelocity() {
    return m_angularVelocity;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /** @brief Update the motor
   *
   * This function should be called as often as possible. Introducing delays elsewhere in 
   * the program will cause the motor to stop. If you'd like to introduce delays elsewhere,
   * call this function continuously in another thread.
   */
  void update() {
    std::chrono::time_point<std::chrono::steady_clock> currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = currentTime - m_prevTime;

    if (moving) {
      m_pulseTime += dt;

      if (m_pulseTime > m_pulseSpacing) {
        m_pulseTime -= m_pulseSpacing;
        step();
      }
      if (accelerating) {
        float velocityStep = m_angularAcceleration * dt.count();
        if (m_angularVelocity < m_targetAngularVelocity) {
          setVelocity(m_angularVelocity + velocityStep);
        }
        else {
          setVelocity(m_angularVelocity - velocityStep);
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
