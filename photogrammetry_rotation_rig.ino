/**
 * Photogrammetry Rotation Rig
 * Version: 0.1
 *
 * In this project we are rotating a platform, waiting some time for the platform to settle, then
 * triggering a camera to take a photo, then repeating. This is primarly used for photogrammetry.
 */

// Settings
// These are the default settings.
// Eventually these settings will be configured by the user using physical switches

const int PHOTOS_PER_ROTATION = 8; // How many stops to take a photo, in a compleat rotation.
const int TIMER_MS_SETTILE = 1000; // How long to wait after moving before going to the next step
const int TIMER_MS_SHUTTER = 250;  // How long to wait before moving after triggering the shutter

const bool ROTATE_CONTINUOUSLY = false;    // Should we keep rotating
const int ROTATE_CONTINUOUSLY_SPEED = 250; // Should we keep rotating

// -----------------------------------------------------------------------------------------------------S

// Const.
// These settings should not change
const int SETTING_BAUD_RATE = 9600; // The baud rate of the serial port

// Include the AccelStepper library:
// https://www.airspayce.com/mikem/arduino/AccelStepper/
#include <AccelStepper.h>

// Define the stepper motor connections
// This will be different depending on the stepper that you use.
#define DIR_PIN 9        // Direction pin
#define STEP_PIN 8       // Step pin
#define MOTOR_STEPS 1600 // Number of steps per revolution. We will need to look at the motor to find this magic number.

// Statemachine
#define STATE_IDLE 0                // Doing nothing
#define STATE_MOVING 1              // Moving the platform
#define STATE_SETTILE_TIME 2        // Waiting for the platform to settle
#define STATE_SHUTTER_TIME 3        // Waiting for the camera to take a photo
#define STATE_CONTINUOUS_ROTATION 4 // Rotating the platform continuously

// Globals
int g_stateMachine = STATE_MOVING;     // The current state of the state machine
unsigned long timer_last_state_change; // The last time we changed states
int g_photo_count = 0;                 // How many times have we triggered the camera

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup()
{
  Serial.begin(SETTING_BAUD_RATE);
  delay(1000); // Wait for the serial port to start

  Serial.println("Photogrammetry Rotation Rig");
  Serial.println("Version: 0.1");
  Serial.println("https://github.com/funvill/");

  // Set the maximum speed and acceleration
  const int MAX_SPEED = 2000.0;
  stepper.setMaxSpeed(MAX_SPEED);     // Set maximum speed in steps per second
  stepper.setAcceleration(MAX_SPEED); // Set acceleration in steps per second^2

  // setSpeed
  // Sets the desired constant speed for use with runSpeed().
  // "speed" - The desired constant speed in steps per second. Positive is clockwise. Speeds of more than
  // 1000 steps per second are unreliable. Very slow speeds may be set (eg 0.00027777 for once per hour,
  // approximately. Speed accuracy depends on the Arduino crystal. Jitter depends on how frequently you call
  // the runSpeed() function. The speed will be limited by the current value of setMaxSpeed()
  stepper.setSpeed(MAX_SPEED); // Set the initial direction (CW or CCW)

  // Set the initial position
  g_stateMachine = STATE_MOVING;
  timer_last_state_change = 0;

  // CHECK TO SEE IF WE ARE IN CONTINUOUS ROTATION MODE
  if (ROTATE_CONTINUOUSLY)
  {
    Serial.println("Continuous rotation mode enabled");
    g_stateMachine = STATE_CONTINUOUS_ROTATION;

    // Set the speed
    stepper.setSpeed(ROTATE_CONTINUOUSLY_SPEED); // Set the initial direction (CW or CCW)
  }
}

void MoveToNextPosition()
{
  g_stateMachine = STATE_MOVING;

  // Move to the next position
  const long stepsToMove = (MOTOR_STEPS / PHOTOS_PER_ROTATION);
  stepper.move((MOTOR_STEPS / PHOTOS_PER_ROTATION));

  Serial.println(String(millis()) + " - stepsToMove: " + String(stepsToMove));
}

void loop()
{
  if (g_stateMachine == STATE_CONTINUOUS_ROTATION)
  {
    stepper.runSpeed();
    return;
  }

  unsigned long current_time = millis();
  switch (g_stateMachine)
  {

  // If we are in the moving state.
  // Check to see how much distance we have left to go.
  // If it is 0 then we can move on to the Settle time state.
  case STATE_MOVING:

    stepper.run();
    Serial.println(String(millis()) + " - currentPosition: " + String(stepper.currentPosition()) + ". distanceToGo: " + String(stepper.distanceToGo()));

    if (stepper.distanceToGo() == 0)
    {
      g_stateMachine = STATE_SETTILE_TIME;
      Serial.println(String(millis()) + " - State change to STATE_SETTILE_TIME. TIMER_MS_SETTILE=" + String(TIMER_MS_SETTILE) + "ms");
      timer_last_state_change = millis();
      break;
    }
    break;

  case STATE_SETTILE_TIME:
    stepper.stop();
    if (current_time - timer_last_state_change > TIMER_MS_SETTILE)
    {
      g_stateMachine = STATE_SHUTTER_TIME;
      Serial.println(String(millis()) + " - State change to STATE_SHUTTER_TIME. TIMER_MS_SHUTTER=" + String(TIMER_MS_SHUTTER) + "ms");
      timer_last_state_change = millis();

      // Trigger the camera
      // ToDo:
      g_photo_count++;

      if(g_photo_count > PHOTOS_PER_ROTATION) {

        // We are done. Set the state to idle
        g_stateMachine = STATE_IDLE;
      }

      break;
    }
    break;

  case STATE_SHUTTER_TIME:
    stepper.stop();
    if (current_time - timer_last_state_change > TIMER_MS_SHUTTER)
    {
      Serial.println(String(millis()) + " - State change to STATE_MOVING. ");
      MoveToNextPosition();
      break;
    }
    break;
  case STATE_IDLE:
  {
    stepper.stop();
    // Do nothing
    delay(1000); // Wait for the serial port to start
    Serial.println(String(millis()) + " - idle");
      
    break;
  }
  }
}
