/**
 * Photogrammetry Rotation Rig
 * Version: 0.2
 *
 * In this project we are rotating a platform, waiting some time for the platform to settle, then
 * triggering a camera to take a photo, then repeating. This is primarly used for photogrammetry.
 */

// Settings
// These are the default settings.
// Eventually these settings will be configured by the user using physical switches
const int SETTING_IDLE_TIME = 3000;     // The time to wait in the idle state
const int SETTING_SETTLING_TIME = 3000; // The time to wait after the platform has moved
const int SETTING_SHUTTER_TIME = 2000;  // The time to wait after the camera has taken a photo
const int SETTING_PHOTOS_PER_REVOLUTION = 360; // How many photos to take per full revolution (360 degrees)

// These settings should not change
const int SETTING_BAUD_RATE = 9600; // The baud rate of the serial port

// -----------------------------------------------------------------------------------------------------

// Include the AccelStepper library:
// https://www.airspayce.com/mikem/arduino/AccelStepper/
#include <AccelStepper.h>

// Define the stepper motor connections
// This will be different depending on the stepper that you use.
#define PIN_DIR 9     // Direction pin
#define PIN_STEP 8    // Step pin
#define PIN_SHUTTER 7 // Shutter pin

#define SHUTTER_TAKE_PHOTO HIGH // Trigger the camera to take a photo
#define SHUTTER_NO_PHOTO LOW    // Trigger the camera to take a photo

const int MOTOR_STEPS = 1600; // Number of steps per revolution. We will need to look at the motor to find this magic number.
const int STEP_PER_PHOTO = MOTOR_STEPS / SETTING_PHOTOS_PER_REVOLUTION;
const int WIGGLE_TIMER = 10;         // ms
const int SHUTTER_TRIGGER_TIME = 20; // ms

// State Machine
// -------------
#define STATE_IDLE 0         // Doing nothing on start up.
#define STATE_START_MOVING 1 // Calulating the next position to move the platform to
#define STATE_MOVING 2       // Moving the platform to the next position
#define STATE_SETTILE_TIME 3 // Waiting for the platform to settle
#define STATE_SHUTTER 4      // Triggering the camera to take a photo
#define STATE_SHUTTER_TIME 5 // Waiting for the camera to take a photo (exposer time, recording to SD card, etc)

// Globals
// -------------
int g_stateMachine = STATE_IDLE;         // The current state of the state machine
unsigned long g_timer_last_state_change; // The last time we changed states
int g_photo_count = 0;                   // How many times have we triggered the camera
unsigned long g_desired_position;        // Where we want to move the stepper to

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

void setup()
{
  Serial.begin(SETTING_BAUD_RATE);
  delay(1000); // Wait for the serial port to start

  Serial.println("Photogrammetry Rotation Rig");
  Serial.println("Version: 0.2");
  Serial.println("https://github.com/funvill/");

  // Set the maximum speed and acceleration
  const int MAX_SPEED = 2000;
  stepper.setMaxSpeed(MAX_SPEED);     // Set maximum speed in steps per second
  stepper.setAcceleration(MAX_SPEED); // Set acceleration in steps per second^2

  // setSpeed
  // Sets the desired constant speed for use with runSpeed().
  // "speed" - The desired constant speed in steps per second. Positive is clockwise. Speeds of more than
  // 1000 steps per second are unreliable. Very slow speeds may be set (eg 0.00027777 for once per hour,
  // approximately. Speed accuracy depends on the Arduino crystal. Jitter depends on how frequently you call
  // the runSpeed() function. The speed will be limited by the current value of setMaxSpeed()
  stepper.setSpeed(MAX_SPEED); // Set the initial direction (CW or CCW)

  pinMode(PIN_SHUTTER, OUTPUT);
  digitalWrite(PIN_SHUTTER, SHUTTER_NO_PHOTO);

  // Set the initial position
  g_desired_position = 0;
  g_stateMachine = STATE_IDLE;
  g_timer_last_state_change = millis();

  Serial.print("IDLE_TIME: " + String(SETTING_IDLE_TIME) + " ms, ");
  Serial.print("SETTLING_TIME: " + String(SETTING_SETTLING_TIME) + "  ms, ");
  Serial.print("SHUTTER_TIME: " + String(SETTING_SHUTTER_TIME) + " ms, ");
  Serial.print("MAX_SPEED: " + String(MAX_SPEED) + ", ");
  Serial.print("MOTOR_STEPS: " + String(MOTOR_STEPS) + " steps, ");
  Serial.print("SETTING_PHOTOS_PER_REVOLUTION: " + String(SETTING_PHOTOS_PER_REVOLUTION) + " photos, ");
  Serial.print("");
}

void loop()
{
  switch (g_stateMachine)
  {
  default:
  case STATE_IDLE:
    State_Idle();
    break;
  case STATE_START_MOVING:
    State_StartMoving();
    break;
  case STATE_MOVING:
    State_Moving();
    break;
  case STATE_SETTILE_TIME:
    State_settile_time();
    break;
  case STATE_SHUTTER:
    State_shutter();
    break;
  case STATE_SHUTTER_TIME:
    State_shutter_time();
    break;
  }
}

void State_Idle()
{
  if (millis() - g_timer_last_state_change < SETTING_IDLE_TIME - WIGGLE_TIMER)
  {
    Serial.println(String(millis()) + " - State: IDLE. " + String(SETTING_IDLE_TIME) + "ms");
    delay(SETTING_IDLE_TIME); // Wait for the serial port to start
  }
  else
  {
    // Move to the next position
    g_stateMachine = STATE_START_MOVING;
    g_timer_last_state_change = millis();
  }
}

void State_StartMoving()
{
  // Calculate the next position to move the stepper to
  g_desired_position += STEP_PER_PHOTO;

  Serial.println(String(millis()) + " - State: START MOVING to position: " + String(g_desired_position) + ", photo_count: " + String(g_photo_count));
  stepper.runToNewPosition(g_desired_position); // Move the stepper to the desired position

  g_stateMachine = STATE_MOVING;
  g_timer_last_state_change = millis();
}

void State_Moving()
{
  // Get the current position of the settper
  // This is used to determine if the stepper has reached the desired position
  long current_position = stepper.currentPosition();

  // Check
  if (stepper.distanceToGo() > 0 || current_position != g_desired_position)
  {
    // We are still moving
    static long last_position = 0;
    if (current_position != last_position)
    {
      last_position = current_position;
      Serial.println(String(millis()) + " - State: MOVING. distanceToGo: " + String(stepper.distanceToGo()) + " current_position: " + String(current_position));
    }
    stepper.runSpeed();
  }
  else
  {
    // We have reached the desired position
    g_stateMachine = STATE_SETTILE_TIME;
    g_timer_last_state_change = millis();
  }
}

void State_settile_time()
{
  if (millis() - g_timer_last_state_change < SETTING_SETTLING_TIME - WIGGLE_TIMER)
  {
    Serial.println(String(millis()) + " - State: SETTILE_TIME. " + String(SETTING_SETTLING_TIME) + "ms");
    delay(SETTING_SETTLING_TIME);
  }
  else
  {
    // Move to the next position
    g_stateMachine = STATE_SHUTTER;
    g_timer_last_state_change = millis();
  }
}

void State_shutter()
{
  // Trigger the camera to take a photo
  Serial.println(String(millis()) + " - State: SHUTTER. photo_count: " + String(g_photo_count));
  g_photo_count++;

  // Trigger the camera to take a photo
  digitalWrite(PIN_SHUTTER, SHUTTER_TAKE_PHOTO);
  delay(SHUTTER_TRIGGER_TIME);
  digitalWrite(PIN_SHUTTER, SHUTTER_NO_PHOTO);

  // Move to the next position
  g_stateMachine = STATE_SHUTTER_TIME;
  g_timer_last_state_change = millis();
}

void State_shutter_time()
{
  if (millis() - g_timer_last_state_change < SETTING_SHUTTER_TIME - WIGGLE_TIMER)
  {
    Serial.println(String(millis()) + " - State: SHUTTER_TIME. " + String(SETTING_SHUTTER_TIME) + "ms");
    delay(SETTING_SHUTTER_TIME);
  }
  else
  {
    // Move to the next position
    g_stateMachine = STATE_START_MOVING;
    g_timer_last_state_change = millis();
  }
}