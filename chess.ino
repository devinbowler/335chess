#include <Adafruit_PWMServoDriver.h> // Include the PWM Driver library

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

// Define adjusted minimum and maximum pulse lengths for the motors
int firstArmMin = 200; // Adjusted minimum for motor 0 to skip dead zone
int firstArmMax = 520; // Adjusted maximum for motor 0 (80% range)

int secondArmMin = 100; // Adjusted minimum for motor 1 to skip dead zone
int secondArmMax = 450; // Adjusted maximum for motor 1 (70% range)

int pullyMotorMin = 0; // Adjusted minimum for motor 2
int pullyMotorMax = 350; // Adjusted maximum for motor 2 (restricted to safe range)

// Motor channels
int firstArm = 0; // Motor 0 on PCA9685 Channel 0
int secondArm = 1; // Motor 1 on PCA9685 Channel 1
int pullyMotor = 2; // Motor 2 on PCA9685 Channel 2

// Servo frequency
int servoFrequency = 50; // 50 Hz for analog servos

// Emergency stop button
const int stopButtonPin = 2; // Pin for the emergency stop button
bool emergencyStop = false;  // Variable to track stop state



void setup() {
  // Start Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("Initializing motors...");

  // Initialize PWM driver
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000); // Set the oscillator frequency
  pwm1.setPWMFreq(servoFrequency);       // Set servo frequency to 50 Hz

  // Initialize the stop button pin
  pinMode(stopButtonPin, INPUT_PULLUP); // Use internal pull-up resistor

  // Move all motors to safe positions
  Serial.println("Moving motors to neutral positions...");
  moveToNeutral(firstArm, firstArmMin, firstArmMax);
  moveToNeutral(secondArm, secondArmMin, secondArmMax);
  moveToNeutral(pullyMotor, pullyMotorMin, pullyMotorMax);

  Serial.println("Motors initialized to neutral positions. Press button to stop.");
  delay(2000); // Brief pause before starting the test
}

void loop() {
  // Continuously check the stop button
  if (digitalRead(stopButtonPin) == LOW) {
    emergencyStop = true; // Activate emergency stop
    Serial.println("Emergency stop activated!");
    stopMotors();         // Stop all motors immediately
    while (true);         // Freeze the program until reset
  }

  if (!emergencyStop) {
    // Sweep motors 0, 1, and 2
    Serial.println("Testing First Arm (Channel 0)");
    testMotor(firstArm, firstArmMin, firstArmMax);

    Serial.println("Testing Second Arm (Channel 1)");
    testMotor(secondArm, secondArmMin, secondArmMax);

    Serial.println("Testing Pully Motor (Channel 2)");
    testMotor(pullyMotor, pullyMotorMin, pullyMotorMax);

    delay(3000); // Pause before repeating the test
  }
}

// Function to move a motor to its neutral position gradually
void moveToNeutral(int motorChannel, int minPulse, int maxPulse) {
  int neutralPulse = (minPulse + maxPulse) / 2; // Calculate the neutral position
  pwm1.setPWM(motorChannel, 0, neutralPulse);  // Move motor directly to neutral
  Serial.print("Motor ");
  Serial.print(motorChannel);
  Serial.print(" set to neutral pulse: ");
  Serial.println(neutralPulse);
  delay(500); // Brief delay for stabilization
}

// Function to stop all motors
void stopMotors() {
  pwm1.setPWM(firstArm, 0, 0); // Stop Motor 0
  pwm1.setPWM(secondArm, 0, 0); // Stop Motor 1
  pwm1.setPWM(pullyMotor, 0, 0); // Stop Motor 2
  Serial.println("All motors stopped.");
}

// Function to sweep the motor slowly through its range
void testMotor(int motorChannel, int minPulse, int maxPulse) {
  Serial.print("Starting slow sweep on motor channel ");
  Serial.println(motorChannel);

  // Slowly move from minimum to maximum
  for (int pulse = minPulse; pulse <= maxPulse; pulse += 2) {
    if (digitalRead(stopButtonPin) == LOW) { // Check stop button during the sweep
      emergencyStop = true;
      Serial.println("Emergency stop activated mid-test!");
      stopMotors();
      while (true);
    }

    // Validate pulse range
    if (pulse < minPulse || pulse > maxPulse) {
      Serial.print("Invalid pulse detected: ");
      Serial.println(pulse);
      break; // Exit sweep to prevent over-driving
    }

    pwm1.setPWM(motorChannel, 0, pulse);
    Serial.print("Pulse: ");
    Serial.println(pulse);
    delay(50); // Slow movement
  }
  delay(1000); // Pause at max position

  // Slowly move from maximum to minimum
  for (int pulse = maxPulse; pulse >= minPulse; pulse -= 2) {
    if (digitalRead(stopButtonPin) == LOW) { // Check stop button during the sweep
      emergencyStop = true;
      Serial.println("Emergency stop activated mid-test!");
      stopMotors();
      while (true);
    }

    // Validate pulse range
    if (pulse < minPulse || pulse > maxPulse) {
      Serial.print("Invalid pulse detected: ");
      Serial.println(pulse);
      break; // Exit sweep to prevent over-driving
    }

    pwm1.setPWM(motorChannel, 0, pulse);
    Serial.print("Pulse: ");
    Serial.println(pulse);
    delay(50); // Slow movement
  }
  delay(1000); // Pause at min position
}
