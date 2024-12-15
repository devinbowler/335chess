#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PWM driver with the default I2C address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define motor channels
const int firstArmChannel = 0;    // Channel 0 on PCA9685
const int secondArmChannel = 1;   // Channel 1 on PCA9685
const int pullyMotorChannel = 2;  // Channel 2 on PCA9685 (used for sweeping)

// Define adjusted minimum and maximum pulse lengths for the motors
const int firstArmMin = 200; // Adjusted minimum for motor 0 to skip dead zone
const int firstArmMax = 520; // Adjusted maximum for motor 0 (80% range)

const int secondArmMin = 100; // Adjusted minimum for motor 1 to skip dead zone
const int secondArmMax = 450; // Adjusted maximum for motor 1 (70% range)

const int pullyMotorMin = 0;   // Adjusted minimum for motor 2
const int pullyMotorMax = 750; // Adjusted maximum for motor 2 (restricted to safe range)

// Define the pin for the emergency stop button
const int stopButtonPin = 2; // Digital Pin D2

// Define LED pins for status indicators (optional)
const int greenLEDPin = 3; // Digital Pin D3
const int redLEDPin = 4;   // Digital Pin D4

// Flag to track the state of the emergency stop
volatile bool emergencyStop = false;

// Define a struct to hold motor positions
struct Position {
  int x;
  int y;
};

// Define a 7x7 chessboard grid with coordinate pairs
// Each Position corresponds to a square on the chessboard
// Row 0 is the Bottom Row, Column 0 is the Rightmost Column
const int GRID_SIZE = 7;
const Position chessBoard[GRID_SIZE][GRID_SIZE] = {
  // Row 0 (Bottom Row)
  { {307, 541}, {345, 520}, {383, 499}, {421, 478}, {459, 457}, {497, 436}, {535, 415} },
  
  // Row 1
  { {307, 480}, {345, 459}, {383, 438}, {421, 417}, {459, 396}, {497, 375}, {535, 354} },
  
  // Row 2
  { {307, 419}, {345, 398}, {383, 377}, {421, 356}, {459, 335}, {497, 314}, {535, 293} },
  
  // Row 3 (Middle Row)
  { {307, 358}, {345, 337}, {383, 316}, {421, 295}, {459, 274}, {497, 253}, {535, 232} },
  
  // Row 4
  { {307, 297}, {345, 276}, {383, 255}, {421, 234}, {459, 213}, {497, 192}, {535, 171} },
  
  // Row 5
  { {307, 236}, {345, 215}, {383, 194}, {421, 173}, {459, 152}, {497, 131}, {535, 110} },
  
  // Row 6 (Top Row)
  { {307, 175}, {345, 154}, {383, 133}, {421, 112}, {459, 91},  {497, 70},  {535, 49} }
};

// Function to move a motor to its neutral position
void moveToNeutral(int motorChannel, int minPulse, int maxPulse) {
  int neutralPulse = (minPulse + maxPulse) / 2; // Calculate the neutral position
  pwm.setPWM(motorChannel, 0, neutralPulse);  // Move motor directly to neutral
  Serial.print("Motor ");
  Serial.print(motorChannel);
  Serial.print(" set to neutral pulse: ");
  Serial.println(neutralPulse);
  delay(500); // Brief delay for stabilization
}

// Function to stop all motors immediately
void stopMotors() {
  pwm.setPWM(firstArmChannel, 0, 0);    // Stop firstArm
  pwm.setPWM(secondArmChannel, 0, 0);   // Stop secondArm
  pwm.setPWM(pullyMotorChannel, 0, 0);  // Stop pullyMotor
  Serial.println("All motors stopped due to emergency stop.");
  
  // Update LEDs to indicate emergency stop (optional)
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(redLEDPin, HIGH);
}

// Function to move motors to a specific (x, y) position
void moveMotorsTo(int x, int y) {
  // Validate motor pulse ranges
  if (x < firstArmMin || x > firstArmMax) {
    Serial.println("Error: X position out of range!");
    return;
  }
  if (y < secondArmMin || y > secondArmMax) {
    Serial.println("Error: Y position out of range!");
    return;
  }

  // Set PWM for firstArm (X-axis) motor
  pwm.setPWM(firstArmChannel, 0, x); // Adjust Channel if necessary
  // Set PWM for secondArm (Y-axis) motor
  pwm.setPWM(secondArmChannel, 0, y); // Adjust Channel if necessary
  
  Serial.print("Moved to Position - X: ");
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.println(y);
  
  // Optional: Add a delay to allow motors to reach the position
  delay(500); // 500 milliseconds
}

// Function to sweep the pully motor from min to max and back
void sweepPullyMotor() {
  Serial.println("Starting sweep of Pully Motor...");

  // Sweep from minimum to maximum pulse
  for (int pulse = pullyMotorMin; pulse <= pullyMotorMax; pulse += 10) { // Increment by 10 for smoother sweep
    if (emergencyStop) { // Check for emergency stop during the sweep
      Serial.println("Emergency stop detected during sweep! Halting sweep.");
      stopMotors();
      return; // Exit the sweep function
    }

    // Validate pulse range
    if (pulse < pullyMotorMin || pulse > pullyMotorMax) {
      Serial.print("Invalid pulse detected: ");
      Serial.println(pulse);
      break; // Exit sweep to prevent over-driving
    }

    pwm.setPWM(pullyMotorChannel, 0, pulse);
    Serial.print("Pully Motor Pulse: ");
    Serial.println(pulse);
    delay(100); // Adjust delay for sweep speed
  }

  // Pause at maximum position
  Serial.println("Pully Motor reached maximum position. Pausing...");
  delay(1000); // 1 second pause

  // Sweep back from maximum to minimum pulse
  for (int pulse = pullyMotorMax; pulse >= pullyMotorMin; pulse -= 10) { // Decrement by 10 for smoother sweep
    if (emergencyStop) { // Check for emergency stop during the sweep
      Serial.println("Emergency stop detected during sweep! Halting sweep.");
      stopMotors();
      return; // Exit the sweep function
    }

    // Validate pulse range
    if (pulse < pullyMotorMin || pulse > pullyMotorMax) {
      Serial.print("Invalid pulse detected: ");
      Serial.println(pulse);
      break; // Exit sweep to prevent over-driving
    }

    pwm.setPWM(pullyMotorChannel, 0, pulse);
    Serial.print("Pully Motor Pulse: ");
    Serial.println(pulse);
    delay(100); // Adjust delay for sweep speed
  }

  // Pause at minimum position
  Serial.println("Pully Motor returned to minimum position. Sweep complete.");
  delay(1000); // 1 second pause
}

// Interrupt Service Routine for Emergency Stop
void handleEmergencyStop() {
  emergencyStop = true;
}

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("Initializing Chessboard Motor Controller...");

  // Initialize the PWM driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Set the oscillator frequency (27MHz for Adafruit boards)
  pwm.setPWMFreq(60); // Set PWM frequency to 60Hz (adjust as needed)

  // Initialize the emergency stop button pin with internal pull-up resistor
  pinMode(stopButtonPin, INPUT_PULLUP);
  
  // Initialize LED pins (optional)
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  
  // Set initial LED states (optional)
  digitalWrite(greenLEDPin, HIGH); // Green LED on: normal operation
  digitalWrite(redLEDPin, LOW);    // Red LED off
  
  // Attach interrupt to the emergency stop button
  attachInterrupt(digitalPinToInterrupt(stopButtonPin), handleEmergencyStop, FALLING);
  
  // Move all motors to their neutral positions
  Serial.println("Moving motors to neutral positions...");
  moveToNeutral(firstArmChannel, firstArmMin, firstArmMax);
  moveToNeutral(secondArmChannel, secondArmMin, secondArmMax);
  moveToNeutral(pullyMotorChannel, pullyMotorMin, pullyMotorMax);
  
  Serial.println("Motors initialized to neutral positions. Starting traversal of Bottom Row.");
  
  // Traverse the Bottom Row (Row 0)
  for (int col = 0; col < GRID_SIZE; col++) {
    if (emergencyStop) { // Check for emergency stop before each move
      Serial.println("Emergency stop detected! Halting traversal.");
      stopMotors();
      while (true) {
        // Infinite loop to halt further execution
      }
    }
    
    // Move to the current position in the Bottom Row
    Serial.print("Moving to Bottom Row, Column ");
    Serial.println(col);
    moveMotorsTo(chessBoard[0][col].x, chessBoard[0][col].y);
    
    // Optional: Add a delay between moves
    delay(5000); // 5 seconds between moves
  }
  
  Serial.println("Completed traversal of Bottom Row. Starting sweep of Pully Motor.");
  
  // Start sweeping the pully motor
  sweepPullyMotor();
  
  Serial.println("Sweep complete. Ready for manual testing.");
  delay(1000); // Brief pause before entering loop
}

void loop() {
  // Check the state of the emergency stop button
  if (emergencyStop) { // Emergency stop activated
    Serial.println("Emergency stop activated!");
    stopMotors(); // Immediately stop all motors
    // Halt further execution
    while (true) {
      // Optionally, you can add visual or auditory indicators here
    }
  }

  // The traversal loop is commented out to allow individual testing
  /*
  if (!emergencyStop) {
    for (int row = 0; row < GRID_SIZE; row++) {
      for (int col = 0; col < GRID_SIZE; col++) {
        // Retrieve the current position's coordinates
        int currentX = chessBoard[row][col].x;
        int currentY = chessBoard[row][col].y;
        
        // Move motors to the current position
        moveMotorsTo(currentX, currentY);
        
        // Optional: Add a delay between moves
        delay(200); // 200 milliseconds
      }
    }
    
    // Optional: Add a longer delay after completing traversal
    Serial.println("Completed chessboard traversal. Restarting in 5 seconds.");
    delay(5000); // 5 seconds before repeating
  }
  */

  // Manual testing: Move to specific squares by specifying row and column
  // Uncomment the following lines to test individual squares

  /*
  // Example 1: Move to the bottom-right square (row 0, column 0)
  Serial.println("Testing: Moving to bottom-right square (row 0, column 0)");
  moveMotorsTo(chessBoard[0][0].x, chessBoard[0][0].y);
  delay(2000); // Wait for 2 seconds

  // Example 2: Move to the middle square (row 0, column 3)
  Serial.println("Testing: Moving to middle square (row 0, column 3)");
  moveMotorsTo(chessBoard[0][3].x, chessBoard[0][3].y);
  delay(2000); // Wait for 2 seconds

  // Example 3: Move to the top-left square (row 0, column 6)
  Serial.println("Testing: Moving to top-left square (row 0, column 6)");
  moveMotorsTo(chessBoard[0][6].x, chessBoard[0][6].y);
  delay(2000); // Wait for 2 seconds
  */

  // Keep the loop running without doing anything else
  // This prevents the loop from exiting and allows continuous testing
  delay(100); // Short delay to prevent rapid looping
}
