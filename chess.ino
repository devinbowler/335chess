#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <game_logic.ino>
#include <Keypad.h>
byte rows[4] = {4, 5, 6, 7};//connect to the row pinouts of the keypad
byte cols[4] = {8, 9, 10, 11};//connect to the column pinouts of the keypad 

char keys[4][4] = { //create 2D array for keys
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'},

};

Keypad mykeypad = Keypad(makeKeymap(keys), rows, cols, 4, 4);//initialize an instance of class NewKeypad
// Initialize the PWM driver with the default I2C address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define motor channels
const int firstArmChannel = 0;    // Channel 0 on PCA9685
const int secondArmChannel = 1;   // Channel 1 on PCA9685
const int pullyMotorChannel = 2;  // Channel 2 on PCA9685 (not used in testing)

// Define adjusted minimum and maximum pulse lengths for the motors
const int firstArmMin = 200; // Adjusted minimum for motor 0 to skip dead zone
const int firstArmMax = 520; // Adjusted maximum for motor 0 (80% range)

const int secondArmMin = 100; // Adjusted minimum for motor 1 to skip dead zone
const int secondArmMax = 450; // Adjusted maximum for motor 1 (70% range)

const int pullyMotorMin = 50;    // Adjusted minimum for motor 2
const int pullyMotorMax = 800; // Increased maximum for motor 2 to ensure full lowering

// Define the pin for the electromagnet
const int electromagnetPin = 3;   // Digital Pin D3

// Define the pin for the emergency stop button
const int stopButtonPin = 2; // Digital Pin D2

// Flag to track the state of the emergency stop
volatile bool emergencyStop = false;

// Define a struct to hold motor positions
struct Position {
  int x;
  int y;
};

// 8x8 chessboard grid with coordinate pairs
const Position chessBoard[8][8] = {
  { {307,541}, {340,525}, {365,513}, {390,490}, {410,470}, {430,450}, {450,425}, {470,400} },
  { {295,530}, {325,510}, {350,495}, {375,475}, {395,455}, {420,435}, {438,410}, {460,380} },
  { {288,513}, {318,495}, {343,475}, {368,458}, {388,438}, {418,418}, {434,393}, {458,358} },
  { {290,497}, {317,475}, {348,458}, {363,445}, {385,430}, {405,405}, {428,377}, {453,347} },
  { {285,477}, {297,455}, {323,430}, {345,415}, {365,400}, {405,380}, {410,360}, {450,320} },
  { {247,486}, {265,450}, {290,423}, {315,415}, {335,395}, {380,375}, {390,350}, {420,300} },
  { {235,475}, {250,435}, {275,405}, {300,400}, {320,380}, {370,360}, {378,335}, {410,280} },
  { {223,464}, {235,420}, {260,387}, {285,385}, {305,365}, {360,345}, {366,320}, {400,260} }
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

void pulleyToNeutral() {
  // Directly set the pulley to the 'up' (neutral) position
  movePullyMotor(pullyMotorMax);
  delay(100);
}


// Function to stop all motors immediately
void stopMotors() {
  pwm.setPWM(firstArmChannel, 0, 0);    // Stop firstArm
  pwm.setPWM(secondArmChannel, 0, 0);   // Stop secondArm
  pwm.setPWM(pullyMotorChannel, 0, 0);  // Stop pullyMotor (not in use)
  Serial.println("All motors stopped due to emergency stop.");
}

// Function to move the pully motor to a specific pulse
void movePullyMotor(int pulse) {
  // Validate pulse range
  if (pulse < pullyMotorMin) pulse = pullyMotorMin;
  if (pulse > pullyMotorMax) pulse = pullyMotorMax;

  pwm.setPWM(pullyMotorChannel, 0, pulse);
  Serial.print("Pully Motor Pulse: ");
  Serial.println(pulse);
}

void raisePullyMotor() {
  // If pullyMotorMin = up, pullyMotorMax = down, we move from min to max
  Serial.println("Raising pulley...");
  for (int p = pullyMotorMin; p <= pullyMotorMax; p += 10) {
    if (emergencyStop) { stopMotors(); return; }
    movePullyMotor(p);
    delay(50);
  }
  movePullyMotor(pullyMotorMax);
  Serial.println("Reached down position.");
  delay(500);
}

void dropPullyMotor() {
  // If pullyMotorMin = up, pullyMotorMax = down, raise goes from max to min
  Serial.println("Dropping pulley...");
  for (int p = pullyMotorMax; p >= pullyMotorMin; p -= 10) {
    if (emergencyStop) { stopMotors(); return; }
    movePullyMotor(p);
    delay(50);
  }
  movePullyMotor(pullyMotorMin);
  Serial.println("Reached up position.");
  delay(500);
}



// Function to move motors to a specific (x, y) position
void moveMotorsTo(int targetX, int targetY) {
  // Get the current position of each motor
  static int currentX = (firstArmMin + firstArmMax) / 2; // Assume neutral position as the starting position
  static int currentY = (secondArmMin + secondArmMax) / 2; // Assume neutral position as the starting position

  int stepX = (currentX < targetX) ? 20 : -20; // Smaller step for slower sweep
  int stepY = (currentY < targetY) ? 20 : -20; // Smaller step for slower sweep

  Serial.print("Moving motors to Target Position - X: ");
  Serial.print(targetX);
  Serial.print(", Y: ");
  Serial.println(targetY);

  // Loop until both X and Y reach their target positions
  while (currentX != targetX || currentY != targetY) {
    if (emergencyStop) {
      Serial.println("Emergency stop detected! Halting motor movement.");
      stopMotors();
      return; // Exit the function immediately
    }

    // Sweep the X motor
    if (currentX != targetX) {
      currentX += stepX;
      if ((stepX > 0 && currentX > targetX) || (stepX < 0 && currentX < targetX)) {
        currentX = targetX;
      }
      pwm.setPWM(firstArmChannel, 0, currentX);
    }

    // Sweep the Y motor
    if (currentY != targetY) {
      currentY += stepY;
      if ((stepY > 0 && currentY > targetY) || (stepY < 0 && currentY < targetY)) {
        currentY = targetY;
      }
      pwm.setPWM(secondArmChannel, 0, currentY);
    }

    delay(100); // Delay to control the sweep speed (100ms for slower sweep)
  }

  Serial.print("Moved to Position - X: ");
  Serial.print(currentX);
  Serial.print(", Y: ");
  Serial.println(currentY);
}

// Interrupt Service Routine for Emergency Stop
void handleEmergencyStop() {
  emergencyStop = true;
}

// Keypad input handling
String keypadInput = "";

void handleKeypadInput() {
  char key = myKeypad.getKey();

  if (key) {
    if (key == '*') { // Confirm key
      if (keypadInput.length() == 4) {
        handleTurn(keypadInput);
        keypadInput = "";
      } else {
        Serial.println("Invalid input length. Enter 4 digits.");
      }
    } else if (key == '#') { // Clear key
      keypadInput = "";
      Serial.println("Input cleared.");
    } else if (isdigit(key)) {
      if (keypadInput.length() < 4) {
        keypadInput += key;
        Serial.print("Keypad input: ");
        Serial.println(keypadInput);
      } else {
        Serial.println("Input already 4 digits. Press * to confirm or # to clear.");
      }
    }
  }
}

// Game logic integration
void parseMove(const String &input, int &fromRow, int &fromCol, int &toRow, int &toCol) {
  fromRow = input[0] - '1';
  fromCol = input[1] - '1';
  toRow = input[2] - '1';
  toCol = input[3] - '1';
}

bool isValidMove(int fromRow, int fromCol, int toRow, int toCol) {
  if (fromRow < 0 || fromRow >= SIZE || fromCol < 0 || fromCol >= SIZE ||
      toRow < 0 || toRow >= SIZE || toCol < 0 || toCol >= SIZE) {
    return false;
  }

  char piece = board[fromRow][fromCol];
  if ((isWhiteTurn && islower(piece)) || (!isWhiteTurn && isupper(piece))) {
    return false;
  }

  char destination = board[toRow][toCol];
  if ((isWhiteTurn && isupper(destination)) || (!isWhiteTurn && islower(destination))) {
    return false;
  }

  return true;
}

void updateBoard(int fromRow, int fromCol, int toRow, int toCol) {
  board[toRow][toCol] = board[fromRow][fromCol];
  board[fromRow][fromCol] = ' ';
}

void handleTurn(const String &input) {
  int fromRow, fromCol, toRow, toCol;
  parseMove(input, fromRow, fromCol, toRow, toCol);

  if (isValidMove(fromRow, fromCol, toRow, toCol)) {
    updateBoard(fromRow, fromCol, toRow, toCol);

    moveMotorsTo(chessBoard[fromRow][fromCol].x, chessBoard[fromRow][fromCol].y);
    delay(1000); // Simulate pickup
    raisePullyMotor();
    moveMotorsTo(chessBoard[toRow][toCol].x, chessBoard[toRow][toCol].y);
    delay(1000); // Simulate drop-off
    dropPullyMotor();

    isWhiteTurn = !isWhiteTurn;
    Serial.println("Move executed successfully.");
  } else {
    Serial.println("Invalid move.");
  }
}

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("Initializing Chessboard Motor Controller...");

  // Initialize the PWM driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Set the oscillator frequency (27MHz for Adafruit boards)
  pwm.setPWMFreq(60); // Set PWM frequency to 60Hz (adjust as needed)

  // Initialize the electromagnet pin as OUTPUT early
  pinMode(electromagnetPin, OUTPUT);
  digitalWrite(electromagnetPin, LOW); // Ensure electromagnet is off initially

  // Initialize the emergency stop button pin with internal pull-up resistor
  pinMode(stopButtonPin, INPUT_PULLUP);
  
  // Attach interrupt to the emergency stop button
  attachInterrupt(digitalPinToInterrupt(stopButtonPin), handleEmergencyStop, FALLING);
  
  // Move all motors to their neutral positions
  Serial.println("Moving motors to neutral positions...");
  moveToNeutral(firstArmChannel, firstArmMin, firstArmMax);
  moveToNeutral(secondArmChannel, secondArmMin, secondArmMax);
  pulleyToNeutral();
  
  Serial.println("Motors initialized to neutral positions. Starting pick and place sequence.");
  delay(1000); // Brief pause before starting

    // ----- Row Testing Sequence -----
  // Choose a specific row to test
  int testRow = 4; 

  for (int col = 0; col < 8; col++) {
    int targetX = chessBoard[testRow][col].x;
    int targetY = chessBoard[testRow][col].y;

    // If this position is (0,0) and you want to skip it, you can do:
    if (targetX == 0 && targetY == 0) {
      Serial.print("Skipping empty coordinate at col ");
      Serial.println(col);
      continue;
    }

    Serial.print("Moving to Row ");
    Serial.print(testRow);
    Serial.print(", Col ");
    Serial.print(col);
    Serial.print(": X=");
    Serial.print(targetX);
    Serial.print(", Y=");
    Serial.println(targetY);

    // Move to the coordinate
    moveMotorsTo(targetX, targetY);

    // Wait 5 seconds before moving to the next coordinate
    delay(5000);
  }

  // ----- Pick-Up Sequence -----
  /* Move to piece
  moveMotorsTo(chessBoard[2][2].x, chessBoard[2][2].y);

  // Lower the pulley (down)
  dropPullyMotor();

  // Magnet on (pick piece)
  digitalWrite(electromagnetPin, HIGH);
  delay(500);

  // Raise pulley (up) before moving
  raisePullyMotor();
  delay(500); // Ensure it's fully raised before moving

  // Move to target
  moveMotorsTo(chessBoard[0][0].x, chessBoard[0][0].y);

  // Lower the pulley to place piece
  dropPullyMotor();

  // Release magnet
  digitalWrite(electromagnetPin, LOW);
  delay(500);

  // Raise pulley again
  raisePullyMotor();

  // Move back to neutral
  moveToNeutral(firstArmChannel, firstArmMin, firstArmMax);
  moveToNeutral(secondArmChannel, secondArmMin, secondArmMax);
  pulleyToNeutral();
  */
}

void loop() {
  // Continuously check the state of the emergency stop button
  if (emergencyStop) { // Emergency stop activated
    Serial.println("Emergency stop activated!");
    stopMotors(); // Immediately stop all motors
    digitalWrite(electromagnetPin, LOW); // Ensure electromagnet is off
    // Halt further execution
    while (true) {
      // Optionally, you can add visual or auditory indicators here
    }
  }
  handleKeypadInput();
  // No other actions in loop; sequence runs once in setup()
  delay(100); // Short delay to prevent rapid looping
}
