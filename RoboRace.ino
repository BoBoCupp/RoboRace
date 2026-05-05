namespace Pins {
  namespace M1 {
    constexpr int Step = 11;
    constexpr int Dir = 6;
  }
  namespace M2 {
    constexpr int Step = 9;
    constexpr int Dir = 4;
  }

  constexpr int Button = 2;
}

class StepperMotor {
  public:
    StepperMotor(int stepPin, int dirPin) :
      stepPin_(stepPin),
      dirPin_(dirPin)
    {}

    void step(bool direction) {
      // Change direction if needed
      if (lastDirection_ != direction) {
        lastDirection_ = direction;
        if (direction) {
          digitalWrite(dirPin_, HIGH);
        } else {
          digitalWrite(dirPin_, LOW);
        }
      }

      digitalWrite(stepPin_, HIGH);
      delayMicroseconds(2);
      digitalWrite(stepPin_, LOW);
    }

  private:
    bool lastDirection_ = true;
    int stepPin_;
    int dirPin_;
};

class StepperMotionController {
public:
  StepperMotionController(StepperMotor& stepper) :
    stepper_(stepper)
  {}

  void setTarget(int steps, int maxStepsPerS, int maxStepsPerSPerS) {
    targetSteps_ = steps;
    remainingSteps_ = steps;
    maxStepsPerS_ = maxStepsPerS;
    maxStepsPerSPerS_ = maxStepsPerSPerS;

    // Calculate nextStepDueS
    nextStepDueS_ = 0.0;
  }

  bool update() {
  }
private:
  double nextStepDueS_ = 0.0;
  int maxStepsPerS_ = 0;
  int maxStepsPerSPerS_ = 0;
  int remainingSteps_ = 0;
  int targetSteps_ = 0;
  StepperMotor& stepper_;
};

class Robot {
public:
  void forward(double distanceM) {
  }

  void reverse(double distanceM) {
  }

  void left() {
  }

  void right() {
  }

private:
  StepperMotionController m1Motion_;
  StepperMotionController m2Motion_;
  StepperMotor m1_;
  StepperMotor m2_;
};

enum class RobotMoveType {
  forward,
  backward,
  left,
  right
};

struct RobotMove {
  RobotMoveType moveType;
  double distanceM;
};

class RobotControl {
public:
  // Runs all moves atomically (doesn't return until moves are completed)
  static void runMovesOnRobot(RobotMove moves[], Robot& robot) {
    for(auto move : moves) {
      switch(move.moveType) {
        case RobotMoveType::forward:
          robot.foward(move.distanceM);
          break;
        case RobotMoveType::backward:
          robot.backward(move.distanceM);
          break;
        case RobotMoveType::left:
          robot.left();
          break;
        case RobotMoveType::right:
          robot.right(move.distanceM);
          break;
      }
    }
  }
};

void setup() {
  // put your setup code here, to run once:
  pinMode(Pins::M1::Step, OUTPUT);
  pinMode(Pins::M1::Dir, OUTPUT);
  digitalWrite(Pins::M1::Step, LOW);
  digitalWrite(Pins::M1::Dir, HIGH);

  pinMode(Pins::M2::Step, OUTPUT);
  pinMode(Pins::M2::Dir, OUTPUT);
  digitalWrite(Pins::M2::Step, LOW);
  digitalWrite(Pins::M2::Dir, LOW);

  pinMode(Pins::Button, INPUT);

  Serial.begin(115200);
}

bool buttonPressed() {
  // Using the static keyword like this in a function gives you
  // a variable whose value persists across function calls
  static long pressStartTime = 0;
  static bool lastReading = LOW;
  static bool buttonArmed = false;

  bool reading = digitalRead(Pins::Button);
  if (reading == HIGH && lastReading == LOW) {
    pressStartTime = millis();

    Serial.print("pressStartTime:");
    Serial.println(pressStartTime);
  }

  lastReading = reading;

  // Make sure we are holding the button down so we don't get any
  // bouncing multiple button presses when interacting with the button.
  if (reading == HIGH && (millis() - pressStartTime >= 30)) {
    if (!buttonArmed) {
      Serial.println("button is armed");
    }
    buttonArmed = true;
  }

  if (buttonArmed && reading == LOW) {
    // We just pressed the button
    buttonArmed = false;
    Serial.println("button is pressed");
    return true;
  }

  return false;
}

void step() {
  digitalWrite(Pins::M1::Step, HIGH);
  digitalWrite(Pins::M2::Step, HIGH);

  delayMicroseconds(1000000);

  digitalWrite(Pins::M1::Step, LOW);
  digitalWrite(Pins::M2::Step, LOW);
}

void loop() {
  static std::array<RobotMove> moves = {
    {RobotMoveType::forward, 0.5},
    {RobotMoveType::backward, 0.5},
    {RobotMoveType::left, 0.0},
    {RobotMoveType::right, 0.0},
    {RobotMoveType::forward, 2.0},
    {RobotMoveType::backward, 2.0},
    {RobotMoveType::left, 0.0},
    {RobotMoveType::right, 0.0}
  };

  // Wait here until we press the button
  while (!buttonPressed()) {}

  Robot robot;
  RobotControl::runMovesOnRobot(moves, robot);
}
