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

// Tunable robot constants. Adjust after calibration.
namespace RobotConfig {
  // Drivetrain geometry.
  constexpr double WheelDiameterM = 0.122;                       // 65mm wheel
  constexpr double WheelCircumferenceM = WheelDiameterM * PI;
  constexpr double WheelBaseM = 0.20;                            // distance between wheels

  // Motor + driver: 200 full steps/rev * microsteps per step (currently 1).
  constexpr long StepsPerRevolution = 200;
  constexpr double StepsPerMeter = StepsPerRevolution / WheelCircumferenceM;

  // Motion limits: tune experimentally so the bot doesn't skip steps.
  constexpr double MaxStepsPerS = 1000.0;
  constexpr double MaxStepsPerSPerS = 750.0;

  // The two motors are mounted facing opposite directions, so "forward"
  // means opposite logical directions on the two drivers.
  constexpr bool M1Forward = true;
  constexpr bool M2Forward = false;
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

// Generates a trapezoidal (or triangular, for short moves) velocity profile
// and emits step pulses at the right time. update() must be called as often
// as possible. Math is from Lesson 2 slides:
//   d = 1/2 * a * t^2   ->   t_n = sqrt(2*n / a)   for the n-th step
//   trapezoidal if v_peak = sqrt(a*d) > v_max, else triangular
class StepperMotionController {
public:
  StepperMotionController(StepperMotor& stepper) :
    stepper_(stepper)
  {}

  void setTarget(long steps, bool direction, double maxStepsPerS, double maxStepsPerSPerS) {
    if (steps <= 0 || maxStepsPerS <= 0.0 || maxStepsPerSPerS <= 0.0) {
      // consider motion completed
      targetSteps_ = 0;
      stepsTaken_ = 0;
      return;
    }

    targetSteps_ = steps;
    direction_ = direction;
    maxV_ = maxStepsPerS;
    accel_ = maxStepsPerSPerS;
    stepsTaken_ = 0;

    // Decide profile: triangular if peak velocity stays below the cap.
    //   v_peak^2 = a * d   (peak occurs at d/2)
    double vPeakSq = accel_ * (double)targetSteps_;
    if (vPeakSq <= maxV_ * maxV_) {
      // Triangular: accel half, decel half. Round accel up so the
      // decel side is never longer than the accel side.
      stepsAccel_ = (targetSteps_ + 1) / 2;
      stepsCruise_ = 0;
      tAccelEndS_ = sqrt(2.0 * stepsAccel_ / accel_);
      tTotalS_ = 2.0 * tAccelEndS_;
    } else {
      // Trapezoidal: accelerate to v_max, cruise, then decelerate.
      //   t_a = v_max / a;  d_accel = v_max^2 / (2a)
      stepsAccel_ = (long)(maxV_ * maxV_ / (2.0 * accel_));
      if (stepsAccel_ < 1) stepsAccel_ = 1;
      stepsCruise_ = targetSteps_ - 2 * stepsAccel_;
      if (stepsCruise_ < 0) stepsCruise_ = 0;
      tAccelEndS_ = maxV_ / accel_;
      tTotalS_ = 2.0 * tAccelEndS_ + (double)stepsCruise_ / maxV_;
    }

    startTimeUs_ = micros();
    nextStepDueUs_ = computeStepDueUs(1);
  }

  // Returns true once the motion is complete.
  bool update() {
    if (stepsTaken_ >= targetSteps_) {
      return true;
    }

    unsigned long now = micros() - startTimeUs_;
    if ((long)(now - nextStepDueUs_) >= 0) {
      stepper_.step(direction_);
      stepsTaken_++;
      if (stepsTaken_ >= targetSteps_) {
        return true;
      }
      nextStepDueUs_ = computeStepDueUs(stepsTaken_ + 1);
    }
    return false;
  }

  bool isDone() const { return stepsTaken_ >= targetSteps_; }

private:
  // Time (in microseconds since start) at which the n-th step (1-based)
  // should be taken.
  unsigned long computeStepDueUs(long n) const {
    double t;
    if (n <= stepsAccel_) {
      // Accel phase: t = sqrt(2n / a)
      t = sqrt(2.0 * (double)n / accel_);
    } else if (n <= stepsAccel_ + stepsCruise_) {
      // Cruise phase: constant velocity v_max
      long cruiseIndex = n - stepsAccel_;
      t = tAccelEndS_ + (double)cruiseIndex / maxV_;
    } else {
      // Decel phase: mirror of accel from the end of motion.
      long stepsFromEnd = targetSteps_ - n + 1;
      if (stepsFromEnd < 1) stepsFromEnd = 1;
      double tFromEnd = sqrt(2.0 * (double)stepsFromEnd / accel_);
      t = tTotalS_ - tFromEnd;
      if (t < tAccelEndS_) t = tAccelEndS_;
    }
    return (unsigned long)(t * 1.0e6);
  }

  StepperMotor& stepper_;
  long targetSteps_ = 0;
  long stepsTaken_ = 0;
  long stepsAccel_ = 0;
  long stepsCruise_ = 0;
  bool direction_ = true;
  double maxV_ = 0.0;
  double accel_ = 0.0;
  double tAccelEndS_ = 0.0;
  double tTotalS_ = 0.0;
  unsigned long startTimeUs_ = 0;
  unsigned long nextStepDueUs_ = 0;
};

class Robot {
public:
  Robot() :
    m1_(Pins::M1::Step, Pins::M1::Dir),
    m2_(Pins::M2::Step, Pins::M2::Dir),
    m1Motion_(m1_),
    m2Motion_(m2_)
  {}

  void forward(double distanceM) {
    long steps = metersToSteps(distanceM);
    runBoth(steps, RobotConfig::M1Forward, steps, RobotConfig::M2Forward);
  }

  void reverse(double distanceM) {
    long steps = metersToSteps(distanceM);
    runBoth(steps, !RobotConfig::M1Forward, steps, !RobotConfig::M2Forward);
  }

  // 90-degree pivot in place: each wheel travels (wheelBase/2) * (PI/2).
  void left() {
    long steps = pivot90Steps();
    runBoth(steps, !RobotConfig::M1Forward, steps, RobotConfig::M2Forward);
  }

  void right() {
    long steps = pivot90Steps();
    runBoth(steps, RobotConfig::M1Forward, steps, !RobotConfig::M2Forward);
  }

private:
  static long metersToSteps(double distanceM) {
    return (long)(distanceM * RobotConfig::StepsPerMeter + 0.5);
  }

  static long pivot90Steps() {
    double arcM = (RobotConfig::WheelBaseM * 0.5) * (PI * 0.5);
    return (long)(arcM * RobotConfig::StepsPerMeter + 0.5);
  }

  void runBoth(long s1, bool d1, long s2, bool d2) {
    m1Motion_.setTarget(s1, d1, RobotConfig::MaxStepsPerS, RobotConfig::MaxStepsPerSPerS);
    m2Motion_.setTarget(s2, d2, RobotConfig::MaxStepsPerS, RobotConfig::MaxStepsPerSPerS);
    while (!m1Motion_.isDone() || !m2Motion_.isDone()) {
      m1Motion_.update();
      m2Motion_.update();
    }
  }

  // Members are constructed in declaration order; motors must come before
  // the controllers that reference them.
  StepperMotor m1_;
  StepperMotor m2_;
  StepperMotionController m1Motion_;
  StepperMotionController m2Motion_;
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
  // Runs all moves atomically (doesn't return until moves are completed).
  // Templated on N so we can use a range-based for over a real array
  // (a bare T[] decays to a pointer and loses its size).
  template <size_t N>
  static void runMovesOnRobot(const RobotMove (&moves)[N], Robot& robot) {
    for (const auto& move : moves) {
      switch (move.moveType) {
        case RobotMoveType::forward:
          robot.forward(move.distanceM);
          break;
        case RobotMoveType::backward:
          robot.reverse(move.distanceM);
          break;
        case RobotMoveType::left:
          robot.left();
          break;
        case RobotMoveType::right:
          robot.right();
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

void loop() {
  static const RobotMove moves[] = {
    {RobotMoveType::forward, 0.5},
    {RobotMoveType::backward, 0.5},
    {RobotMoveType::left, 0.0},
    {RobotMoveType::right, 0.0},
    {RobotMoveType::forward, 2.0},
    {RobotMoveType::backward, 2.0},
    {RobotMoveType::left, 0.0},
    {RobotMoveType::right, 0.0}
  };

  static Robot robot;

  // Wait here until we press the button
  while (!buttonPressed()) {}

  RobotControl::runMovesOnRobot(moves, robot);
}
