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
  // Wait here until we press the button
  while (!buttonPressed()) {}

  long lastStepTaken = 0;
  while (!buttonPressed()) {
    long currentTime = millis();
    if (currentTime - lastStepTaken > 500) {
      step();
      lastStepTaken = currentTime;
    }
  }
}
