// =========================
// BKA30D Stepper Wrapper
// =========================
class BKA30DStepper {
  private:
    uint8_t pins[3];
    uint8_t currentStep = 0;

    static constexpr uint8_t driveSequence[6][3] = {
      {1, 0, 1}, // 0x5
      {0, 0, 1}, // 0x1
      {0, 1, 1}, // 0x3
      {0, 1, 0}, // 0x2
      {1, 1, 0}, // 0x6
      {1, 0, 0}  // 0x4
    };

    void setContacts(uint8_t stepIndex) {
      digitalWrite(pins[0], driveSequence[stepIndex][0]);
      digitalWrite(pins[1], driveSequence[stepIndex][1]);
      digitalWrite(pins[2], driveSequence[stepIndex][2]);
    }

  public:
    BKA30DStepper(uint8_t pin0, uint8_t pin1, uint8_t pin2) {
      pins[0] = pin0;
      pins[1] = pin1;
      pins[2] = pin2;
      for (int i = 0; i < 3; ++i) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
      }
    }

    void stepForward() {
      currentStep = (currentStep + 1) % 6;
      setContacts(currentStep);
    }

    void stepBackward() {
      currentStep = (currentStep + 5) % 6;
      setContacts(currentStep);
    }
};