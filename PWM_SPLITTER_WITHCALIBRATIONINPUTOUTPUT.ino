#include <Arduino.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------
#define SWITCH_PIN      2   // PD2 (Digital Pin 2) for the calibration button
#define PWM_INPUT_PIN   8   // PB0 (Digital Pin 8) for input frequency measurement
#define PWM_OUTPUT_PIN1 9   // PB1 (Digital Pin 9, OC1A) for output PWM
#define PWM_OUTPUT_PIN2 10  // PB2 (Digital Pin 10, OC1B) for output PWM
#define LED_PIN         LED_BUILTIN  // On most boards, this is pin 13

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define MAX_SPEED         200.0  // Max speed after quadratic calc
#define MAX_FREQUENCY     200.0  // Max frequency for PWM output
#define CALIBRATION_STEPS 3
#define DEBOUNCE_DELAY    100    // ms

// ---------------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------------

// Frequency Measurement
volatile float measuredFrequency = 0.0;
volatile bool  frequencyUpdated  = false;
volatile unsigned long lastPulseTime = 0;
volatile bool lastState = LOW;

// Calibration
int   calibrationStep = 0;
float calibrationFrequencies[CALIBRATION_STEPS] = {0}; 
const float knownSpeeds[CALIBRATION_STEPS]      = {40.0, 60.0, 110.0};

// Quadratic Coefficients
float coeffA = 0, coeffB = 0, coeffC = 0;

// ---------------------------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------------------------
void handleCalibration();
void updateCalibrationLED(int step);
void calculateCoefficients();
float calculateCurrentSpeed(float frequency);
void updatePWM(float speed);

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  // 1) Pin Setup
  pinMode(SWITCH_PIN, INPUT);         // Calibration Button (assumed pull-down or external resistor)
  pinMode(PWM_INPUT_PIN, INPUT);      // Frequency input on pin 8
  pinMode(PWM_OUTPUT_PIN1, OUTPUT);   // PWM output pin 9 (OC1A)
  pinMode(PWM_OUTPUT_PIN2, OUTPUT);   // PWM output pin 10 (OC1B)
  pinMode(LED_PIN, OUTPUT);           // LED for calibration feedback

  // 2) Enable Pin-Change Interrupt on pin 8 (PB0 => PCINT0)
  PCICR  |= (1 << PCIE0);    // Enable Pin Change Interrupts for PCINT[7:0]
  PCMSK0 |= (1 << PCINT0);   // Enable interrupt for PCINT0 (PB0)
  // => Triggers ISR(PCINT0_vect) on ANY change of pin 8

  // 3) Configure Timer1 for Dual PWM (Mode 14: Fast PWM w/ ICR1 as TOP)
  TCCR1A = 0;
  TCCR1B = 0;
  
  // Fast PWM, mode 14: WGM13=1, WGM12=1, WGM11=1, WGM10=0
  // Non-inverting outputs on OC1A/OC1B: COM1A1=1, COM1B1=1
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); 
  TCCR1B |= (1 << WGM13)  | (1 << WGM12);

  // Prescaler = 8 => Timer clock = 16 MHz / 8 = 2 MHz
  TCCR1B |= (1 << CS11);

  // Initial PWM frequency ~100 Hz (just a placeholder)
  ICR1  = 20000 - 1;          // ~100 Hz
  OCR1A = ICR1 / 2;           // 50% duty
  OCR1B = ICR1 / 2;           // 50% duty

  // 4) Collect Calibration Data via Button Presses
  //    - Wait here until all 3 calibration steps are complete.
  while (calibrationStep < CALIBRATION_STEPS) {
    handleCalibration();
    updateCalibrationLED(calibrationStep);
  }
  
  // Turn off LED after the last press
  updateCalibrationLED(calibrationStep);

  // 5) Compute the Quadratic Coefficients from the calibration data
  calculateCoefficients();
}

// ---------------------------------------------------------------------------
// Main Loop
// ---------------------------------------------------------------------------
void loop() {
  // After calibration, simply measure frequency, compute speed, print & update PWM
  if (frequencyUpdated) {
    noInterrupts();
    float currentFreq = measuredFrequency;
    frequencyUpdated  = false;
    interrupts();

    // Use the quadratic fit to calculate speed from frequency
    float speed = calculateCurrentSpeed(currentFreq);

    // Print the computed speed to Serial
    Serial.print("Current Speed: ");
    Serial.println(speed);

    // Update Timer1 PWM frequency based on the speed
    updatePWM(speed);
  }
}

// ===========================================================================
// Pin Change Interrupt ISR for PB0 (Digital Pin 8)
// ===========================================================================
ISR(PCINT0_vect) {
  bool currentState = (PINB & (1 << PB0)) != 0; // Read PB0 directly
  unsigned long now = micros();

  // Detect rising edge
  if (currentState && !lastState) {
    unsigned long elapsed = now - lastPulseTime;
    lastPulseTime = now;
    
    if (elapsed > 0) {
      measuredFrequency = 1e6 / (float)elapsed;  // freq = 1 / period, in Hz
      frequencyUpdated  = true;
    }
  }
  lastState = currentState;
}

// ---------------------------------------------------------------------------
// (A) Calibration Step: Read Button, Store Frequency
// ---------------------------------------------------------------------------
void handleCalibration() {
  static uint32_t lastDebounce    = 0;
  static bool     buttonState     = LOW;
  static bool     lastButtonState = LOW;

  bool reading = digitalRead(SWITCH_PIN);

  // Debounce logic
  if (reading != lastButtonState) {
    lastDebounce = millis();
  }

  if ((millis() - lastDebounce) > DEBOUNCE_DELAY && (reading != buttonState)) {
    buttonState = reading;
    // Button is considered pressed if HIGH
    if (buttonState == HIGH) {
      // Store the currently measured frequency in the calibration array
      calibrationFrequencies[calibrationStep] = measuredFrequency;

      // Quick LED flash to acknowledge the press
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);

      // Move to next calibration step
      calibrationStep++;
    }
  }
  lastButtonState = reading;
}

// ---------------------------------------------------------------------------
// (B) LED Behavior for Each Calibration Step
// ---------------------------------------------------------------------------
void updateCalibrationLED(int step) {
  /*
    step == 0 => fast blink (200ms period)
    step == 1 => medium blink (500ms period)
    step == 2 => always ON
    step >= 3 => OFF
  */
  if (step >= 3) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  if (step == 2) {
    // Solid ON
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  // Blink for step=0 or step=1
  static unsigned long lastBlink   = 0;
  static bool          ledState    = false;
  unsigned long        currentTime = millis();

  unsigned long blinkPeriod = (step == 0) ? 200 : 500;

  if (currentTime - lastBlink >= blinkPeriod) {
    lastBlink = currentTime;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}

// ---------------------------------------------------------------------------
// (C) Calculate Quadratic Coefficients for Speed(Frequency)
// ---------------------------------------------------------------------------
//
// We'll assume that the user has pressed the button 3 times at 3 known speeds
// (knownSpeeds[]), capturing the measured frequency each time (calibrationFrequencies[]).
// We solve for A, B, C in:
//     Frequency = A*Speed^2 + B*Speed + C
//
// Then, in runtime, we invert that relationship to get Speed = f(Frequency)
// using the quadratic formula.
void calculateCoefficients() {
  // For convenience: x = knownSpeeds[i], y = calibrationFrequencies[i]
  float x1 = knownSpeeds[0], y1 = calibrationFrequencies[0];
  float x2 = knownSpeeds[1], y2 = calibrationFrequencies[1];
  float x3 = knownSpeeds[2], y3 = calibrationFrequencies[2];

  // We'll do a standard 3-point parabola fit: y = A*x^2 + B*x + C
  // See: https://math.stackexchange.com/questions/356258/
  float denom = (x1 - x2) * (x1 - x3) * (x2 - x3);
  if (fabs(denom) < 1e-12) {
    // Degenerate case: not enough distinct points, etc. 
    // Just default to something safe
    coeffA = 0;
    coeffB = 0;
    coeffC = 0;
    return;
  }

  // Solve for A, B, C
  coeffA = ( x3 * (y2 - y1) 
           + x2 * (y1 - y3) 
           + x1 * (y3 - y2) ) / denom;

  coeffB = ( x3*x3 * (y1 - y2) 
           + x1*x1 * (y2 - y3) 
           + x2*x2 * (y3 - y1) ) / denom;

  coeffC = ( x2*x3*(x2 - x3)*y1
           + x3*x1*(x3 - x1)*y2
           + x1*x2*(x1 - x2)*y3 ) / denom;
}

// ---------------------------------------------------------------------------
// (D) Compute Speed from Measured Frequency via Quadratic Inversion
// ---------------------------------------------------------------------------
//
//   freq = A*speed^2 + B*speed + C
// => 0 = A*speed^2 + B*speed + (C - freq)
// Solve for speed using the quadratic formula. Return the positive root.
float calculateCurrentSpeed(float frequency) {
  // freq = A*s^2 + B*s + C
  // => A*s^2 + B*s + (C - freq) = 0
  float a = coeffA;
  float b = coeffB;
  float c = coeffC - frequency;

  float discriminant = b*b - 4*a*c;
  if (discriminant < 0) {
    return 0;
  }
  
  float sqrt_d = sqrt(discriminant);
  float speed1 = (-b + sqrt_d) / (2*a);
  float speed2 = (-b - sqrt_d) / (2*a);

  // Take the positive solution. Then cap at MAX_SPEED
  float candidate = (speed1 > 0) ? speed1 : speed2;
  if (candidate < 0) candidate = 0;

  // Constrain final speed
  if (candidate > MAX_SPEED) candidate = MAX_SPEED;
  return candidate;
}

// ---------------------------------------------------------------------------
// (E) Update the PWM Frequency on Pins 9 & 10 Based on Speed
// ---------------------------------------------------------------------------
//
// We'll map "speed" to a final PWM frequency (capped by MAX_FREQUENCY).
// For example, if speed is 0..200, we might just use the same numeric range
// for output frequency in Hz, or adapt as needed. 
// This example directly treats "speed" as "output frequency" up to 200 Hz.
void updatePWM(float speed) {
  // Constrain the frequency for Timer1
  float frequencyHz = speed;  // simplest direct mapping: speed -> freq
  if (frequencyHz > MAX_FREQUENCY) {
    frequencyHz = MAX_FREQUENCY;
  }

  // If frequency is too low => turn off
  if (frequencyHz < 1.0) {
    OCR1A = 0;
    OCR1B = 0;
    return;
  }

  // frequency = F_CPU / (prescaler * (1 + ICR1))
  // => ICR1 = (F_CPU / (prescaler*frequency)) - 1
  // With prescaler=8 => Timer clock = 2 MHz
  // => ICR1 = (2,000,000 / frequencyHz) - 1
  uint32_t topValue = (uint32_t)(2000000UL / frequencyHz) - 1;

  // Bound topValue within 16-bit range
  if (topValue < 1)      topValue = 1;
  if (topValue > 65535)  topValue = 65535;

  uint16_t dutyValue = topValue / 2;  // 50% duty cycle

  noInterrupts();
  ICR1  = (uint16_t)topValue;
  OCR1A = dutyValue;
  OCR1B = dutyValue;
  interrupts();
}
