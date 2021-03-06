// Inverted pendulum - for Teensy 3.2

#include "i2c_t3.h"
#include "Encoder.h"
#include "Pendulum.h"
#include "PIDControl.h"
#include "SimpleShell.h"
#include "I2CCom.h"

#define LEFT HIGH
#define RIGHT LOW
#define TRACK_ENCODER_PIN_A 0
#define TRACK_ENCODER_PIN_B 1
#define PENDULUM_ENCODER_PIN_A 2
#define AS5600_I2C_ADDR 0x36
// Add MAGNET_OFFSET to the 12 bit sensor reading so zero is returned when the
// pendulum is hanging at rest.
#define MAGNET_OFFSET (4095 - 3596)
// #define PENDULUM_ENCODER_PIN_B 3
#define LEFT_BUMP_PIN 4
#define RIGHT_BUMP_PIN 5
#define PWM_PIN 10
#define DIR_PIN 12
#define LED_PIN 13
#define CART_RESET_PIN 21

// #define DEBUG 1 // Uncomment to print info for study/debugging

#ifdef DEBUG
elapsedMillis riseTimer = 0;
bool riseTimerStarted = false;
float deltaSetpoint = 0;
#endif

float maxPwm = 16383;  // 100% PWM value with 14-bit resolution
float pwmFreq = 2929.687; // See http://www.pjrc.com/teensy/td_pulse.html
float dutyCycle = 0;  // Percent

float meanAbsError = 0; // Mean absolute error
float alpha = 0.001; // Smoothing parameter for absolute error running average

Encoder trackEncoder(TRACK_ENCODER_PIN_A, TRACK_ENCODER_PIN_B);
// Encoder thetaEncoder(PENDULUM_ENCODER_PIN_A, PENDULUM_ENCODER_PIN_B);

// For communication with AS5600 magnetic sensor ("contactless potentiometer")
I2CCom as5600(AS5600_I2C_ADDR);

// Initialize with update interval and swing-up limit.
// Theta is positive clockwise in degrees, with theta = 0 when hanging at rest.
Pendulum pendulum(2 /* ms */, 100.0 /* deg */);

// Pendulum setpoint: not exactly 180 degrees due to nonlinearity of magnetic
// sensor and (to a lesser extent) imperfect MAGNET_OFFSET calibration.
// If the pendulum systematically drifts
//  - to the right: decrease the setpoint angle
//  - to the left: increase the setpoint angle
float setPointAngle = 178.0;

// PID controllers for pendulum and cart.
// Parameters: p,i,d, initial setpoint, update timestep [ms]
PIDControl penPid(0.75, 0.005, 150.0, setPointAngle, 10);
// PIDControl cartPid(2.0, 0.0005, 0, 0.0, 5);
PIDControl cartPid(3.0, 0.0005, 0, 0.0, 5);

elapsedMillis printTimer = 0;
unsigned int printerval = 50; // ms

float readTheta()
{
  // Read 12 bit measurement, compute offset, and convert to degrees (359/4095)
  int reading = (as5600.readTwoBytes(0x0d, 0x0c) + MAGNET_OFFSET) % 4096;
  return reading * 0.08766788766788766;
}

// Command callbacks for use by SimpleShell
ExecStatus setPID(CommandLine *cl)
{
  if (cl->argc != 3)
  {
    Serial.print("setPID: expected 3 args (kp, ki, kd), got ");
    Serial.println(cl->argc);
    return FAILED;
  }

  float pidGains[3] = {0};
  for (int i = 0; i < 3; i++)
  {
    Serial.print(" ");
    Serial.print(cl->argv[i]);

    // atof returns 0.0 on failure, which is a pretty decent fallback here
    pidGains[i] = atof(cl->argv[i]);

    // Gains should all be non-negative
    if (pidGains[i] < 0)
    {
      Serial.print("setPID: received negative gain coefficient: ");
      Serial.println(pidGains[i]);
      return FAILED;
    }
  }

  cartPid.setPID(pidGains[0], pidGains[1], pidGains[2]);

  return SUCCESS;
}

// Expected setpoint is a percentage from -100 (max left) to +100 (max right)
// The actual setpoint is in -1 to 1, but percentages are used for convenience.
ExecStatus setSetpoint(CommandLine *cl)
{
  if (cl->argc != 1)
  {
    Serial.print("setSetpoint: expected 1 arg, got ");
    Serial.println(cl->argc);
    return FAILED;
  }

  // atof returns 0.0 on failure (the center), which is a good fallback here
  float setpoint = atof(cl->argv[0]);

  if (setpoint < -100 || setpoint > +100)
  {
    Serial.print("setSetpoint: valid range is [-100,+100], got ");
    Serial.println(setpoint);
    return FAILED;
  }

  cartPid.setpoint = setpoint / 100;

#ifdef DEBUG
  deltaSetpoint = setpoint / 100 - cartPid.setpoint;
  riseTimer = 0;
  riseTimerStarted = true;
#endif

  return SUCCESS;
}

/**
 * Find limits using bump switches and center cart between them.
 */
ExecStatus reset(CommandLine *cl)
{
  elapsedMillis resetTimer = 0;
  unsigned long maxTime = 5000;

  digitalWrite(DIR_PIN, LEFT);
  analogWrite(PWM_PIN, 0.5 * maxPwm);

  while (true)
  {
    // TODO: this timeout block is repeated 3x - refactor
    if (resetTimer > maxTime)
    {
      Serial.println("Reset timeout");
      return FAILED;
    }
    if (digitalRead(LEFT_BUMP_PIN) == LOW)
    {
      trackEncoder.write(0);
      break;
    }
  }
  Serial.println("Found left limit.");

  resetTimer = 0;
  while (true)
  {
    if (resetTimer > maxTime)
    {
      Serial.println("Reset timeout");
      return FAILED;
    }
    if (digitalRead(RIGHT_BUMP_PIN) == LOW)
    {
      pendulum.xEncMax = trackEncoder.read() / 2;
      trackEncoder.write(pendulum.xEncMax);
      break;
    }
  }
  Serial.print("Found right limit. Limits = +/-");
  Serial.println(pendulum.xEncMax);

  resetTimer = 0;
  pendulum.update(trackEncoder.read(), readTheta(), millis());
  while (abs(pendulum.xEnc) > 2)
  {
    if (resetTimer > maxTime)
    {
      Serial.println("Reset timeout");
      return FAILED;
    }
    digitalWrite(DIR_PIN, pendulum.xCart > 0 ? LEFT : RIGHT);
    analogWrite(PWM_PIN, (pendulum.xCart + 0.15)*maxPwm);
    delay(10);
    pendulum.update(trackEncoder.read(), readTheta(), millis());
  }

  analogWrite(PWM_PIN, 0);

  Serial.print("Cart position: ");
  Serial.println(pendulum.xEnc);

  cartPid.setpoint = 0;

  return SUCCESS;
}

void printStatus(PIDControl &pid, Pendulum &pen)
{
  // Serial.println(i2c.readTwoBytes(0x0d, 0x0c) + MAGNET_OFFSET); // Direct sensor reading
  // Serial.print(",");
  Serial.print(pendulum.x);
  Serial.print(",");
  Serial.println(pendulum.y);
  // Serial.print(pendulum.theta);
  // Serial.print(",");
  // Serial.println(180 + 180*pendulum.omega); // Scale + offset for visibility with pendulum inverted

  // String s = String("{\"pid\":[") + pid.kp + "," + pid.ki + "," + pid.kd + "]"
  //            + String(", \"setpoint\":") + (100*pid.setpoint)
  //            + String(", \"output\":") + (100*pid.output)
  //            + String(", \"pwm\":") + dutyCycle
  //            + String(", \"x\":") + (100*pen.x)
  //            + String(", \"theta\":") + pen.theta
  //            + String(", \"omega\":") + pen.omega
  //            + String(", \"time\":") + pid.prevTime
  //            + "}";
  // Serial.println(s);
}

// string -> callback "dictionary"
Command cmdlist[] =
{
  {"pid", setPID},
  {"setpoint", setSetpoint},
  {"reset", reset},
  {NULL, NULL} // Required final entry
};

SimpleShell shell(cmdlist);

void handleCommands()
{
  if (Serial.available() > 0)
  {
    char input[256];
    memset(input, 0, 256);

    Serial.readBytesUntil('\r', input, 256);

    switch (shell.executeCommand(input))
    {
    case SUCCESS:
      break;
    case FAILED:
      Serial.print("FAILED: ");
      Serial.println(input);
      break;
    case NOTFOUND:
      Serial.print("NOTFOUND: ");
      Serial.println(input);
      break;
    }
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  // Motor control pins
  analogWriteResolution(14); // bits
  analogWriteFrequency(PWM_PIN, pwmFreq);
  analogWrite(PWM_PIN, dutyCycle / 100 * maxPwm);
  pinMode(DIR_PIN, OUTPUT);

  // Bump switch pins
  pinMode(LEFT_BUMP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_BUMP_PIN), onLeftBump, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_BUMP_PIN), onRightBump, FALLING);

  // Reset pushbutton
  pinMode(CART_RESET_PIN, INPUT_PULLUP);

  // Set output limits on PID controllers.
  cartPid.minOutput = -1; // Full speed left
  cartPid.maxOutput = +1; // Full speed right

  penPid.minOutput = -1; // Full speed left
  penPid.maxOutput = +1; // Full speed right


  Serial.begin(115200);

  // Don't continue until a terminal is connected. Useful for development.
  while (!Serial)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  Serial.println("Connecting to as5600...");
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Serial.println("Reading initial angle");
  Serial.println(readTheta());
  delay(100);

  char cmd[] = "reset";
  shell.executeCommand(cmd);

  Serial.println("Entering loop");
  delay(1000);
}

void loop()
{
  handleCommands();

  // Update pendulum state
  pendulum.update(trackEncoder.read(), readTheta(), millis());

  // Software endstops
  if (pendulum.xCart > +0.85 && pendulum.vCart > +0.001) onRightBump();
  if (pendulum.xCart < -0.85 && pendulum.vCart < -0.001) onLeftBump();

  // Compute PID output for x position
  cartPid.update(pendulum.xCart, millis());

  penPid.update(pendulum.theta, millis());

#ifdef DEBUG
  // Report rise time
  if (riseTimerStarted && deltaSetpoint != 0)
  {
    if (fabs((cartPid.setpoint - cartPid.prevInput) / deltaSetpoint) < 0.05)
    {
      Serial.print("rise time: ");
      Serial.println(riseTimer);
      riseTimerStarted = false;
    }
  }
#endif

  if (printTimer >= printerval)
  {
    printTimer -= printerval;
    printStatus(cartPid, pendulum);
  }

  float thetaError = penPid.setpoint - penPid.prevInput;

  // Compute the running mean absolute theta error
  meanAbsError = alpha * fabs(thetaError) + (1 - alpha) * meanAbsError;

  // if (meanAbsError < 0.5) Serial.println(penPid.prevInput);

  // Seems necessary for numerical stability (?)
  if (meanAbsError < 1e-5) meanAbsError = 0;

  // The "control" band
  if (fabs(thetaError) <= 10 && fabs(pendulum.omega) < 1)
  {
    // Set motor direction
    digitalWrite(DIR_PIN, penPid.output > 0 ? LEFT : RIGHT);

    // Set motor PWM value. Include output deadband to suppress jitter
    float percent_output = 100 * fabs(penPid.output);
    dutyCycle = percent_output < 0.0 ? 0 : percent_output;
    dutyCycle = PIDControl::clamped(dutyCycle, 0, 100); // Ensure within 0-100%
    analogWrite(PWM_PIN, dutyCycle / 100 * maxPwm);
  }

  // The "swing" band
  else
  {
    // If pendulum is not inverted, generate setpoint for swinging action
    float amplitude = 0.6 * fabs(sin(M_PI / 180 * pendulum.thetaHighPoint));
    cartPid.setpoint = pendulum.swingX(amplitude);

    // TODO refactor! ---------------------------------------------------------
    // Set motor direction
    digitalWrite(DIR_PIN, cartPid.output < 0 ? LEFT : RIGHT);

    // Set motor PWM value. Include a 1% output deadband to suppress jitter
    float percent_output = 100 * fabs(cartPid.output);
    dutyCycle = percent_output < 1 ? 0 : percent_output;
    dutyCycle = PIDControl::clamped(dutyCycle, 0, 100); // Ensure within 0-100%
    analogWrite(PWM_PIN, dutyCycle / 100 * maxPwm);
    // ------------------------------------------------------------------------
  }

}

// Bump switch ISRs
void onLeftBump() { digitalWrite(DIR_PIN, RIGHT); }
void onRightBump() { digitalWrite(DIR_PIN, LEFT); }
