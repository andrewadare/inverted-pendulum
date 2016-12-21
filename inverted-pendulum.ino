// Inverted pendulum - for Teensy 3.2

#include "Encoder.h"
#include "PIDControl.h"
#include "SimpleShell.h"

#define LEFT HIGH
#define RIGHT LOW
#define TRACK_ENCODER_PIN_A 0
#define TRACK_ENCODER_PIN_B 1
#define PENDULUM_ENCODER_PIN_A 2
#define PENDULUM_ENCODER_PIN_B 3
#define LEFT_BUMP_PIN 4
#define RIGHT_BUMP_PIN 5
#define PWM_PIN 10
#define DIR_PIN 12
#define CART_RESET_PIN 21

// float prevSetpoint = 0; // temp - debug

// For incoming serial data
int incomingByte = 0;
String cmd = "";

int xPrev = 0;
int xCart = 0;
int xMax = 0;

int thetaPrev = 0;
int theta = 0;

float maxPwm = 16383;  // 100% PWM value with 14-bit resolution
float pwmFreq = 2929.687; // See http://www.pjrc.com/teensy/td_pulse.html
float dutyCycle = 0;  // Percent

Encoder trackEncoder(TRACK_ENCODER_PIN_A, TRACK_ENCODER_PIN_B);
Encoder thetaEncoder(PENDULUM_ENCODER_PIN_A, PENDULUM_ENCODER_PIN_B);

// PID control for cart position
float kp = 2, ki = 10, kd = 0; // Initial/default PID gain coeffs
unsigned long timeStep = 20; // ms
PIDControl cartPid(kp, ki, kd, 0, timeStep);
elapsedMillis pidTimer = 0;

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
  for (int i=0; i<3; i++)
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

  Serial.println();
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

  cartPid.setpoint = setpoint/100;

  Serial.println();
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
  analogWrite(PWM_PIN, 0.3*maxPwm);

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
      xMax = trackEncoder.read()/2;
      trackEncoder.write(xMax);
      break;
    }
  }
  Serial.print("Found right limit.\r\nLimits = +/-");
  Serial.println(xMax);

  resetTimer = 0;
  xCart = trackEncoder.read();
  while (abs(xCart) > 2)
  {
    if (resetTimer > maxTime)
    {
      Serial.println("Reset timeout");
      return FAILED;
    }
    digitalWrite(DIR_PIN, xCart > 0 ? LEFT : RIGHT);
    analogWrite(PWM_PIN, (float(xCart)/xMax + 0.15)*maxPwm);
    delay(10);
    xCart = trackEncoder.read();
  }

  analogWrite(PWM_PIN, 0);

  Serial.print("Cart position: ");
  Serial.println(xCart);

  cartPid.setpoint = 0;

  return SUCCESS;
}

void printStatus(PIDControl &pid)
{
  String s = String("{\"pid\":[") + pid.kp + "," + pid.ki + "," + pid.kd + "]"
             + String(", \"setpoint\":") + (100*pid.setpoint)
             + String(", \"output\":") + pid.output
             + String(", \"pwm\":") + dutyCycle
             + String(", \"x\":") + (100*float(trackEncoder.read())/xMax)
             + String(", \"theta\":") + thetaEncoder.read()
             + String(", \"time\":") + millis()
             + "}";
  Serial.println(s);
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
  // Motor control pins
  analogWriteResolution(14); // bits
  analogWriteFrequency(PWM_PIN, pwmFreq);
  analogWrite(PWM_PIN, dutyCycle/100*maxPwm);
  pinMode(DIR_PIN, OUTPUT);

  // Bump switch pins
  pinMode(LEFT_BUMP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_BUMP_PIN), onLeftBump, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_BUMP_PIN), onRightBump, FALLING);

  // Reset pushbutton
  pinMode(CART_RESET_PIN, INPUT_PULLUP);

  // Set output limits on cart PID controller.
  cartPid.minOutput = -1; // Full speed left
  cartPid.maxOutput = +1; // Full speed right

  Serial.begin(115200);

  // Don't continue until a terminal is connected. Useful for development.
  while (!Serial) {;}

  char cmd[] = "reset";
  shell.executeCommand(cmd);
}

void loop()
{
  handleCommands();

  if (pidTimer >= timeStep)
  {
    // if (prevSetpoint != cartPid.setpoint) { printStatus(cartPid); }
    pidTimer -= timeStep;
    float input = float(trackEncoder.read())/xMax;
    cartPid.update(input, 0.001);
    digitalWrite(DIR_PIN, cartPid.output < 0 ? LEFT : RIGHT);
    analogWrite(PWM_PIN, (fabs(cartPid.output) + 0.1)*maxPwm);
    // prevSetpoint = cartPid.setpoint;

    printStatus(cartPid);
  }
}

// Bump switch ISRs
void onLeftBump() { digitalWrite(DIR_PIN, RIGHT); }
void onRightBump() { digitalWrite(DIR_PIN, LEFT); }
