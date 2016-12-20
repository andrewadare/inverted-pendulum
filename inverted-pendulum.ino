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

// Command callbacks for use by SimpleShell
ExecStatus setPID(CommandLine *clp)
{
  Serial.print("setPID ");
  for (int i=0; i<clp->argc; i++)
  {
    Serial.print(" ");
    Serial.print(clp->argv[i]);
    // TODO: add cartPid.setPID(kp, ki, kd);
  }
  Serial.println();
  return SUCCESS;
}

ExecStatus setSetpoint(CommandLine *clp)
{
  Serial.print("setSetpoint ");
  for (int i=0; i<clp->argc; i++)
  {
    Serial.print(" ");
    Serial.print(clp->argv[i]);
    // TODO: add something like this:
    // float setpoint = cmd.toFloat()/xMax;
    // cartPid.setpoint = constrain(setpoint, cartPid.minOutput, cartPid.maxOutput);
  }
  Serial.println();
  return SUCCESS;
}

float prevSetpoint = 0; // temp - debug

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

unsigned long timeStep = 15; // ms

Encoder trackEncoder(TRACK_ENCODER_PIN_A, TRACK_ENCODER_PIN_B);
Encoder thetaEncoder(PENDULUM_ENCODER_PIN_A, PENDULUM_ENCODER_PIN_B);

// PID control for cart position
float kp = 2, ki = 10, kd = 0;
elapsedMillis pidTimer = 0;
PIDControl cartPid(kp, ki, kd, 0, timeStep);

/**
 * Find limits using bump switches and center cart between them.
 */
ExecStatus reset(CommandLine *clp)
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
  String s = String("kp,ki,kd: ") + kp + ", " + ki + ", " + kd
             + String("; p,i,d: ") + pid.kp + ", " + pid.ki + ", " + pid.kd
             + String(" setpoint: ") + pid.setpoint
             + String(" output: ") + pid.output
             + String(" pwm: ") + dutyCycle;
  Serial.println(s);
}

void handleByte(byte b)
{
  Serial.print((char)b);

  if (b == '-')
    cmd = "-";

  // kp
  else if (b == 'p') // increase kp
    kp += 0.01;
  else if (b == 'l') // decrease kp
    kp -= 0.01;

  // ki
  else if (b == 'i') // increase ki
    ki += 0.1;
  else if (b == 'k') // decrease ki
    ki -= 0.1;

  // kd
  else if (b == 'd') // increase kd
    kd += 0.001;
  else if (b == 'c') // decrease kd
    kd -= 0.001;

  else if (b == '.')
    cmd += b;
  else if (isDigit(b))
  {
    byte digit = b - 48;
    cmd += digit;
  }
  else if (b == '\r' || b == '\n')
  {
    // Assume here that cmd is a signed track x setpoint value in encoder units.
    // Convert to a fraction in [-1,1] for PID input
    float setpoint = cmd.toFloat()/xMax;
    cartPid.setpoint = constrain(setpoint, cartPid.minOutput, cartPid.maxOutput);
    cmd = "";
  }
  else
  {
    cartPid.setPID(kp, ki, kd);
    Serial.println();
    printStatus(cartPid);
  }
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
    if (prevSetpoint != cartPid.setpoint) printStatus(cartPid);
    pidTimer -= timeStep;
    float input = float(trackEncoder.read())/xMax;
    cartPid.update(input, 0.001);
    digitalWrite(DIR_PIN, cartPid.output < 0 ? LEFT : RIGHT);
    analogWrite(PWM_PIN, (fabs(cartPid.output) + 0.1)*maxPwm);
    prevSetpoint = cartPid.setpoint;
  }
}

// Bump switch ISRs
void onLeftBump() { digitalWrite(DIR_PIN, RIGHT); }
void onRightBump() { digitalWrite(DIR_PIN, LEFT); }
