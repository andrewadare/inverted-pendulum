#include <cmath>
#include "Pendulum.h"

Pendulum::Pendulum(int thetaEncoderMax, float swingMaxDegrees) :
xEnc(0),
xEncMax(0),
x(0),
v(0),
thetaEnc(0),
thetaEncMax(thetaEncoderMax),
theta(0),
thetaEncPrev(0),
swingThetaMax(swingMaxDegrees),
omega(0),
dt(10),
height(0),
highPoint(0),
thetaHighPoint(0)
{
}

void Pendulum::update(const int xEncoder, const int thetaEncoder, unsigned long currentTime)
{
  static unsigned long tPrev = 0;
  static float xPrev = 0;
  static float omegaPrev = 0;
  static float alpha = 0.3; // Smoothing parameter for EWMA

  // Return early if it's too soon for an update.
  if (currentTime - tPrev < dt)
  {
    return;
  }

  xEnc = xEncoder;
  x = xEncMax != 0 ? (float)xEncoder/xEncMax : x;

  thetaEnc = thetaEncoder;

  theta = fmod(thetaEnc*360.0/thetaEncMax, 360.0);

  while (theta < 0) theta += 360;

  float thetaRad = M_PI/180 * (theta > 180 ? 360 - theta : theta);

  height = 1 - cos(thetaRad);

  // velocities
  v = x - xPrev;
  omega = alpha*(thetaEnc - thetaEncPrev)*360.0/thetaEncMax + (1 - alpha)*omegaPrev;
  // omega = (thetaEnc - thetaEncPrev)*360.0/thetaEncMax;

  if ((omegaPrev >= 0 && omega < 0) || (omegaPrev <= 0 && omega > 0))
  {
    highPoint = height;
    thetaHighPoint = theta;
  }

  // For some reason, small values were crashing the MCU.
  // Forcing an underflow at a high threshold seems to help.
  if (fabs(omega) < 1e-5) omega = 0;

  // Store for next call
  omegaPrev = omega;
  tPrev = currentTime;
  thetaEncPrev = thetaEnc;
  xPrev = x;
}

float Pendulum::swingX(const float amplitude)
{
  // left, swinging right
  if (theta < swingThetaMax && omega < 0)
    return amplitude*cos(theta/swingThetaMax*M_PI);

  // right, swinging left
  else if (theta > 360 - swingThetaMax && omega > 0)
    return -amplitude*cos((360 - theta)/swingThetaMax*M_PI);

  else
    return x;
}
