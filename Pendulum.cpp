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
dt(10)
{
}

void Pendulum::update(const int xEncoder, const int thetaEncoder, unsigned long currentTime)
{
  static unsigned long prevTime = 0;
  static float xPrev = 0;

  // Return early if it's too soon for an update.
  if (currentTime - prevTime < dt)
  {
    return;
  }

  xEnc = xEncoder;
  x = xEncMax != 0 ? (float)xEncoder/xEncMax : x;

  thetaEnc = thetaEncoder;

  theta = fmod(thetaEnc*360.0/thetaEncMax, 360.0);

  while (theta < 0) theta += 360;

  // velocities
  v = x - xPrev;
  omega = (thetaEnc - thetaEncPrev)*360.0/thetaEncMax;

  // Store for next call
  prevTime = currentTime;
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
