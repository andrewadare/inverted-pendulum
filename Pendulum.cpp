#include <cmath>
#include "Pendulum.h"

Pendulum::Pendulum(int thetaEncoderMax, float swingMaxDegrees) :
xEnc(0),
xEncMax(0),
x(0),
thetaEnc(0),
thetaEncMax(thetaEncoderMax),
theta(0),
thetaPrev(0),
swingThetaMax(swingMaxDegrees),
omega(0)
{
}

void Pendulum::setX(const int xEncoder)
{
  xEnc = xEncoder;
  x = xEncMax != 0 ? (float)xEncoder/xEncMax : x;
}

float Pendulum::swingX(const float amplitude)
{
  if (theta > 0 && theta < swingThetaMax && omega < 0)
    return amplitude*cos(theta/swingThetaMax*M_PI);
  else if (theta < 0 && theta > - swingThetaMax && omega > 0)
    return -amplitude*cos(theta/swingThetaMax*M_PI);
  else
    return x;
}
