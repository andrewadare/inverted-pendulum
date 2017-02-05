#include <cmath>
#include "Pendulum.h"

Pendulum::Pendulum(unsigned long updateInterval, float swingMaxDegrees) :
  xEnc(0),
  xEncMax(0),
  x(0),
  v(0),
  theta(0),
  swingThetaMax(swingMaxDegrees),
  omega(0),
  dt(updateInterval),
  height(0),
  highPoint(0),
  thetaHighPoint(0)
{
}

void Pendulum::update(const int xEncoder, const float thetaDeg, unsigned long currentTime)
{
  static unsigned long tPrev = 0;
  static float xPrev = 0, thetaPrev = 0, omegaPrev = 0;

  // Return early if it's too soon for an update.
  if (currentTime - tPrev < dt)
    return;

  // Cart position and velocity
  xEnc = xEncoder;
  x = xEncMax != 0 ? (float)xEncoder/xEncMax : x;
  v = x - xPrev;

  // Pendulum angle and instantaneous angular velocity
  theta = thetaDeg;
  float omegaInst = theta - thetaPrev;

  // Handle rollovers due to periodicity
  if (omegaInst < -180) omegaInst += 360;
  if (omegaInst > +180) omegaInst -= 360;

  // The AS5600 magnetic sensor produces severe random spikes. The angular
  // velocity should always be smooth, so if a large difference from the moving
  // average is measured, use an estimate instead of the bad measurement.
  if (fabs(omegaInst - omega) > 2)
    theta = thetaPrev + omega;
  else
    omega = (1 - 0.95)*omegaInst + 0.95*omega;

  float thetaRad = M_PI/180 * (theta > 180 ? 360 - theta : theta);

  height = 1 - cos(thetaRad);

  // Note: if trouble here, add a prevHighPoint variable and check it
  if ((omegaPrev >= 0 && omega < 0) || (omegaPrev <= 0 && omega > 0))
  {
    highPoint = height;
    thetaHighPoint = theta;
  }

  // Store for next call
  tPrev = currentTime;
  xPrev = x;
  thetaPrev = theta;
  omegaPrev = omegaInst;
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
