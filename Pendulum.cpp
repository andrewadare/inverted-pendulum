#include <cmath>
#include "Pendulum.h"

Pendulum::Pendulum(unsigned long updateInterval, float swingMaxDegrees) :
  xEnc(0),
  xEncMax(0),
  xCart(0),
  vCart(0),
  x(0),
  y(0),
  vx(0),
  vy(0),
  theta(0),
  omega(0),
  swingThetaMax(swingMaxDegrees),
  dt(updateInterval),
  thetaHighPoint(0)
{
}

void Pendulum::update(const int xEncoder, const float thetaDeg, unsigned long currentTime)
{
  static unsigned long tPrev = 0;
  static float xCartPrev = 0, thetaPrev = 0, omegaPrev = 0, xPrev = 0, yPrev = 0, vyPrev = 0;

  // Return early if it's too soon for an update.
  if (currentTime - tPrev < dt)
    return;

  // Cart position and velocity
  xEnc = xEncoder;
  xCart = xEncMax != 0 ? (float)xEncoder/xEncMax : xCart;
  vCart = xCart - xCartPrev;

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

  // Coordinates of the "bob" w.r.t. the pivot in units of pendulum rod length.
  // Note that x is chosen to be positive right / negative left
  x = -sin(theta*M_PI/180);
  y = 1 - cos(theta*M_PI/180);
  vx = x - xPrev;
  vy = y - yPrev;

  // Note: if trouble here, add a prevHighPoint variable and check it
  // if ((x < 0 && omega > 0 && y < yPrev ) || (omegaPrev <= 0 && omega > 0))
  if (vyPrev >= 0 && vy < 0)
    thetaHighPoint = theta;

  // Store for next call
  tPrev = currentTime;
  xCartPrev = xCart;
  thetaPrev = theta;
  omegaPrev = omegaInst;
  xPrev = x;
  yPrev = y;
  vyPrev = vy;
}

float Pendulum::swingX(const float amplitude)
{
  // left, swinging right
  if (theta < swingThetaMax && vy < 0)
    return amplitude*cos(theta/swingThetaMax*M_PI);

  // right, swinging left
  else if (theta > 360 - swingThetaMax && vy < 0)
    return -amplitude*cos((360 - theta)/swingThetaMax*M_PI);

  else
    return xCart;
}
