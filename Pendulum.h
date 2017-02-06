#ifndef __PENDULUM_H__
#define __PENDULUM_H__

class Pendulum
{
public:
  Pendulum(unsigned long updateInterval, float swingMaxDegrees);
  ~Pendulum() {}

  void update(const int xEncoder, const float thetaDeg, unsigned long time);

  float swingX(const float amplitude);

  int xEnc;
  int xEncMax;
  float xCart;
  float vCart;
  float x;
  float y;
  float vx;
  float vy;
  float theta;
  float omega;
  float swingThetaMax;
  unsigned long dt;
  float thetaHighPoint;
};

#endif // __PENDULUM_H__