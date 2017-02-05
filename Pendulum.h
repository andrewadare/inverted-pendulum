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
  float x;
  float v;
  float theta;
  float swingThetaMax;
  float omega;
  unsigned long dt;
  float height;
  float highPoint;
  float thetaHighPoint;
};

#endif // __PENDULUM_H__