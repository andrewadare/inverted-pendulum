#ifndef __PENDULUM_H__
#define __PENDULUM_H__

class Pendulum
{
public:
  Pendulum(int thetaEncoderMax, float swingMaxDegrees);
  ~Pendulum() {}

  void setX(const int xEncoder);

  float swingX(const float amplitude);

  int xEnc;
  int xEncMax;
  float x;
  int thetaEnc;
  int thetaEncMax;
  float theta;
  float thetaPrev;
  float swingThetaMax;
  float omega;
};

#endif // __PENDULUM_H__