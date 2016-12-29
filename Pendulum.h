#ifndef __PENDULUM_H__
#define __PENDULUM_H__

class Pendulum
{
public:
  Pendulum(int thetaEncoderMax, float swingMaxDegrees);
  ~Pendulum() {}

  void update(const int xEncoder, const int thetaEncoder);

  float swingX(const float amplitude);

  int xEnc;
  int xEncMax;
  float x;
  int thetaEnc;
  int thetaEncMax;
  float theta;
  int thetaEncPrev;
  float swingThetaMax;
  float omega;
};

#endif // __PENDULUM_H__