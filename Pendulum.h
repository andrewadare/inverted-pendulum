#ifndef __PENDULUM_H__
#define __PENDULUM_H__

class Pendulum
{
public:
  Pendulum(int thetaEncoderMax, float swingMaxDegrees);
  ~Pendulum() {}

  void update(const int xEncoder, const int thetaEncoder, unsigned long time);

  float swingX(const float amplitude);

  int xEnc;
  int xEncMax;
  float x;
  float v;
  int thetaEnc;
  int thetaEncMax;
  float theta;
  int thetaEncPrev;
  float swingThetaMax;
  float omega;
  unsigned long dt;
};

#endif // __PENDULUM_H__