#include "Smr.h"

int main (int argc, char** argv)
{

  Smr::initSmr(false);
  LOG_INFO(logger, "testing quaternions");
  LOG_INFO(logger, "instantiating three quaternions");
  SMRQuaternion firstQuaternion(SMRVector3(1.0,0.0,0.0),rad(15.0));
  SMRQuaternion secondQuaternion(SMRVector3(0.0,1.0,0.0),rad(25.0));
  SMRQuaternion thirdQuaternion(SMRVector3(0.0,0.0,1.0),rad(35.0));

  LOG_INFO(logger, "combining quaternions, XYZ");
  SMRQuaternion xyz = firstQuaternion * secondQuaternion * thirdQuaternion;

  LOG_INFO(logger, "resulting quaternion" << xyz << " has a norm of " << xyz.norm());
  LOG_INFO(logger, "derivating quaternion");

  double x=0;
  double y=0;
  double z=0;

  xyz.toEulerAngles(x,y,z);
  LOG_INFO(logger, "here are the results: X=" << deg(x) << " Y=" << deg(y) << " Z=" << deg(z));

  LOG_INFO(logger, "combining quaternions, ZYX");
  SMRQuaternion zyx = thirdQuaternion * secondQuaternion * firstQuaternion;

  LOG_INFO(logger, "resulting quaternion" << zyx << " has a norm of " << xyz.norm());
  LOG_INFO(logger, "derivating quaternion");

  x=0;
  y=0;
  z=0;

  zyx.toEulerAnglesZYX(x,y,z);
  LOG_INFO(logger, "here are the results: X=" << deg(x) << " Y=" << deg(y) << " Z=" << deg(z));

  LOG_INFO(logger, "combining quaternions, YXZ");
  SMRQuaternion yxz = secondQuaternion * firstQuaternion * thirdQuaternion;

  LOG_INFO(logger, "resulting quaternion" << yxz << " has a norm of " << xyz.norm());
  LOG_INFO(logger, "derivating quaternion");

  x=0;
  y=0;
  z=0;

  yxz.toEulerAnglesYXZ(x,y,z);
  LOG_INFO(logger, "here are the results: X=" << deg(x) << " Y=" << deg(y) << " Z=" << deg(z));

  exit(0);

}