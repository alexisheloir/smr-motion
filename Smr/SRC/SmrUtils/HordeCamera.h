#ifndef HORDECAMERA_H
#define HORDECAMERA_H

#include "Horde3D.h"
#include "Horde3DUtils.h"


/**
* \class HordeCamera
* \brief Tutorial class.
**/
class HordeCamera
{
private:
  float m_posX,m_posY,m_posZ;
  float m_rotX,m_rotY;
  float m_velocity;
  H3DNode m_node;

public:
  HordeCamera();
  HordeCamera(float posX,float posY,float posZ,float rotX,float rotY,float velocity);
  void add(H3DRes pipelineRes);
  H3DNode getNodeHandle();
  void set(float posX,float posY,float posZ,float rotX,float rotY,float velocity);
  void setTransform();
  void setParameters(float _fieldOfView,int _width,int _height,float _near,float _far);
  void moveForward(float deltaTime);
  void moveBackward(float deltaTime);
  void strafeLeft(float deltaTime);
  void strafeRight(float deltaTime);
  void rotate(float dx,float dy);
};

#endif
