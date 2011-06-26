#include "HordeCamera.h"
#include "SmrUtils.h"

HordeCamera::HordeCamera()
{
  m_posX=0; m_posY=0; m_posZ=0;
  m_rotX=0; m_rotY=0;
  m_velocity=0;
  m_node=0;
}

HordeCamera::HordeCamera(float posX,float posY,float posZ,float rotX,float rotY,float velocity)
{
  m_posX=posX; m_posY=posY; m_posZ=posZ;
  m_rotX=rotX; m_rotY=rotY;
  m_velocity=velocity;
  m_node=0;
}

void HordeCamera::set(float posX,float posY,float posZ,float rotX,float rotY,float velocity)
{
  m_posX=posX; m_posY=posY; m_posZ=posZ;
  m_rotX=rotX; m_rotY=rotY;
  m_velocity=velocity;
}

void HordeCamera::add(H3DRes pipelineRes)
{
  m_node=h3dAddCameraNode(H3DRootNode,"Camera",pipelineRes);
}

H3DNode HordeCamera::getNodeHandle()
{
  return m_node;
}

void HordeCamera::setTransform()
{
  h3dSetNodeTransform(m_node,m_posX,m_posY,m_posZ,m_rotX,m_rotY,0,1,1,1);
}

void HordeCamera::setParameters(float _fieldOfView,int _width,int _height,float _near,float _far)
{
  float aspect = static_cast<float>(_width)/static_cast<float>(_height);
  h3dSetupCameraView(m_node,_fieldOfView,aspect,_near,_far);
}

void HordeCamera::moveForward(float deltaTime)
{
  deltaTime*=m_velocity;
  m_posX-=sinf(SMRUtils::degToRad(m_rotY))*cosf(-SMRUtils::degToRad(m_rotX))*deltaTime;
  m_posY-=sinf(-SMRUtils::degToRad(m_rotX))*deltaTime;
  m_posZ-=cosf(SMRUtils::degToRad(m_rotY))*cosf(-SMRUtils::degToRad(m_rotX))*deltaTime;
}

void HordeCamera::moveBackward(float deltaTime)
{
  deltaTime*=m_velocity;
  m_posX+=sinf(SMRUtils::degToRad(m_rotY))*cosf(-SMRUtils::degToRad(m_rotX))*deltaTime;
  m_posY+=sinf(-SMRUtils::degToRad(m_rotX))*deltaTime;
  m_posZ+=cosf(SMRUtils::degToRad(m_rotY))*cosf(-SMRUtils::degToRad(m_rotX))*deltaTime;
}

void HordeCamera::strafeLeft(float deltaTime)
{
  deltaTime*=m_velocity;
  m_posX+=sinf(SMRUtils::degToRad(m_rotY-90))*deltaTime;
  m_posZ+=cosf(SMRUtils::degToRad(m_rotY-90))*deltaTime;
}

void HordeCamera::strafeRight(float deltaTime)
{
  deltaTime*=m_velocity;
  m_posX+=sinf(SMRUtils::degToRad(m_rotY+90))*deltaTime;
  m_posZ+=cosf(SMRUtils::degToRad(m_rotY+90))*deltaTime;
}

void HordeCamera::rotate(float dx,float dy)
{
  m_rotY+=dy;
  //Loop up/down but only in a limited range
  m_rotX+=dx;
  if(m_rotX>90) m_rotX=90;
  if(m_rotX<-90) m_rotX=-90;
}
