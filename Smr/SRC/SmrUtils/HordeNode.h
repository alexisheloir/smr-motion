#ifndef HORDENODE_H
#define HORDENODE_H

#include "Horde3D.h"
#include "Horde3DUtils.h"
#include "HordeResource.h"
#include "SmrMath.h"

#include <string>

/**
* \class HordeNode
* \brief Tutorial class.
**/
class HordeNode
{
private:
  H3DNode m_handle;
  //H3DNode m_boneMeshHandle;

public:
  void create(HordeResource &resource, const H3DNode &parentNode = H3DRootNode);
  void create(HordeResource &resource, std::string name, const H3DNode &parentNode = H3DRootNode);
  void removeNode();
  void create(std::string name, const H3DNode &parentNode = H3DRootNode);
  void setTransform(float tx,float ty,float tz,float rx,float ry,float rz,float sx,float sy,float sz);
  void setTransform(float tx,float ty,float tz,float rx,float ry,float rz);
//  void setTransform(float tx,float ty,float tz,SMRQuaternion rotation);
//  void setTransform(float tx,float ty,float tz,SMRQuaternion rotation,float sx,float sy,float sz);
  void getTransform(float *tx,float *ty,float *tz,float *rx,float *ry,float *rz,float *sx,float *sy,float *sz);
  void getTransform(float *tx,float *ty,float *tz,float *rx,float *ry,float *rz);
  void setPosition(float tx,float ty,float tz);
  void getPosition(float *tx,float *ty,float *tz);
  void setScale(float sx,float sy,float sz);

  HordeNode getFirstChild();
  bool isValid();
  const char *getName();
  void findJoint(const char *name);
  inline const H3DNode getHandle(){return m_handle;}
};

#endif

