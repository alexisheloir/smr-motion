#include "HordeNode.h"

///////////////////
///////////////////
///////////////////

void HordeNode::create(HordeResource &resource, const H3DNode &parentNode)
{
  m_handle=h3dAddNodes(parentNode,resource.getHandle());
}

void HordeNode::create(HordeResource &resource, std::string name, const H3DNode &parentNode)
{
  m_handle=h3dAddNodes(parentNode,resource.getHandle());
  //m_boneMeshHandle=h3dAddNodes(parentNode,resource.getHandle());
}

void HordeNode::removeNode()
{
  h3dRemoveNode(m_handle);
}

void HordeNode::create(std::string name, const H3DNode &parentNode)
{
  //m_handle=h3dAddNodes(parentNode,0);
  m_handle=h3dAddGroupNode(parentNode,name.c_str());
  //m_boneMeshHandle=h3dAddNodes(parentNode,resource.getHandle());
}

void HordeNode::setTransform(float tx,float ty,float tz,float rx,float ry,float rz,float sx,float sy,float sz)
{
  h3dSetNodeTransform(m_handle,tx,ty,tz,rx,ry,rz,sx,sy,sz);
}

void HordeNode::setTransform(float tx,float ty,float tz,float rx,float ry,float rz)
{
  float sx,sy,sz,foo;
  getTransform(&foo,&foo,&foo,&foo,&foo,&foo,&sx,&sy,&sz);
  h3dSetNodeTransform(m_handle,tx,ty,tz,rx,ry,rz,sx,sy,sz);
}

void HordeNode::getTransform(float *tx,float *ty,float *tz,float *rx,float *ry,float *rz,float *sx,float *sy,float *sz)
{
  h3dGetNodeTransform(m_handle,tx,ty,tz,rx,ry,rz,sx,sy,sz);
}

void HordeNode::getTransform(float *tx,float *ty,float *tz,float *rx,float *ry,float *rz)
{
  float foo;
  h3dGetNodeTransform(m_handle,tx,ty,tz,rx,ry,rz,&foo,&foo,&foo);
}

void HordeNode::setPosition(float tx,float ty,float tz)
{
  float rx,ry,rz,sx,sy,sz,foo;
  getTransform(&foo,&foo,&foo,&rx,&ry,&rz,&sx,&sy,&sz);
  setTransform(tx,ty,tz,rx,ry,rz,sx,sy,sz);
}

void HordeNode::setScale(float sx,float sy,float sz)
{
  float rx,ry,rz,tx,ty,tz,foo;
  H3DNode child = 0;
  unsigned int i=0;
  do
  {
    child = h3dGetNodeChild(m_handle,i);
    if (child > 0 && h3dGetNodeType(child) == H3DNodeTypes::Mesh)
    {
      h3dGetNodeTransform(child,&tx,&ty,&tz,&rx,&ry,&rz,&foo,&foo,&foo);
      h3dSetNodeTransform(child,tx,ty,tz,rx,ry,rz,sx,sy,sz);
    }
    i++;
  }while(child > 0);
}


void HordeNode::getPosition(float *tx,float *ty,float *tz)
{
  float rx,ry,rz,sx,sy,sz;
  getTransform(tx,ty,tz,&rx,&ry,&rz,&sx,&sy,&sz);
}

HordeNode HordeNode::getFirstChild()
{
  HordeNode result;
  result.m_handle=h3dGetNodeChild(m_handle,0);
  return result;
}

bool HordeNode::isValid()
{
  return m_handle>0;
}

const char *HordeNode::getName()
{
  return h3dGetNodeParamStr(m_handle, H3DNodeParams::NameStr);
}

void HordeNode::findJoint(const char *name)
{
  h3dFindNodes(H3DRootNode,name,H3DNodeTypes::Joint);
  m_handle=h3dGetNodeFindResult(0);
}
