#ifndef HORDEBONES_H
#define HORDEBONES_H

#include "Horde3D.h"
#include "Horde3DUtils.h"

#include "HordeNode.h"
#include "SmrCore.h"

class HordeBone
{
public:
  HordeNode node;
  int jointIndex;
  string name;
  std::vector<HordeNode*> childNodes;
public:
  HordeBone(){;}
  ~HordeBone()
  {
    node.removeNode();
    childNodes.clear();
  }
};

/**
* \class HordeBones
* \brief Tutorial class.
**/
class HordeBones
{
private:
  std::vector<HordeBone*> m_bones;
  HordeResource m_boneRes;
  SMRKinematicChain *m_chain;
public:
  HordeBones();
  ~HordeBones();
  void loadResource();
  void freeResource();
  void create(SMRKinematicChain *chain, float scale);
  void create(SMRSkeleton *skeleton, float scale);
  void update();
  void update(SMRSkeleton *skeleton);
  void clear();
};

#endif
