#ifndef HORDERESOURCE_H
#define HORDERESOURCE_H

#include "Horde3D.h"
#include "Horde3DUtils.h"

/**
* \class HordeResource
* \brief Tutorial class.
**/
class HordeResource
{
private:
  H3DRes m_handle;
public:
  void load(const char *name);
  void free();
  H3DRes getHandle();
};

#endif
