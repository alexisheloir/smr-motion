#include "HordeResource.h"

void HordeResource::load(const char *name)
{
  m_handle=h3dAddResource(H3DResTypes::SceneGraph,name,0);
}

void HordeResource::free()
{
  int result = h3dRemoveResource(m_handle);
}


H3DRes HordeResource::getHandle()
{
  return m_handle;
}
