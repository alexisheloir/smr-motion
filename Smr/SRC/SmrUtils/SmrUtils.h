/// \defgroup SmrUtils Smr utility classes and routines

/// \file SmrUtils.h
/// \brief Definition files for Smr Utils library.

#ifndef SMRUTILS_H
#define SMRUTILS_H

// Core
#include "SmrCore.h"

//#include "HordeCamera.h"
//#include "HordeWindow.h"
//#include "HordeApplication.h"
//#include "HordeResource.h"
//#include "HordeNode.h"
//#include "HordeBones.h"

#ifdef WIN32
  #include <windows.h>
#else
  #include <sys/time.h>
#endif

/**
* \class SMRUtils
* \brief Tutorial class.
**/
class SMRUtils
{
public:
  inline static float degToRad(float f) 
  {
    //Convert from degrees to radians
    return f*((float)M_PI/180.0f);
  }

  inline static float radToDeg(float f) 
  {
    //Convert from degrees to radians
    return f*(180.0f/(float)M_PI);
  }

  inline static float ranf()
  {
    //return a random number between -1 and +1 
    float w =(static_cast<float>(rand()%10000)-5000.0f);
    if(w>0) return(sqrt(w)/100.0f);
    else return -(sqrt(-w)/100.0f);
  }
  inline static unsigned int getCurrentTime()
  { 
#ifdef WIN32
    return clock();
#else
    timeval time;
    gettimeofday(&time,NULL);
    unsigned int currtime = (unsigned int) (time.tv_sec * 1000 + time.tv_usec / 1000);
    return currtime; 
#endif
  }
};

#endif

