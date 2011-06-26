#ifndef HORDEAPPLICATION_H
#define HORDEAPPLICATION_H

#include <GL/glfw.h>

#include "Horde3D.h"
#include "Horde3DUtils.h"
#include "HordeCamera.h"
#include "HordeResource.h"
#include "HordeNode.h"

#include <sstream>
#include <math.h>
#include <iomanip>
#include <iostream>


using namespace std;

/**
* \class HordeApplication
* \brief Tutorial class.
**/
class HordeApplication
{
protected:
  bool m_keys[320];
  float _x, _y, _z, _rx, _ry;  // Viewer position and orientation
	float _velocity;  // Velocity for movement
  H3DNode m_camera;
  float m_curFPS,m_timer,m_time;
  stringstream m_fpsText;

  bool m_freeze,m_showFPS,m_debugViewMode,m_wireframeMode;
  
  //Engine objects
  H3DRes m_fontMatRes,m_forwardPipeRes,m_skyBoxRes,m_envRes;

  HordeResource m_frameGizmoRes;
  HordeNode m_frameGizmoNode;
    
  string _contentDir;

public:
  HordeApplication(const std::string &appPath );

  //Instanciate objects and set up rendering engine
  virtual bool init();

  //Draws the scene
  virtual void mainLoop(float fps);

  //free memory before closing
  virtual void release();

  //change windows size
  virtual void resize(int width,int height);

  //What to do when a key is pressed
  virtual void onKeyPress(int key);

  //update the state of one key
  virtual void onKeyStateChange(int key,bool state);

  //what to do when the mouse was moved
  virtual void mouseMoveEvent(float dX,float dY);

  virtual void intermediateDraw();

  virtual void onLoadResources();
  virtual void onAddNodes();
  //virtual void onInit();
  virtual void keyHandler();
  virtual void onUpdate();
    
  std::string extractAppPath( const std::string &fullPath );  
};

#endif

