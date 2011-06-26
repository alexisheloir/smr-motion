/**
* \ingroup Examples
* \file Tentacle.cpp
* \brief This program is a simple introduction how to use simple inverse kinematics and Horde3D next gen rendering engine 
*  with Smr
*/
#ifndef _app_H_
#define _app_H_

#include "Smr.h"
#include "SmrGSMMSolver.h"
#include "SmrInvJacSolver.h"
//#include "SmrCCDSolver.h"


#include <GL/glfw.h>
#include "Horde3D.h"
#include "Horde3DUtils.h"

#include <sstream>
#include <math.h>
#include <iomanip>

using namespace std;

static SMRIKConstraint * _ikConstraint;
//static SmrCCDSolver * _ikCCDSolver;
static SMRGSMMSolver * _ikGSMMSolver;
static SMRInvJacSolver * _ikInvJacSolver;
static SMRIKSolver	* _currentSolver;
static SMRKinematicChain* _tentacleIK;

class TentacleApplication
{
private:

  bool _keys[320];
  float _x, _y, _z, _rx, _ry;	// Viewer position and orientation
  float _velocity;		// Velocity for movement
  float _curFPS, _timer;
  stringstream _fpsText;

  char* _ikMethodString;

  bool _freeze, _showFPS, _debugViewMode, _wireframeMode, _freezeFly, _freezeIK, _ikStep;
  float _animTime;
  float _dx,_dy,_dz;
	
  // Engine objects
  H3DRes _pipeRes, _fontMatRes, _logoMatRes, _hdrPipeRes, _forwardPipeRes, _invJacMatRes;
  H3DNode _cam, _tentacle, _knight, _particleSys, _fly;

	std::string _contentDir;


  void keyHandler();

public:

  /**
   * Simple constructor for the tentacle TentacleApplication
   */
  TentacleApplication( const std::string &contentDir );

  /**
   * Instanciate objects and set up rendering engine
   */
  bool init();

  /**
   * Draws the scene
   */
  void mainLoop( float fps );

  /**
   * free memory before closing
   */
  void release();

  /**
   * change windows size
   */
  void resize( int width, int height );

  /**
   * What to do when a key is pressed
   */
  void keyPressEvent( int key );

  /**
   * update the state of one key
   * \param key the key whose state has changed
   * \param state the state of the key 
   */
  void keyStateChange( int key, bool state ) { if( key >= 0 && key < 320 ) _keys[key] = state; }

  /**
   * what to do when the mouse was moved
   */
  void mouseMoveEvent( float dX, float dY );
};

#endif // _app_H_
