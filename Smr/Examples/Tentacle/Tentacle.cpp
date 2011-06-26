/**
* \ingroup Examples
* \file Tentacle.cpp
* \brief This program is a simple introduction how to use simple inverse kinematics and Horde3D next gen rendering engine 
*  with Smr
*/

#include "Tentacle.h"

// Convert from degrees to radians
inline float degToRad( float f ) 
{
  return f * (3.1415926f / 180.0f);
}

inline float radToDeg( float f ) 
{
  return f * (180.0f / 3.1415926f);
}

inline float sqr( float f )
{
  return f*f;
}

inline float ranf()
{
  //return a random number between -1 and +1 
  float w =(static_cast<float>(rand()%10000)-5000.0f);
  if(w > 0) return (sqrt(w)/100.0f);
  else return -(sqrt(-w)/100.0f);
}

inline float pseudoGaussRand()
{
  float x1, x2, w;
  do {
    x1 = 2.0f * ranf() - 1.0f;
    x2 = 2.0f * ranf() - 1.0f;
    w = x1 * x1 + x2 * x2;
  } while ( w >= 1.0f );
  w = sqrt( (-2.0f * log( w ) ) / w );
  return x1 * w;
}

TentacleApplication::TentacleApplication( const std::string &contentDir )
{

  _contentDir = contentDir;

  //set up the state of the whole keyboard
  for( unsigned int i = 0; i < 320; ++i ) _keys[i] = false;

  // set up calera parameters
  _x = 5; _y = 3; _z = 19; _rx = 7; _ry = 15; _velocity = 10.0f;
  _curFPS = 30; _timer = 0;

  // other parameters
  _freeze = false; _showFPS = false; _debugViewMode = false; _wireframeMode = false; _freezeFly = false; _freezeIK = false; _ikStep = false;

  //set up nodeHandle index for camera
  _cam = 0;
}

/**
* update the Horde tentacle according to Smr _tentacleIK
*/
static void updateTentacle()
{
  //set of floats storing Horde's tentacle joints state
  float tx,ty,tz,rx,ry,rz,foo;
  double rrx,rry,rrz;
  // a Horde handle towards a tentacle Bone (joint)
  H3DNode tentacleBone;
  // and its corresponding node in Smr
  SMRKinematicJoint* ikJoint;

  //get the transformation parameters from the SmrTentacle
  for( unsigned int i=0; i < _tentacleIK->getNumJoints()-1 ; i++)
  {
    ikJoint = _tentacleIK->getJoint(i);

    //get Horde tentacle's corresponding joint joint (Horde) 
    h3dFindNodes(H3DRootNode,ikJoint->getName().c_str(), H3DNodeTypes::Joint);
    tentacleBone = h3dGetNodeFindResult(0);
    h3dGetNodeTransform(tentacleBone,&tx,&ty,&tz,&rx,&ry,&rz,&foo,&foo,&foo);

    //put the rotation values into euler angles (in degrees)
    ikJoint->getOrientation().toEulerAngles(rrx,rry,rrz);

    // update tentacle's joint according to Smr kinematic chain equivalent
    h3dSetNodeTransform( tentacleBone, tx, ty, tz, radToDeg( ((float)(rrx)) ), radToDeg( ((float)(rry)) ), radToDeg( ((float)(rrz)) ), foo, foo, foo );
  }
  // orientate Horde tentacles root joint correctly //No more in the new version !
  ikJoint = _tentacleIK->getJoint(0);
  h3dFindNodes(H3DRootNode,ikJoint->getName().c_str(), H3DNodeTypes::Joint);
  tentacleBone = h3dGetNodeFindResult(0);
  h3dGetNodeTransform(tentacleBone,&tx,&ty,&tz,&rx,&ry,&foo,&foo,&foo,&foo);
  h3dSetNodeTransform(tentacleBone,tx,ty,tz,rx,ry,rz,foo,foo,foo);
}

/**
* set up Smr IK tentacke according to Horde tentacle morphology
*/
SMRKinematicChain * setUpKinematikChain(H3DNode _firstNode)
{
  float tx,ty,tz,rx,ry,rz,foo;
  // currentJoint and previous joint referencing for relevant parenting when creating tentacles joints
  SMRKinematicJoint *currentJoint, *prevJoint;
  currentJoint = NULL;
  prevJoint = NULL;

  //prevJoint = NULL;
  // instanciate an empty kinematic chain
  SMRKinematicChain * tentacle = new SMRKinematicChain(RELATIVEMODE,TRANSLATIONFIRST,"tentacle");

  // get Hordes tentacle first joint
  _firstNode = h3dGetNodeChild(_firstNode,0);
  // again
  //_firstNode = h3dGetNodeChild(_firstNode,0);
  // will help computing Smr joints gain 
  int i = 0;
  // while Horde tentacles joints are found
  while (_firstNode >0)
  {
    //instanciate a new SMRKinematicJoint
    currentJoint = (new SMRKinematicJoint());
    //get joint translation parameters in parents local frame (bone length)
    h3dGetNodeTransform(_firstNode,&tx,&ty,&tz,&rx,&ry,&rz,&foo,&foo,&foo);
    //add DOF along X axis
    currentJoint->addDOF(SMRDOF::XAXIS,-M_PI/16.0f,M_PI/16.0f,degToRad(rx));
    //add DOF along Y axis
    currentJoint->addDOF(SMRDOF::YAXIS,-M_PI/3.0f,M_PI/3.0f,degToRad(ry));
    //add DOF along Z axis
    currentJoint->addDOF(SMRDOF::ZAXIS,-M_PI/3.0f,M_PI/3.0f,degToRad(rz));
    //set up joint gain (the more distal, the more gain)
    currentJoint->getDOF(0)->setGain(0.015*i);
    currentJoint->getDOF(1)->setGain(0.015*i);
    currentJoint->getDOF(2)->setGain(0.015*i);


    //give a name to the joint (same as Horde)
    currentJoint->setName(h3dGetNodeParamStr(_firstNode, H3DNodeParams::NameStr));
    //set up joint translation parameters in local frame (bone length)
    currentJoint->setPosition(tx,ty,tz);
    // take care of parenting issues
    if (prevJoint) 
    {
      currentJoint->setParentName(prevJoint->getName());
    }
    // and add the brand new joint into the inverse kinematics chain !
    tentacle->insertJoint(currentJoint);
    // keep a reference of the newly created joint for the next turn (parenting issue)
    prevJoint = currentJoint;
    // get the next joint according to Horde structure
    _firstNode = h3dGetNodeChild(_firstNode,0);
    // increase the gain increment
    i++;
  }
  tentacle->setStartJointIndex(tentacle->getJoint(0)->getName());

  // force the root joint not to move (tentacle basis should be fix)
  tentacle->getJoint(0)->getDOF(0)->setUpperBound(0);
  tentacle->getJoint(0)->getDOF(0)->setLowerBound(0);
  tentacle->getJoint(0)->getDOF(1)->setUpperBound(0);
  tentacle->getJoint(0)->getDOF(1)->setLowerBound(0);
  tentacle->getJoint(0)->getDOF(2)->setUpperBound(0);
  tentacle->getJoint(0)->getDOF(2)->setLowerBound(0);

  // and add the fly constraint (relative to tentacles' tip) to the tentacle
  _ikConstraint = new SMRIKConstraint(SMRVector3(0,0,0),SMRVector3(0,0,0),currentJoint->getName());

  // tentacle is instanciated and set up !
  return tentacle;
}

bool TentacleApplication::init()
{
  // Initialize engine
  if( !h3dInit() )
  {
    h3dutDumpMessages();
    return false;
  }

	// Set options
	h3dSetOption( H3DOptions::LoadTextures, 1 );
	h3dSetOption( H3DOptions::TexCompression, 0 );
	h3dSetOption( H3DOptions::MaxAnisotropy, 4 );
	h3dSetOption( H3DOptions::ShadowMapSize, 2048 );
	h3dSetOption( H3DOptions::FastAnimation, 1 );

  // Add resources
  // Pipelines
  _hdrPipeRes = h3dAddResource( H3DResTypes::Pipeline, "pipelines/hdr.pipeline.xml", 0 );
  _forwardPipeRes = h3dAddResource( H3DResTypes::Pipeline, "pipelines/forward.pipeline.xml", 0 );
  // Font
  _fontMatRes = h3dAddResource( H3DResTypes::Material, "overlays/font.material.xml", 0 );
  // Logo
  _logoMatRes = h3dAddResource( H3DResTypes::Material, "overlays/logo.material.xml", 0 );
  // invJacXpl
  //_invJacMatRes = h3dAddResource( H3DResTypes::Material, "materials/invJac.material.xml", 0 );
  // Environment
  H3DRes envRes = h3dAddResource( H3DResTypes::SceneGraph, "models/sphere/sphere.scene.xml", 0 );
  // Tentacle
  H3DRes tentacleRes = h3dAddResource( H3DResTypes::SceneGraph, "models/tentacle.scene.xml", 0 );
  // Fly
  H3DRes flyRes = h3dAddResource( H3DResTypes::SceneGraph, "models/fly.scene.xml", 0 );

  _dx = _dy = _dz = 0;

  // Load resources
  h3dutLoadResourcesFromDisk( _contentDir.c_str() );

  // Add scene nodes
  // Add camera
  _cam = h3dAddCameraNode( H3DRootNode, "Camera", _forwardPipeRes );
  // Add environment
  H3DNode env = h3dAddNodes( H3DRootNode, envRes );
  h3dSetNodeTransform( env, 0, -20, 0, 0, 0, 0, 20, 20, 20 );

  // Add tentacle node
  _tentacle = h3dAddNodes( H3DRootNode, tentacleRes );
  h3dSetNodeTransform( _tentacle, 0, 0, 0, 0, 0, 0.0, 1.0f, 1.0f, 1.0f );

  // Create tentacle kinematic chain
  _tentacleIK = setUpKinematikChain(_tentacle);
  //cout << _tentacleIK->getEndPosition(_tentacleIK->getNumJoints()-1);
  //_ikCCDSolver = new SmrCCDSolver(_tentacleIK);
  //_ikCCDSolver->addConstraintPtr(_ikConstraint);

  _ikGSMMSolver = new SMRGSMMSolver(_tentacleIK);
  _ikGSMMSolver->addConstraintPtr(_ikConstraint);

  _ikInvJacSolver = new SMRInvJacSolver(_tentacleIK);
  _ikInvJacSolver->addConstraintPtr(_ikConstraint);

  //delete(_tentacleIK);

  _currentSolver = _ikInvJacSolver;
  _ikMethodString = "J pseudo-inverse";

  // Add fly node
  _fly = h3dAddNodes( H3DRootNode, flyRes );
  h3dSetNodeTransform( _fly, 0, 4, 0, 0, 0, 0, 0.2f, 0.2f, 0.2f );

  // Add light source
  H3DNode light = h3dAddLightNode( H3DRootNode, "Light1", 0, "LIGHTING", "SHADOWMAP" );
  h3dSetNodeTransform( light, 0, 15, 10, -60, 0, 0, 1, 1, 1 );
  h3dSetNodeParamF( light, H3DLight::RadiusF,0, 30 );
  h3dSetNodeParamF( light, H3DLight::FovF,0, 90 );
  h3dSetNodeParamI( light, H3DLight::ShadowMapCountI, 0 );
  h3dSetNodeParamF( light, H3DLight::ShadowMapBiasF,0, 0.01f );
  h3dSetNodeParamF( light, H3DLight::ColorF3,0, 1.0f );
  h3dSetNodeParamF( light, H3DLight::ColorF3,1, 0.8f );
  h3dSetNodeParamF( light, H3DLight::ColorF3,2, 0.7f );

  // Customize post processing effects
	H3DNode matRes = h3dFindResource( H3DResTypes::Material, "pipelines/postHDR.material.xml" );

  //hdrParams: exposure, brightpass threshold, brightpass offset (see shader for description)
	h3dSetMaterialUniform( matRes, "hdrParams", 2.5f, 0.5f, 0.08f, 0 );

  return true;
}


void TentacleApplication::mainLoop( float fps )
{
  _curFPS = fps;
  _timer += 1 / fps;
  keyHandler();

  h3dSetOption( H3DOptions::DebugViewMode, _debugViewMode ? 1.0f : 0.0f );
  h3dSetOption( H3DOptions::WireframeMode, _wireframeMode ? 1.0f : 0.0f );

  if( !_freeze )
  {
    float tx,ty,tz,foo;
    h3dGetNodeTransform(_fly,&tx,&ty,&tz,&foo,&foo,&foo,&foo,&foo,&foo);
    if (!_freezeFly)
    {
      // routhly simulates the random fly of a fly
      if((rand() % 100)>95)
      {
        _dx = ((ranf()))/50.0f;
        _dy = ((ranf()))/50.0f;
        _dz = ((ranf()))/50.0f;
      }
      // change fly position
      h3dSetNodeTransform( _fly, tx+_dx, ty+_dy, tz+_dz, 0, 0, 0, 0.2f, 0.2f, 0.2f );
      if (tx < -4.0) _dx =  0.05f;
      if (tx >  4.0) _dx = -0.05f;
      if (ty <  0.0) _dy =  0.05f;
      if (ty >  6.0) _dy = -0.05f;
      if (tz < -4.0) _dz =  0.05f;
      if (tz >  4.0) _dz = -0.05f;
      h3dGetNodeTransform(_fly,&tx,&ty,&tz,&foo,&foo,&foo,&foo,&foo,&foo);


      //update constraint position according to fly position
      _ikConstraint->setPosition(SMRVector3(tx, ty, tz));
    }

    if (!_freezeIK || _ikStep)
    {
      //Compute inverse kinematics
      _currentSolver->process();

      // update tentacle's joint (IK)
      updateTentacle();
      _ikStep = false;
    }
  }

  // Set camera parameters
  h3dSetNodeTransform( _cam, _x, _y, _z, _rx ,_ry, 0, 1, 1, 1 );

  if( _showFPS )
  {
    // Avoid updating FPS text every frame to make it readable
    if( _timer > 0.3f )
    {
      _fpsText.str( "" );
      _fpsText << "FPS: " << fixed << setprecision( 2 ) << _curFPS;
      _timer = 0;
    }

    // Show text
    //h3dutShowText( _fpsText.str().c_str(), 0, 0.95f, 0.03f, 0, _fontMatRes );
  }

  // Show title
  //h3dutShowText( "Comparing Standard IK algorithms.", 0.0f, 0.90f, 0.03f, 0, _fontMatRes );

  // Show IK method
  //h3dutShowText( _ikMethodString, 0.6f, 0.80f, 0.03f, 0, _fontMatRes );


  // Show logo
  //h3dShowOverlay( 0.7f, 0.2, 0, 0,
  //					    1, 0.2, 1, 0,
  //                        1, 0.6f, 1, 1, 
  //						0.7f, 0.6f, 0, 1,
  //                      7, _invJacMatRes );

  // Render scene
  h3dRender( _cam );

  // Remove all overlays
  h3dClearOverlays();

  // Write all mesages to log file
  h3dutDumpMessages();
}


void TentacleApplication::release()
{
  // Release engine
  h3dRelease();
  // dele Smr objects
  delete(_ikGSMMSolver);
  delete(_ikInvJacSolver);
  //delete(_ikCCDSolver);

  delete(_ikConstraint);
  //delete(_tentacleIK);
}


void TentacleApplication::resize( int width, int height )
{
  // Resize viewport
	h3dSetNodeParamI(_cam,H3DCamera::ViewportXI, 0);
 h3dSetNodeParamI(_cam,H3DCamera::ViewportYI, 0);
 h3dSetNodeParamI(_cam,H3DCamera::ViewportWidthI, width);
 h3dSetNodeParamI(_cam,H3DCamera::ViewportHeightI, height);


  // Set virtual camera parameters
  h3dSetupCameraView( _cam, 45.0f, (float)width / height, 0.1f, 1000.0f );
}


void TentacleApplication::keyPressEvent( int key )
{
  if( key == 32 )		// Space
    _freeze = !_freeze;

  if( key == 260 )	// F3
  {
    if( h3dGetNodeParamI( _cam, H3DCamera::PipeResI ) == _hdrPipeRes )
      h3dSetNodeParamI( _cam, H3DCamera::PipeResI, _forwardPipeRes );
    else
      h3dSetNodeParamI( _cam, H3DCamera::PipeResI, _hdrPipeRes );
  }

  if( key == 264 )	// F7
    _debugViewMode = !_debugViewMode;

  if( key == 265 )	// F8
    _wireframeMode = !_wireframeMode;

  if( key == 266 )	// F9
    _showFPS = !_showFPS;

  if( key == 51 )// 3
  {
    //_currentSolver = _ikGSMMSolver;
    //_ikMethodString = "J transpose";
  } 

  if( key == 52 )	// 4
  {
    _currentSolver = _ikInvJacSolver;
    _ikMethodString = "J pseudoInv";
  }
  if( key == 53 )	// 5
  {
    //_currentSolver = _ikCCDSolver;
    //_ikMethodString = "CCD like";
  }
  if( key == 70 )	// F
    _freezeFly = !_freezeFly;

  if( key == 73 )// i
    _freezeIK = !_freezeIK;

  if( key == 80 )// p
    _ikStep= true;
}


void TentacleApplication::keyHandler()
{
  float curVel = _velocity / _curFPS;

  if( _keys[287] ) curVel *= 5;	// LShift

  if( _keys['W'] )
  {
    // Move forward
    _x -= sinf( degToRad( _ry ) ) * cosf( -degToRad( _rx ) ) * curVel;
    _y -= sinf( -degToRad( _rx ) ) * curVel;
    _z -= cosf( degToRad( _ry ) ) * cosf( -degToRad( _rx ) ) * curVel;
  }

  if( _keys['S'] )
  {
    // Move backward
    _x += sinf( degToRad( _ry ) ) * cosf( -degToRad( _rx ) ) * curVel;
    _y += sinf( -degToRad( _rx ) ) * curVel;
    _z += cosf( degToRad( _ry ) ) * cosf( -degToRad( _rx ) ) * curVel;
  }

  if( _keys['A'] )
  {
    // Strafe left
    _x += sinf( degToRad( _ry - 90) ) * curVel;
    _z += cosf( degToRad( _ry - 90 ) ) * curVel;
  }

  if( _keys['D'] )
  {
    // Strafe right
    _x += sinf( degToRad( _ry + 90 ) ) * curVel;
    _z += cosf( degToRad( _ry + 90 ) ) * curVel;
  }
}


void TentacleApplication::mouseMoveEvent( float dX, float dY )
{
  // Look left/right
  _ry -= dX / 100 * 30;
  // Loop up/down but only in a limited range
  _rx += dY / 100 * 30;
  if( _rx > 90 ) _rx = 90; 
  if( _rx < -90 ) _rx = -90;
}
