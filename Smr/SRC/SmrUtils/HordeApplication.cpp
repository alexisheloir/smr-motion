#include "HordeApplication.h"

// Convert from degrees to radians
inline float degToRad( float f ) 
{
	return f * (3.1415926f / 180.0f);
}

HordeApplication::HordeApplication( const std::string &appPath )
{
    
  //set up the state of the whole keyboard
  for(unsigned int i=0; i<320; ++i) m_keys[i]=false;

	_x = 5; _y = 3; _z = 19; _rx = 7; _ry = 15; _velocity = 10.0f;
	m_curFPS = 30;
  m_timer = 0;
  m_time = 0;

  //other parameters
  m_freeze=false; m_showFPS=true; m_debugViewMode=false; m_wireframeMode=false;
    
  _contentDir = extractAppPath(appPath) + "../../data/hordeContent/";
  cout << _contentDir;
}

bool HordeApplication::init()
{
  //Initialize engine
  if(!h3dInit())
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

  //Add resources
  //Pipeline
  m_forwardPipeRes=h3dAddResource(H3DResTypes::Pipeline,"pipelines/forward.pipeline.xml",0);
  //Font
  //m_fontMatRes=h3dAddResource(H3DResTypes::Material,"font.material.xml",0);
  // Skybox
  m_skyBoxRes=h3dAddResource(H3DResTypes::SceneGraph,"models/skybox/skybox.scene.xml",0);

  onLoadResources();

  m_frameGizmoRes.load("models/FrameGizmo/FrameGizmo.scene.xml");

  //Load resources
  h3dutLoadResourcesFromDisk(_contentDir.c_str());

  //Add scene nodes
  //Add camera
  //m_camera.add(m_forwardPipeRes);
  m_camera = h3dAddCameraNode( H3DRootNode, "Camera", m_forwardPipeRes );

  onAddNodes();
 // 
  m_frameGizmoNode.create(m_frameGizmoRes);
  m_frameGizmoNode.setTransform(0.0,0.0,0.0,-90,0,0,1,1,1);

 // 

  //Add light source
  H3DNode light=h3dAddLightNode(H3DRootNode,"Light1",0,"LIGHTING","SHADOWMAP");
  h3dSetNodeTransform(light,0,50,0,-90,0,0,1,1,1);
  h3dSetNodeParamF(light,H3DLight::RadiusF,0,200);
  h3dSetNodeParamF(light,H3DLight::FovF,0,90);
  
  /*h3dSetNodeParamI(light,H3DLight::ShadowMapCountI,1);
  h3dSetNodeParamF(light,H3DLight::ShadowMapBias,0,0.01f);*/

  h3dSetNodeParamI(light,H3DLight::ShadowMapCountI,3);
  h3dSetNodeParamF(light,H3DLight::ShadowSplitLambdaF,0,0.9f);
  h3dSetNodeParamF(light,H3DLight::ShadowMapBiasF,0,0.001f);

  h3dSetNodeParamF(light,H3DLight::ColorF3,0,1.0f);
  h3dSetNodeParamF(light,H3DLight::ColorF3,1,1.0f);
  h3dSetNodeParamF(light,H3DLight::ColorF3,2,1.0f);

  // Add skybox
  H3DNode sky = h3dAddNodes( H3DRootNode, m_skyBoxRes );
  h3dSetNodeTransform( sky, 0, 0, 0, 0, 0, 0, 210, 50, 210 );

  return true;
}

void HordeApplication::mainLoop(float fps)
{
  m_curFPS=fps;
  m_timer+=1/fps;
  m_time+=1/fps;
  keyHandler();

  h3dSetOption(H3DOptions::DebugViewMode,m_debugViewMode?1.0f:0.0f);
  h3dSetOption(H3DOptions::WireframeMode,m_wireframeMode?1.0f:0.0f);

  if(!m_freeze)
  {
    onUpdate();
  }

  //Set camera parameters
  h3dSetNodeTransform( m_camera, _x, _y, _z, _rx ,_ry, 0, 1, 1, 1 );

  if(m_showFPS)
  {
    //Avoid updating FPS text every frame to make it readable
    if(m_timer>0.3f)
    {
      m_fpsText.str("");
      m_fpsText<<fixed<<setprecision(2)<<m_curFPS<<"fps";
      m_timer=0;
    }

    //Show text
    //h3dutShowText(m_fpsText.str().c_str(),0.01f,0.96f,0.03f,0,m_fontMatRes);
  }

  //Render scene
  h3dRender(m_camera);

  glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_HINT_BIT | GL_LIGHTING_BIT);

  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);

  ////Display the Debug shit here
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  const float* camera = 0; 
  // // Retrieve camera position...      
  h3dGetNodeTransMats(m_camera, 0, &camera);   

  // // In case of an invalid camera (e.g. pipeline not set) return   
  if ( !camera ) return;   

  // // ... and projection matrix
  float projMat[16];
  h3dGetCameraProjMat(m_camera, projMat );

  // // Save OpenGL States
	 //glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_HINT_BIT | GL_LIGHTING_BIT);
	 //glDisable(GL_LIGHTING);		
	 //glDisable(GL_BLEND);
	//glMatrixMode( GL_PROJECTION );
	//glLoadMatrixf( projMat );
	//glMatrixMode( GL_MODELVIEW );

  // Set projection matrix
  glMatrixMode( GL_PROJECTION );
  glLoadMatrixf( projMat );
  // apply camera transformation
  glMatrixMode( GL_MODELVIEW );
  QMatrix4f transMat( camera );
  glLoadMatrixf( transMat.inverted().x );

  // then later in e.g. drawGizmo

  glPushMatrix();

  // //glTranslatef(0.0,2.0,0.0);

  intermediateDraw();
  // //m_debugDrawer.debugDrawWorld();

  glPopMatrix();

  glPopAttrib();

  h3dFinalizeFrame();

  //Remove all overlays
  h3dClearOverlays();

  //Write all messages to log file
  h3dutDumpMessages();
}

void HordeApplication::release()
{
  //Release engine
  h3dRelease();
}

void HordeApplication::resize(int width,int height)
{
  // Resize viewport
	h3dSetNodeParamI(m_camera,H3DCamera::ViewportXI,0);
	h3dSetNodeParamI(m_camera,H3DCamera::ViewportYI,0);
	h3dSetNodeParamI(m_camera,H3DCamera::ViewportWidthI,width);
	h3dSetNodeParamI(m_camera,H3DCamera::ViewportHeightI,height);
	//h3dSetupViewport( 0, 0, width, height, true );

  //Set virtual camera parameters
  //m_camera.setParameters(45.0f,width,height,0.1f,1000.0f);
	h3dSetupCameraView( m_camera, 45.0f, (float)width / height, 0.1f, 1000.0f );

}

void HordeApplication::onKeyPress(int key)
{
  if(key==32) //Space
    m_freeze=!m_freeze;

  if(key==259) //F2
    m_debugViewMode=!m_debugViewMode;

  if(key==260) //F3
    m_wireframeMode=!m_wireframeMode;

  if(key==261) //F4
    m_showFPS=!m_showFPS;
}

void HordeApplication::onKeyStateChange(int key,bool state)
{
  if(key>=0&&key<320) m_keys[key]=state;
}

void HordeApplication::keyHandler()
{
	float curVel = _velocity / m_curFPS;

	if( m_keys[287] ) curVel *= 5;	// LShift
	
	if( m_keys['W'] )
	{
		// Move forward
    _x -= sinf( degToRad( _ry ) ) * cosf( -degToRad( _rx ) ) * curVel;
		_y -= sinf( -degToRad( _rx ) ) * curVel;
		_z -= cosf( degToRad( _ry ) ) * cosf( -degToRad( _rx ) ) * curVel;
	}

	if( m_keys['S'] )
	{
		// Move backward
		_x += sinf( degToRad( _ry ) ) * cosf( -degToRad( _rx ) ) * curVel;
		_y += sinf( -degToRad( _rx ) ) * curVel;
		_z += cosf( degToRad( _ry ) ) * cosf( -degToRad( _rx ) ) * curVel;
	}

	if( m_keys['A'] )
	{
		// Strafe left
		_x += sinf( degToRad( _ry - 90) ) * curVel;
		_z += cosf( degToRad( _ry - 90 ) ) * curVel;
	}

	if( m_keys['D'] )
	{
		// Strafe right
		_x += sinf( degToRad( _ry + 90 ) ) * curVel;
		_z += cosf( degToRad( _ry + 90 ) ) * curVel;
	}
}

void HordeApplication::mouseMoveEvent(float dX,float dY)
{
	// Look left/right
	_ry -= dX / 100 * 30;
	
	// Loop up/down but only in a limited range
	_rx += dY / 100 * 30;
	if( _rx > 90 ) _rx = 90; 
	if( _rx < -90 ) _rx = -90;
}

void HordeApplication::onLoadResources()
{

}

void HordeApplication::onAddNodes()
{

}

void HordeApplication::onUpdate()
{

}

void HordeApplication::intermediateDraw()
{

}

std::string  HordeApplication::extractAppPath( const std::string &fullPath )
{
#ifdef __APPLE__
	std::string s( fullPath );
	for( int i = 0; i < 4; ++i )
		s = s.substr( 0, s.rfind( "/" ) );
	return s + "/../";
	//return s;
#else
	const std::string s( fullPath );
	if( s.find( "/" ) != std::string::npos )
		return s.substr( 0, s.rfind( "/" ) ) + "/";
	else if( s.find( "\\" ) != std::string::npos )
		return s.substr( 0, s.rfind( "\\" ) ) + "\\";
	else
		return "";
#endif
}
