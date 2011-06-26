/**
* \ingroup Examples
* \file PCA.cpp
* \brief This program is a simple introduction how to use PCA decompostion 
*  with Smr
*/
// -------------- Includes
#include "Smr.h"
#include "Shaders.h"
#include "SmrPCA.h"
#include "SmrBasicRenderer.h"
# ifdef __MACH__
  #include <glut.h>
#else
  #include <GL/glut.h>
#endif

/// the toon shaded rendered that will be used
SMRBasicRenderer * myBasicRenderer;
/// 4 motion clips to be displayed -> 4 motion players
SMRMotionPlayer * myMotionPlayer;
SMRMotionPlayer * myMotionPlayer2;
SMRMotionPlayer * myMotionPlayer3;
SMRMotionPlayer * myMotionPlayer4;
/// The smr PCA reference towards the instqnce that will perfom the actual ˆotion decomposition
SMRPCALinearizedQuaternion * myPCA;

/// 4 motions we will use (original + 3 degradated reprojected motions)
SMRMotion motion, projMotion, projMotion2, projMotion3 ;

/// 
unsigned int frame =0;
float framet =0;
unsigned int eigen = 0;

/// rendering engine 
GLfloat pPosition_light[4];
GLfloat pAmbient_light[4];
double theta = 0 ;

//--------------- view parameters
static const float WINSIZE = 512;
float dimx, dimy = WINSIZE;
int ox, oy;
int buttonState = 0;
float tx = 0, ty = -1, tz = -3;
float rx = 0, ry = 0;
float delta = 0.01;
float radius = 5;

bool m_renderPlain = true;
bool onPlay = true;
/******************************************************************************/
// intialize rendering scene... what's going on and on 
static void InitScene(void)
{
	// ---------------------------------------------------------------------------------------
	// OpenGL Related stuff 
	// ---------------------------------------------------------------------------------------
	// setup some light
	pPosition_light[0] = 2.0f;	pPosition_light[1] = 1.5f ;	pPosition_light[2] = 2.0f; pPosition_light[3] = 1.0f;
	glLightfv( GL_LIGHT0, GL_POSITION, pPosition_light );

	// ---------------------------------------------------------------------------------------
	// SMR Related stuff 
	// ---------------------------------------------------------------------------------------
	/// load a motion
	motion = loadMotionFromBVH(getFileName("bvh/unknown.bvh"));	
	/// cut the first 3 frames of the motion (T-pose)
	motion.cutMotionUntilFrame(3);
	/// switch every pose to absolute mode
	motion.changeMode(ABSOLUTEMODE);
	///  create new PCA on linearized quaternions object
	myPCA =  new SMRPCALinearizedQuaternion (motion,RELATIVEMODE);
	/// performing eigen analysis
	myPCA->computeEigenPostures();	
	cout << "Reconstructing Motion" ;
	cout << " ------ Done !" << endl << "Projection on 1 dimensional subspace : "; projMotion = myPCA->getProjectedMotion(1); 
	cout << " ------ Done !" << endl << "Projection on 3 dimensional subspace : "; projMotion2 = myPCA->getProjectedMotion(3);
	cout << " ------ Done !" << endl << "Projection on 5 dimensional subspace : ";projMotion3 = myPCA->getProjectedMotion(5);
	cout << " ------ Done !"<<endl;
	///init motion players
	myMotionPlayer = new SMRMotionPlayer(&motion);
	myMotionPlayer->setInterpolationMode(true);
	myMotionPlayer2 = new SMRMotionPlayer(&projMotion);
	myMotionPlayer2->setInterpolationMode(true);
	myMotionPlayer3 = new SMRMotionPlayer(&projMotion2);
	myMotionPlayer3->setInterpolationMode(true);
	myMotionPlayer4 = new SMRMotionPlayer(&projMotion3);
	myMotionPlayer4->setInterpolationMode(true);
	/// init renderer
	myBasicRenderer = new SMRBasicRenderer(myMotionPlayer->getSkeleton());
}

// Exit properly
static void ShutDown()
{	// shutdown Smr library
	Smr::shutdown();
}


/******************************************************************************/
// DISPLAY CALLBACK
/******************************************************************************/

static void DisplayScene(void)
{
	// erase previous buffer
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(tx,-ty,-tz,  0,1,0,  0,3,0);

	{
		//glScalef(0.001f,0.001f,0.001f);
		//glRotatef(-90,1,0,0);
		//glRotatef(-90,0,0,1);
		if (onPlay)	framet+=0.4f;
		myMotionPlayer->setFrameToDisplay(framet);
		myMotionPlayer->process();
		myMotionPlayer2->setFrameToDisplay(framet);
		myMotionPlayer2->process();
		myMotionPlayer3->setFrameToDisplay(framet);
		myMotionPlayer3->process();
		myMotionPlayer4->setFrameToDisplay(framet);
		myMotionPlayer4->process();
		
		glTranslated(-1.5,0,0);
		myBasicRenderer->setRenderStyle(SMRBasicRenderer::toon);
		myBasicRenderer->setSkeletonPtr(myMotionPlayer->getSkeleton());
		myBasicRenderer->draw();
		myBasicRenderer->setRenderStyle(SMRBasicRenderer::gooch);
		glTranslated(1,0,0);
		myBasicRenderer->setSkeletonPtr(myMotionPlayer2->getSkeleton());
		myBasicRenderer->draw();
		glTranslated(1,0,0);
		myBasicRenderer->setSkeletonPtr(myMotionPlayer3->getSkeleton());
		myBasicRenderer->draw();
		glTranslated(1,0,0);
		myBasicRenderer->setSkeletonPtr(myMotionPlayer4->getSkeleton());
		myBasicRenderer->draw();
	}
	// Swap rendering buffers for double buffered 
	glutSwapBuffers();
}

/******************************************************************************/

static void Idle(void)
{
	glutPostRedisplay();
}




/******************************************************************************/

void mmotion(int x, int y)
{
  float dx, dy; dx = x - ox; dy = y - oy;
  if (buttonState == 3) 
	  tz -= dx / 5.0; 
  else if (buttonState & 2) 
  {	tx += dx / 100.0;
	ty -= dy / 100.0;}
  else if (buttonState & 1) 
  {tx += dx / 100.0;
	ty -= dy / 100.0;}
  ox = x; oy = y;
  glutPostRedisplay();
}

/******************************************************************************/

void mouse(int button, int state, int x, int y)
{
  int mods;
  if (state == GLUT_DOWN)
 	buttonState |= 1<<button;
	  else if (state == GLUT_UP)
		buttonState = 0;
	  mods = glutGetModifiers();
	  if (mods & GLUT_ACTIVE_SHIFT) {
		buttonState = 2;
	  } else if (mods & GLUT_ACTIVE_CTRL) {
		buttonState = 3;
	  } 
  ox = x; oy = y;
  glutPostRedisplay();
}

/******************************************************************************/

void keyboard( unsigned char key, int x, int y ) {
	switch((int) key) {
	case '\033': 
		ShutDown();
		exit( 0 );
		break; 
	case 'p':
		onPlay =  onPlay ? false : true;
		break;
	case ' ':
		framet = 0; 
		frame = 0;
		break;
	case 'e':
		eigen++;
		break;
	case 'r':
		eigen = 0 ;
		break;
	}
} 

/******************************************************************************/

void reshape(int w, int h)
{
  dimx = w;  dimy = h;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, (float) w / (float) h, 0.001, 1000.0);
  glMatrixMode(GL_MODELVIEW);
  glViewport(0, 0, w, h);
}
/******************************************************************************/
static void InitShaders()
{
  GLenum err = glewInit();
  if( GLEW_OK != err )
  {
    // Problem: glewInit failed, something is seriously wrong.
	cerr << "Error: " << glewGetErrorString( err ) << endl;
  }
  cout << "GLEW Version String: " << glewGetString( GLEW_VERSION ) << endl;
  cout << "OpenGL Version String: " << glGetString( GL_VERSION ) << endl;   
}
/******************************************************************************/
static void InitView()
{	// initialize matrix stacks
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(45.0, (float) WINSIZE / (float) WINSIZE, 0.001, 1000.0);
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glViewport( 0, 0, WINSIZE, WINSIZE );
	glEnable( GL_DEPTH_TEST );
	glClearColor(0.2f,0.2f,0.2f,0);
	glShadeModel( GL_SMOOTH );
}

/******************************************************************************/

static void InitializeGlut(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowSize(WINSIZE*2,WINSIZE);
	glutCreateWindow("PCA Demonstration");
	glutDisplayFunc(DisplayScene);
	glutIdleFunc(Idle);
	glutKeyboardFunc( keyboard );
	glutMouseFunc(mouse);
	glutMotionFunc(mmotion); 
	glutPassiveMotionFunc(mmotion);
	glutReshapeFunc(reshape);
	InitView();
}



/******************************************************************************/
/******************************************************************************/
// MAIN 


int main(int argc, char *argv[])
{
	cout << "*** PCA examples on motion data with Smr" << endl 
	 << "----------------------------------- " << endl;

	// get GLUT to work
	InitializeGlut(&argc, argv);
	// Init the Smr Library
	Smr::initSmr();
	// Init the shaders capability
	InitShaders();
	// Get my scene prepared
	InitScene();
	// Enter the endless loop
	glutMainLoop(); 
	// Exit properly
	ShutDown();

	return 0;
}
