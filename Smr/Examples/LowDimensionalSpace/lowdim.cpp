/**
* \ingroup Examples
* \file lowdim.cpp
* \brief This program allows to navigate through a 2-dimensional PCA space
*  with Smr
*/
// -------------- Includes
#include "CImg.h"
using namespace cimg_library;

#include "Smr.h"
#include "Shaders.h"
#include "SmrPCA.h"
#include "SmrBasicRenderer.h"
#ifdef __MACH__
  #include <glut.h>
#else
  #include <GL/glut.h>
#endif
SMRBasicRenderer * myBasicRenderer;
SMRPCALinearizedQuaternion * myPCA;

SMRMotion motion;
SMRSkeleton finalSkel;
double min1, max1, min2, max2;
unsigned int frame =0;
float framet =0;
unsigned int eigen = 0;

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

// ---------------- CImg related stuff
/// Define colors used to plot the trajectory in low dim space
#define SIZE_SPACE 500
int x=0,y=0;
const unsigned char red[]    = {255,0,0};
const unsigned char blue[]    = {0,255,200};
const unsigned char white[]    = {255,255,255};
CImg <unsigned char> dImg(SIZE_SPACE,SIZE_SPACE,1,3,10);
CImgDisplay main_disp(SIZE_SPACE,SIZE_SPACE,"Low Dimensional Space");

/******************************************************************************/
// intialize rendering scene... what's going on and on 
static void InitScene(void)
{
	// ---------------------------------------------------------------------------------------
	// SMR Related stuff 
	// ---------------------------------------------------------------------------------------
	/// load a motion
	motion = loadMotionFromBVH(getFileName("bvh/normal1.bvh"));	
	/// cut the first 3 frames of the motion (T-pose)
	motion.cutMotionUntilFrame(3);
	/// switch every pose to absolute mode
	motion.changeMode(ABSOLUTEMODE);
	///  create new PCA on linearized quaternions object
	myPCA =  new SMRPCALinearizedQuaternion (motion,RELATIVEMODE);
	/// performing eigen analysis
	cout << "Computing Eigen Postures" ;
	myPCA->computeEigenPostures();	
	cout << "... Done !"<<endl;
	
	// get motion curves 
	SMRTimeSerie<> mCurve1 = myPCA->getMotionCurve(1);
	SMRTimeSerie<> mCurve2 = myPCA->getMotionCurve(2);
	// normalize curves
	min1 = mCurve1.getMin(); max1 = mCurve1.getMax();
    min2 = mCurve2.getMin(); max2 = mCurve2.getMax();
	SMRTimeSerie<unsigned int> mCoord1, mCoord2;
	for (unsigned int i=0; i < mCurve1.getLength(); i++){
		mCoord1.add((mCurve1[i] - min1)*SIZE_SPACE/(max1 - min1));	
		mCoord2.add((mCurve2[i] - min2)*SIZE_SPACE/(max2 - min2));	
	}
	// draw in image
	dImg.draw_grid(100,100,-30,-30,false,false,white,0.5f);
	// draw the trajectory in image
	for (unsigned int t=0; t < mCoord1.getLength()-1; t++)
		dImg.draw_line(mCoord1[t],mCoord2[t],mCoord1[t+1],mCoord2[t+1],red);
	main_disp.display(dImg);

	/// init renderer
	myBasicRenderer = new SMRBasicRenderer(&finalSkel);
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


	//---------------------------------------------------------------
	// CImg Related stuff
	//--------------------------------------------------------------
	const unsigned int but = main_disp.button;
    x = main_disp.mouse_x; y = main_disp.mouse_y;
	CImg<unsigned char> finalImage = dImg;
	if (main_disp.is_resized||but) main_disp.resize().display(finalImage.draw_circle(x,y,5,blue));



	vector<double> coefs;
	coefs.push_back(x*(max1-min1)/SIZE_SPACE + min1);coefs.push_back(y*(max2-min2)/SIZE_SPACE + min2);
	if(but) finalSkel = myPCA->getProjectedPose(coefs);


	glScaled(0.001,0.001,0.001);
	glRotatef(-90,1,0,0);
	glRotatef(-90,0,0,1);
	myBasicRenderer->draw();

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
	glutInitWindowSize(WINSIZE,WINSIZE);
	glutCreateWindow("Low Dimensional Space demonstration");
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
	cout << "*** PCA Low Dimensional Space exploration" << endl 
	 << "----------------------------------- " << endl;

	// get GLUT to work
	InitializeGlut(&argc, argv);
	// Init the Smr Library
	Smr::initSmr();
	// Init the shaders capability
	InitShaders();
	//// Get my scene prepared
	InitScene();
	//// Enter the endless loop
	glutMainLoop(); 
	// Exit properly
	ShutDown();
	return 0;
}
