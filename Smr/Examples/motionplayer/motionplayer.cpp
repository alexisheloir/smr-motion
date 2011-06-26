/// \defgroup Examples Smr examples program

/// \ingroup Examples
/// \file motionplayer.cpp
/// \brief This program demontrates the use of motion player and its interpolation 
/// method


#include "Smr.h"
#include "Shaders.h"
//#include "SmrPCA.h"
#include "SmrBasicRenderer.h"
#ifdef __MACH__
  #include <glut.h>
#else
  #include <GL/glut.h>
#endif
#include "argh.h"

SMRBasicRenderer * myBasicRenderer;
SMRMotionPlayer * myMotionPlayer;

SMRSkeleton mySkeleton;
SMRMotion myMotion;

unsigned int frame =0;
float framet =0;
double tempo = 0.01;

GLfloat pPosition_light[4];
double theta = 0 ;

//--------------- view parameters
static const float WINSIZE = 800;
float dimx, dimy = WINSIZE;
int ox, oy;
int buttonState = 0;
float tx = -2, ty = -3, tz = -3;
float rx = 0, ry = 0, rz = 0;
float sz = 1.0f;
float delta = 0.01;
float radius = 5;


/**
* \brief intialize all elements in the scene 
*/

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
  // set interpolation mode to be true
  ////myMotionPlayer->setInterpolationMode(true);
  // create the rendering engine with a pointer to the motion player skeleton
  myBasicRenderer = new SMRBasicRenderer(myMotionPlayer->getSkeleton());
  myBasicRenderer->setRenderStyle(SMRBasicRenderer::toon);
  myBasicRenderer->setScaleFactor(0.1);
  myBasicRenderer->setSkeletonPtr(myMotionPlayer->getSkeleton());
  // that's all
}

/**
* \brief Exit properly 
*/
static void ShutDown(){
  cout << "Shutting down the light..." ;
  // shutdown Smr library
  Smr::shutdown();
  delete(myMotionPlayer);
  delete(myBasicRenderer);
  cout << "Done !" << endl;
}


/******************************************************************************/
// DISPLAY CALLBACK
/******************************************************************************/

static void DisplayScene(void)
{
  // Setup view volume
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(-2.0,-3.0,-3.0,  0,0,0,  0,3,0);

  // ---------------------------------------------------------------------------------------
  // SMR Related stuff 
  // ---------------------------------------------------------------------------------------
  // time is running
  framet+=tempo;
  // this will trigger interpolation mode since framet is a non-integer parameter
  myMotionPlayer->setFrameToDisplay(framet);
  myMotionPlayer->process(framet/myMotionPlayer->getTotalTime());
  //cout << framet/myMotionPlayer->getTotalTime() << endl;
  //glTranslatef(1.0f, 0.0f,0.0f);
  glRotatef(ry,0.0,1.0f,0.0f);
  glRotatef(rz,0.0,0.0f,1.0f);
  glScalef(sz,sz,sz);
  myBasicRenderer->draw();
  // back to original render style
  // Swap rendering buffers for double buffered 
  glutSwapBuffers();
}

/******************************************************************************/

static void Idle(void)
{
  glutPostRedisplay();
}




/******************************************************************************/

void motion(int x, int y){
  float dx, dy;dx = x - ox;dy = y - oy;
  if (buttonState == 3){
    // left+middle = zoom
    sz -= dx / 100.0;} 
  else if (buttonState & 2) {
    // middle = translate  
    ry += dx / 10.0;
    rz -= dy / 10.0;}
  else if (buttonState & 1){
    ry += dx / 10.0;
    rz -= dy / 10.0;}
  ox = x; oy = y;
  glutPostRedisplay();
}

/******************************************************************************/

void mouse(int button, int state, int x, int y){
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
  case ' ':
    framet = 0; 
    frame = 0;
    break;
  case '+':
    tempo += 0.1;
    break;
  case '-':
    tempo -= 0.1;
    break;

  }

} 

/******************************************************************************/

void reshape(int w, int h)
{
  dimx = w;
  dimy = h;

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
{
  // initialize matrix stacks
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  gluPerspective(45.0, (float) WINSIZE / (float) WINSIZE, 0.001, 1000.0);
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  glViewport( 0, 0, WINSIZE, WINSIZE );
  glEnable( GL_DEPTH_TEST );
  glClearColor(0.9f,0.9f,0.9f,0);
}

/******************************************************************************/

static void InitializeGlut(int *argc, char *argv[])
{
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
  glutInitWindowSize(WINSIZE,WINSIZE*0.8);
  glutCreateWindow("Motion Player Demonstration");
  glutDisplayFunc(DisplayScene);
  glutIdleFunc(Idle);
  glutKeyboardFunc( keyboard );
  glutMouseFunc(mouse);
  glutMotionFunc(motion); 
  glutPassiveMotionFunc(motion);
  glutReshapeFunc(reshape);
  InitView();
}



/******************************************************************************/
/******************************************************************************/
// MAIN 


int main(int argc, char *argv[])
{
  string inputSkeleton;
  string inputMotion;
  SMRMotion motion;

  ParamHandler argh;
  argh.AddLong("motion", 'm').SetString().SetDesc("input motion file" , "<file>");
  argh.AddLong("skeleton", 's').SetString().SetDesc("input skeleton file" , "<file>");
  argh.AddLong("help", 'h').SetBool().SetDesc("Displays quick manual.");

  argh.StartParse(argc, argv);
  for(;;)
  {
    int c = argh.GetParam();
    if(c == -1)break;
    switch(c)
    {
    case 's': inputSkeleton = argh.GetString(); break;//worst = argh.GetBool(); break;
    case 'm': inputMotion = argh.GetString(); break;
    default :
      printf(
        "*** SMR simple viewer demo ***\n"
        "\n Usage: motionPlayer [<option> [<...>]] \n"
        "\n This demo lets you play bvh or asf/amc animations\n");
      argh.ListOptions();
      printf("\nNo warranty whatsoever.\n");
      return 0;
    }
  }

  if (inputMotion.find(".amc",0) !=  string::npos )
  {
    if (inputSkeleton.find(".asf") == string::npos )
    {
      cout << "You shall specify an asf skeleton file with an amc file" << endl;
      return 0;
    }else
    {
      motion = loadMotionFromAcclaim( getFileName(inputSkeleton), getFileName(inputMotion) );
      myMotionPlayer = new SMRMotionPlayer( &motion );
    }
  }else if (inputMotion.find(".bvh",0) !=  string::npos )
  {
    motion = loadMotionFromBVH( getFileName(inputMotion) );
    myMotionPlayer = new SMRMotionPlayer( &motion );
  }else if (inputSkeleton.find(".vsk",0) !=  string::npos )
  {
    motion = loadMotionFromVSK( inputSkeleton, inputMotion);
    myMotionPlayer = new SMRMotionPlayer( &motion );
    //SMRSkeleton mySkeleton = loadSkeletonFromVSK(inputSkeleton);
    //SMRQuaternion identity;
    //identity.identity();
    //for(int i = 0; i< mySkeleton.getNumJoints(); i++)
    //{
    //  mySkeleton.getJoint(i)->setOrientation(identity);
    //}
    //save the file as bvh

    //exportMotionToBVH("shrug.bvh",motion, mySkeleton);
    //
    //exportPoseToBVH("blah.bvh", mySkeleton);
    //mySkeleton.setMode(RELATIVEMODE);
  }
  else
  {
    cout << "unknown file format" << endl;
    return 0;
  }

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
  //delete(motion);
  return 0;
}

