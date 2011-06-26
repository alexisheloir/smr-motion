/**
* \ingroup Examples
* \file quatViz.cpp
* \brief This program is a simple quaternionic time serie viewer
* two modes are available either:
*  - either browse the time serie of a given motion
*  - either visualize a quaternionic gaussian sampling
*/

// -------------- Includes
#include "Smr.h"
#include "SmrTimeSerie.h"
#include "SmrPCA.h"
#include "SmrQuaternion.h"
#include "SmrLoader.h"

// Windows or Linux
#if defined(__WIN32__) || ! defined(__APPLE__)
  #include <GL/glut.h>
#else
// Mac
  #include <glut.h>
#endif

GLfloat pPosition_light[4];

SMRTimeSerie<SMRQuaternion> serie, serie1 ;
SMRQuaternion identity;
SMRQuaternion myQuaternion;

double rotX = M_PI/12.0;
double rotY = M_PI/12.0;
double rotZ = 0.0;


//--------------- view parameters
static const float WINSIZE = 1000;
float dimx, dimy = WINSIZE;
int ox, oy;
int buttonState = 0;
float tx = 1, ty = 1, tz = 0;
float rx = 0, ry = 0;
float delta = 0.01;
float radius = 5;
bool plain = true;

unsigned int cpt = 0;
bool mode = true;


/**
* \brief intialize all elements in the scene 
*/
static void InitScene(void)
{

  double theta  =  10.0 * M_PI / 180.0;
  double phi    =  15.0 * M_PI / 180.0;
  double gamma  =  30.0 * M_PI / 180.0;
  double alpha  =  50.0 * M_PI / 180.0;

  SMRQuaternion q0(1.0,cos(phi),sin(phi),sin(alpha));
  q0.fromEulerAngles(phi,gamma,theta);
  q0.normalize();
  myQuaternion = q0;

  SMRQuaternion q1(1.0,cos(gamma),sin(gamma),-sin(gamma));
  q1.fromEulerAngles(0.01,0.0,0.0);
  q1.normalize();
  identity = q1;
  
	for (double t=0.0; t<1.0; t+=0.01)
  {
    serie.add( Slerp(identity,myQuaternion,t).normalize() );
  }

	for (double t=0.0; t<1.0; t+=0.1)
  {
    SMRQuaternion newQuaterion;
    newQuaterion.fromEulerAngles(rotX*t,rotY*t,rotZ*t);
    newQuaterion.normalize();
    serie1.add(newQuaterion);
  }

}

/**
* \brief Exit properly 
*/
static void ShutDown()
{	/// shutdown SMR library
	Smr::shutdown();
}


/******************************************************************************/
// DISPLAY CALLBACK
/******************************************************************************/
static void drawAxis(void)
{
	glPushMatrix();
	glScalef(1.0,1.0,1.0);
	glTranslatef(0,0,0);
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);	// X AXIS STARTS - COLOR RED
	glVertex3f(-0.2f,  0.0f, 0.0f);
	glVertex3f( 0.2f,  0.0f, 0.0f);
	glVertex3f( 0.2f,  0.0f, 0.0f);	// TOP PIECE OF ARROWHEAD
	glVertex3f( 0.15f,  0.04f, 0.0f);
	glVertex3f( 0.2f,  0.0f, 0.0f);	// BOTTOM PIECE OF ARROWHEAD
	glVertex3f( 0.15f, -0.04f, 0.0f);
	glColor3f(0.0f, 1.0f, 0.0f);	// Y AXIS STARTS - COLOR GREEN
	glVertex3f( 0.0f,  0.2f, 0.0f);
	glVertex3f( 0.0f, -0.2f, 0.0f);			
	glVertex3f( 0.0f,  0.2f, 0.0f);	// TOP PIECE OF ARROWHEAD
	glVertex3f( 0.04f,  0.15f, 0.0f);
	glVertex3f( 0.0f,  0.2f, 0.0f);	// BOTTOM PIECE OF ARROWHEAD
	glVertex3f( -0.04f,  0.15f, 0.0f);
	glColor3f(0.0f, 0.0f, 1.0f);	// Z AXIS STARTS - COLOR BLUE
	glVertex3f( 0.0f,  0.0f,  0.2f);
	glVertex3f( 0.0f,  0.0f, -0.2f);
	glVertex3f( 0.0f,  0.0f, 0.2f);	// TOP PIECE OF ARROWHEAD
	glVertex3f( 0.0f,  0.04f, 0.15f);
	glVertex3f( 0.0f, 0.0f, 0.2f);	// BOTTOM PIECE OF ARROWHEAD
	glVertex3f( 0.0f, -0.04f, 0.15f);
	glColor3f(0.0f, 0.0f,0.0f);
	glEnd();
	glPopMatrix();
}

static void drawQuaternion(const SMRQuaternion & q, const char*  _color = " ")
{
	SMRVector3 axis = q.getRotationAxis(); 
	double rot = q.getRotationAngle(); 
	glPointSize(4.0);
	// draw point on sphere
	glBegin(GL_POINTS);	
  // Color represents rotation:
  //  0, 2Pi :    blue
  //  Pi, :       red
  if (!strcmp(_color, "red")==0)
  { 
    if (!strcmp(_color, "blue")==0)
    {
      if (!strcmp(_color, "green")==0)
      {
        if (!strcmp(_color, "yellow")==0)
        { 
          if (!strcmp(_color, "white")==0)
          { 
            if (!strcmp(_color, "black")==0)
            { 
              if (!strcmp(_color, "magenta")==0) glColor3d(q.m_w,0,1.0-q.m_w);
              else glColor3d(1,0,1);
           }else glColor3d(0,0,0);
         }else glColor3d(1,1,1);
       }else  glColor3d(1,1,0);
     }else  glColor3d(0,1,0);
   }else glColor3d(0,0,1);           
 }else glColor3d(1,0,0);  

 
  glVertex3d(axis.m_x,axis.m_y,axis.m_z);
	glEnd();
}


static void drawTimeSerie(SMRTimeSerie<SMRQuaternion>  _serie, const char* _color = " "){
	// draw quaternions
	for (unsigned int i=0; i < _serie.getLength() ; i++)
  {
		drawQuaternion(_serie[i],_color);
  }
}

static void DisplayScene(void){
	// erase previous buffer
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	SMRVector3 pos(4,0,0);
	gluLookAt(pos.m_x,pos.m_y,pos.m_z,  0,0,0,  0,1,0);

	glPushMatrix();
	glTranslatef(0,-1.3,1.3);
	glRotatef(tx,0,1,0);
	glRotatef(-ty,0,0,1);
	drawAxis();
	glPopMatrix();


	// display some text
	glPushMatrix();
		glTranslatef(0,-1.3,0.5);
		glRotatef(tx,0,1,0);
		glRotatef(-ty,0,0,1);	
		glRasterPos2f(0,0);
		if (mode)
		{
			const char * s = "Motion time series. Joint:  ";
			while(*s){
				glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,*s);
				s++;}
			//string name = motion.getSkeleton(0).getJoint(cpt)->getName();
      string name = "troulala";
			const char * s2 = name.c_str();
			while(*s2){glutBitmapCharacter(GLUT_BITMAP_9_BY_15,*s2);
			s2++;}
		}
		else
		{
			const char * s = "5000 samples from a gaussian quaternionic distribution";
			while(*s){glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,*s);
			s++;}
		}
	glPopMatrix();


	glLightfv(GL_LIGHT0,GL_POSITION,pPosition_light);
	glRotatef(tx,0,1,0);
	glRotatef(-ty,0,0,1);


	// draw the unit sphere S2
	glEnable(GL_LIGHTING);
	glutSolidSphere(0.97,50,50);
	glDisable(GL_LIGHTING);



	// ---------------------------------------------------------------------------------------
	// SMR Related stuff 
	// ---------------------------------------------------------------------------------------
	if(mode)
  { 
    drawQuaternion(identity,"blue");
    drawQuaternion(myQuaternion,"magenta");
  }
  else
  {
		drawTimeSerie(serie, "red");	
		//drawTimeSerie(serie1, "green");	
  }

	// Swap rendering buffers for double buffered 
	glutSwapBuffers();
}

/******************************************************************************/

static void Idle(void){
	glutPostRedisplay();
}




/******************************************************************************/

void motionMouse(int x, int y)
{
	float dx, dy; dx = x - ox; dy = y - oy;
	if (buttonState & 1) 
	{
		tx += dx / 10;
		ty += dy / 10;
	}
	ox = x; oy = y;
	glutPostRedisplay();
}

/******************************************************************************/

void mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
		buttonState |= 1<<button;
	else if (state == GLUT_UP)
		buttonState = 0;
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
	case 'w': 
		plain = plain ? 0 : 1;
		if (plain) glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		else glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		break; 
	case '+':
//		cpt = cpt!=(motion.getSkeleton(0).getNumJoints()-1) ? cpt+1:0;
		break;
	case '-':
//		cpt = cpt!=0 ? cpt-1:motion.getSkeleton(0).getNumJoints()-1;
		break;
	case ' ':
		mode = mode ? false:true;
		break;
	}

} 

/******************************************************************************/

void reshape(int w, int h)
{
	GLdouble iaspect = (GLdouble) h / (GLdouble) w;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1.5,1.5,-1.5*iaspect,1.5*iaspect,0.01,1000.0);
	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, w, h);
}

/******************************************************************************/
static void InitView()
{	// initialize matrix stacks
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	glOrtho(-1.5,1.5,-1.5,1.5,0.01,1000.0);
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glViewport( 0, 0, WINSIZE, WINSIZE );
	glEnable( GL_DEPTH_TEST );	
	//glClearColor(0.8,0.8,0.8,1);
	glClearColor(1,1,1,1);

	glShadeModel( GL_SMOOTH );
}

/******************************************************************************/

static void InitializeGlut(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowSize(WINSIZE,WINSIZE);
	glutCreateWindow("Quaternion Visualization");
	glutDisplayFunc(DisplayScene);
	glutIdleFunc(Idle);
	glutKeyboardFunc( keyboard );
	glutMouseFunc(mouse);
	glutMotionFunc(motionMouse); 
	glutPassiveMotionFunc(motionMouse);
	glutReshapeFunc(reshape);
	InitView();

	GLfloat matDiff[4] = {0.8,0.8,0.8,1};
	GLfloat matAmb[4] = {0.6,0.6,0.6,1};
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,matDiff);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,matAmb);

	glEnable(GL_LIGHT0);
	glLightfv( GL_LIGHT0, GL_DIFFUSE, matDiff );
	glLightfv( GL_LIGHT0, GL_AMBIENT, matAmb );


	// init light position
	pPosition_light[0] = 10.0f;	pPosition_light[1] = 5.0f ;	pPosition_light[2] = -10.0f; pPosition_light[3] = 1.0f;

	glShadeModel( GL_SMOOTH );
}



/******************************************************************************/
/******************************************************************************/
// MAIN 


int main(int argc, char *argv[])
{
	// get GLUT to work
	InitializeGlut(&argc, argv);
	// Init the SMR Library
  Smr::initSmr();

	// Get my scene prepared
	InitScene();

	// Enter the endless loop
	glutMainLoop(); 

	// Exit properly
	ShutDown();

	return 0;
}
