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
SMRMotion motion;

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

// covariances value for gaussian distribution
double covx = 1.0;
double covy = 0.09;
double covz = 0.03;
double covalpha = 1.0;




/**loads a quaternion series and interpolation parameters from a file**/

bool readQuaternionFile(vector<SMRQuaternion> & _targetVector, vector<int> & _length,
                                                               vector<int> & _type,   vector<double> & _tension,
                                                                                      vector<double> & _bias, 
                                                                                      vector<double> & _continuity, string _filename)
{
  string source = "../../../win32Build/bin/Release/" + _filename;
  ifstream infile(source.c_str());
  if (infile.fail())
  {
    cout << "file " +  _filename + " not found" << endl;
    exit(1);
  }
  
  if (!expect("SERIES",infile)) return false;
  int series, size, style;
  double tension, bias, continuity;
  infile >> series;
  
  for ( int j = 0; j<series; j++)
  {
    if (!expect("SIZE",infile)) return false;
    infile >> size;
    _length.push_back(size);
    
    if (!expect("STYLE",infile)) return false;
    infile >> style;
    _type.push_back(style);
    
    for (int i = 0; i< size; i++)
    {
      if (!expect("QUATERNION",infile)) return false;
    
      double inputR,inputX,inputY,inputZ;
      infile >> inputR; infile >> inputX; infile >> inputY; infile >> inputZ;
    
      SMRQuaternion newQuaternion(inputR,inputX,inputY,inputZ);
      newQuaternion.normalize();
    
      _targetVector.push_back(newQuaternion);
    }
    if (!expect("TENSION",infile)) return false;
    infile >> tension;  
    _tension.push_back(tension);
    
    if (!expect("BIAS",infile)) return false;
    infile >>  bias;
    _bias.push_back(bias);
    
    if (!expect("CONTINUITY",infile)) return false;
    infile >> continuity;
    _continuity.push_back(continuity);
  }
  return true;
}


/**
* \brief Hemispherize a quaternionic time serie
*/
SMRTimeSerie<SMRQuaternion> hemispherize(const SMRTimeSerie<SMRQuaternion> & ts)
{
  SMRTimeSerie<SMRQuaternion> result;
  unsigned int cpt = 0;
  for (unsigned int i = 0 ; i < ts.getLength() ; i++){
    if (ts[i].m_w > 0)
      result.getSerie().push_back( ts[i] );
    else 
    {	
      cpt ++;
      result.getSerie().push_back( -ts[i]);
    }
  }
  //cout << "Number of inversed elements : " << cpt << endl;
  return result;
}

/**
* \brief get the quaternionic mean of a quaternionic time serie
*/
SMRQuaternion getQuaternionicMean(const SMRTimeSerie<SMRQuaternion> & ts)
{
  SMRQuaternion result, gradq;
  result = ts[0];
  do 
  {
    SMRTimeSerie<SMRQuaternion> gradvect;
    for (unsigned int i = 0 ; i < ts.getLength() ; i++)
      gradvect.getSerie().push_back(Inverse(result)*ts[i]);
    SMRVector3 sumgrad(0,0,0);
    for (unsigned int g = 0 ; g < gradvect.getLength() ; g++)
      sumgrad+=Log(gradvect[g]);
    sumgrad = sumgrad/gradvect.getLength();
    gradq = Exp(sumgrad);
    result = result * gradq;
  }
  while (Log(gradq).norm()>0.0001);
  return result;
}


/**
* \brief intialize all elements in the scene 
*/
static void InitScene(void)
{
  // ---------------------------------------------------------------------------------------
  // SMR Related stuff 
  // ---------------------------------------------------------------------------------------
  /// load a bvh motion
  motion = loadMotionFromBVH(getFileName("bvh/unknown.bvh"));
  /// cut the first 3 frames of the motion (T-pose)
  motion.cutMotionUntilFrame(3);
  /// switch every pose to absolute mode
  motion.changeMode(ABSOLUTEMODE);

  // fills up serie with random Gaussian quat
  for (unsigned int i = 0 ; i < 5000 ; i ++)
    serie1.add(SMRRandom::getRandomGaussianQuaternion(SMRQuaternion().identity(),covx,covy,covz,covalpha));
  serie=hemispherize(serie1);
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



static void drawQuaternion(const SMRQuaternion & q, const char*  _color = " ", float _pointSize = 4.0)
{
  SMRVector3 axis = q.getRotationAxis(); 
  double rot = q.getRotationAngle(); 
  glPointSize(_pointSize);
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

static void drawGeodesic(SMRVector3 & v, unsigned int step = 100){
  // first ensure that v is a unit vector
  v.normalize();
  double t = 2*M_PI/static_cast<double>(step); 
  for (unsigned int i = 0; i < step ; i++)
    drawQuaternion( Exp( v*(i*t) ) );
}

static void drawTimeSerie(SMRTimeSerie<SMRQuaternion>  _serie, const char* _color = " ", int _type = 0){
  
  if (_type == 1)
  // draw quaternions
    for (unsigned int i=0; i < _serie.getLength() ; i++)
      drawQuaternion(_serie[i],_color,4.0);
  else 
  {
    if (_type == 3)
    for (unsigned int i=0; i < _serie.getLength() ; i++)
      drawQuaternion(_serie[i],_color,5.0);   
    //draw path
    glLineWidth(1.5);
    glColor3d(0,0,0);
    glBegin(GL_LINE_STRIP);
    for (unsigned int i=0; i < _serie.getLength() ; i++)
    {
      SMRVector3 axis = _serie[i].getRotationAxis();
      glVertex3d(axis.m_x,axis.m_y,axis.m_z);
    }
    glEnd();
  }  
  
  glLineWidth(1);
}

static void drawQuatSet(SMRTimeSerie<SMRQuaternion>  _serie){
  // draw quaternions
  for (unsigned int i=0; i < _serie.getLength() ; i++)
    drawQuaternion(_serie[i]);
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
    string name = motion.getSkeleton(0).getJoint(cpt)->getName();
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
    vector<SMRQuaternion> quatVector;      //one vector contains all quaternions
    vector<int> length;                    
    vector<int> type;
    vector<double> tension;
    vector<double> bias;
    vector<double> continuity;
    
    //read file
    if (!(readQuaternionFile(quatVector,length,type,tension,bias,continuity,"quaternions.qat"))) exit(1);
    
    int numOfSeries = length.size();
    int index = 0;
    int offset = 0;
    double timeStep;
    vector<SMRTimeSerie<SMRQuaternion> > series;
    
    //calculate all series
    for ( int i = 0; i<numOfSeries; i++)
    {
      SMRTimeSerie<SMRQuaternion> currentSerie;
      // first segment
 
      timeStep = 1.0 / (GeodesicDistance(quatVector[offset],quatVector[offset+1]) * 40.0);
      for (double t= 0.0; t <= 1.0; t+= timeStep)
      {
        currentSerie.add(Interpolate(quatVector[offset],quatVector[offset],quatVector[offset+1],quatVector[offset+2],t,tension[i],bias[i],continuity[i]).normalize());
      }
      //inbetweens
      for (int j = 0; j < (length[i]-3); j++)
      {
        index = offset + j;
        timeStep = 1.0 / (GeodesicDistance(quatVector[index+1],quatVector[index+2])* 40.0 );
        for (double t= 0.0; t <= 1.0;t+= timeStep)
        {
          currentSerie.add(Interpolate(quatVector[index],quatVector[index+1],quatVector[index+2],quatVector[index+3],t,tension[i],bias[i],continuity[i]).normalize());
        }
      }
      //last segment
       timeStep = 1.0 /( GeodesicDistance(quatVector[index+2],quatVector[index+3]) *40.0);
      for (double t= 0.0; t <= 1.0;t+=timeStep)
      {
        currentSerie.add(Interpolate(quatVector[index+1],quatVector[index+2],quatVector[index+3],quatVector[index+3],t,tension[i],bias[i],continuity[i]).normalize());
      }
      
      offset += length[i];
      series.push_back(currentSerie);
    }
    
    //draw all series and quaternions
    for (unsigned int k = 0; k < quatVector.size(); k++)
      drawQuaternion(quatVector[k],"black",8.0);
    for (unsigned int j = 0; j < series.size(); j++)
      drawTimeSerie(series[j],  "black", type[j]);
  }
  else
    drawQuatSet(serie);	
    
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
    cpt = cpt!=(motion.getSkeleton(0).getNumJoints()-1) ? cpt+1:0;
    break;
  case '-':
    cpt = cpt!=0 ? cpt-1:motion.getSkeleton(0).getNumJoints()-1;
    break;
  case ' ':
    mode = mode ? false:true;
    break;
  case 'a':
    covx +=0.01;
    serie1.empty();
    // fills up serie with random Gaussian quat
    for (unsigned int i = 0 ; i < 5000 ; i ++)
      serie1.add(SMRRandom::getRandomGaussianQuaternion(SMRQuaternion().identity(),covx,covy,covz,covalpha));
    serie=hemispherize(serie1);
    break;
  case 'q':
    covx -=0.01;serie1.empty();
    // fills up serie with random Gaussian quat
    for (unsigned int i = 0 ; i < 5000 ; i ++)
      serie1.add(SMRRandom::getRandomGaussianQuaternion(SMRQuaternion().identity(),covx,covy,covz,covalpha));
    serie=hemispherize(serie1);
    break;
  case 'z':
    covy +=0.01;serie1.empty();
    // fills up serie with random Gaussian quat
    for (unsigned int i = 0 ; i < 5000 ; i ++)
      serie1.add(SMRRandom::getRandomGaussianQuaternion(SMRQuaternion().identity(),covx,covy,covz,covalpha));
    serie=hemispherize(serie1);
    break;
  case 's':
    covy -=0.01;serie1.empty();
    // fills up serie with random Gaussian quat
    for (unsigned int i = 0 ; i < 5000 ; i ++)
      serie1.add(SMRRandom::getRandomGaussianQuaternion(SMRQuaternion().identity(),covx,covy,covz,covalpha));
    serie=hemispherize(serie1);
    break;
  case 'e':
    covz +=0.01;serie1.empty();
    // fills up serie with random Gaussian quat
    for (unsigned int i = 0 ; i < 5000 ; i ++)
      serie1.add(SMRRandom::getRandomGaussianQuaternion(SMRQuaternion().identity(),covx,covy,covz,covalpha));
    serie=hemispherize(serie1);
    break;
  case 'd':
    covz -=0.01;serie1.empty();
    // fills up serie with random Gaussian quat
    for (unsigned int i = 0 ; i < 5000 ; i ++)
      serie1.add(SMRRandom::getRandomGaussianQuaternion(SMRQuaternion().identity(),covx,covy,covz,covalpha));
    serie=hemispherize(serie1);
    break;
  case 'r':
    covalpha +=0.01;serie1.empty();
    // fills up serie with random Gaussian quat
    for (unsigned int i = 0 ; i < 5000 ; i ++)
      serie1.add(SMRRandom::getRandomGaussianQuaternion(SMRQuaternion().identity(),covx,covy,covz,covalpha));
    serie=hemispherize(serie1);
    break;
  case 'f':
    covalpha -=0.01;serie1.empty();
    // fills up serie with random Gaussian quat
    for (unsigned int i = 0 ; i < 5000 ; i ++)
      serie1.add(SMRRandom::getRandomGaussianQuaternion(SMRQuaternion().identity(),covx,covy,covz,covalpha));
    serie=hemispherize(serie1);
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
  // program infos
  cout << "*** SMR Quaternion visualization Demo" << endl;
  cout << "*** ----------------------------- " << endl;
  cout << "there are two modes of visualization in this demo. One is the visualization of quaternionic time" << endl;
  cout << "series of a given motion. The other allows to visualize a sampling over quaternionic gaussian distribution" << endl;
  cout << "*** ----------------------------- Motion mode " << endl;
  cout << " <+,->: browse joints" << endl; 
  cout << "*** ----------------------------- Gaussian Sampling  " << endl;
  cout << " <a,q>: increase/decrease X covariance" << endl;
  cout << " <z,s>: increase/decrease Y covariance" << endl;
  cout << " <e,d>: increase/decrease Z covariance" << endl;
  cout << " <r,f>: increase/decrease Alpha covariance" << endl;
  cout << "*** ----------------------------- All " << endl;
  cout << " <space>: switch between modes" << endl; 
  cout << " <w>: wireframe mode" << endl;

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



  
  
  
