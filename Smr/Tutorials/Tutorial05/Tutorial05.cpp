#include "SmrUtils.h"
#include "Smr.h"
#include "SmrAnalyticalIKSolver.h"

#include "HordeApplication.h"
#include "HordeWindow.h" 
#include "HordeBones.h"


class Tutorial05:public HordeApplication
{
private:
  float m_dx,m_dy,m_dz;
  HordeResource m_envRes,m_boneRes,m_flyRes;
  HordeNode m_envNode,m_flyNode;
  HordeBones m_bones;
  SMRKinematicChain *m_kinematicChain;

  SMRIKConstraint *m_ikConstraint;
  SMRAnalyticalIKSolver *m_ikSolver;

public:
  Tutorial05(const std::string &appPath);
  void onLoadResources();
  void onAddNodes();
  void onUpdate();
  void createKinematicChain();
  void updateKinematicChain();
};

Tutorial05::Tutorial05(const std::string &appPath):HordeApplication(appPath)
{
  //set up camera parameters
  //m_camera.set(5,3,19,7,15,10);
}

void Tutorial05::onLoadResources()
{
  m_dx=m_dy=m_dz=0;

  //environment
  m_envRes.load("models/platform/platform.scene.xml");
  
  //fly
  m_flyRes.load("models/fly/fly.scene.xml");

  //bones
  m_bones.loadResource();
}

void Tutorial05::createKinematicChain()
{
  //instantiate an empty kinematic chain
  m_kinematicChain=new SMRKinematicChain(RELATIVEMODE,TRANSLATIONFIRST,"kinematicChain");

  SMRKinematicJoint *currentJoint,*prevJoint;
  prevJoint=NULL;

  for(int i=0;i<3;i++)
  {
    //instantiate a new SMRKinematicJoint
    currentJoint= (new SMRKinematicJoint());
  
    //give a name to the joint
    char name[100];
    sprintf(name,"joint%d",i);
    currentJoint->setName(name);

    //set up joint translation parameters in local frame (bone length)
    currentJoint->setPosition(0,2,0);

    //take care of parenting issues
    if(prevJoint) 
    {
      currentJoint->setParentName(prevJoint->getName());
    }

    currentJoint->addDOF( SMRDOF::YAXIS, 0, 0, 0, 0 );
    currentJoint->addDOF( SMRDOF::XAXIS, -M_PI/1.0, +M_PI/1.0, 0, 0 );
    currentJoint->addDOF( SMRDOF::YAXIS, 0, 0, 0, 0 );
    currentJoint->addDOF( SMRDOF::ZAXIS, -M_PI/1.0, +M_PI/1.0, 0, 0 );
    currentJoint->addDOF( SMRDOF::YAXIS, -M_PI/1.0, +M_PI/1.0, 0, 0 );

    //add the new joint into the kinematics chain
    m_kinematicChain->insertJoint(currentJoint);

    //keep a reference of the newly created joint for the next turn (parenting issue)
    prevJoint=currentJoint;
  }

  //SMRQuaternion rot;
  //rot.fromEulerAngles(M_PI/2,0.0,0.0);
  //m_kinematicChain->getJoint(1)->setOrientation(rot);
  //m_kinematicChain->getJoint(2)->setOrientation(rot);

  //set root joint
  m_kinematicChain->setStartJointIndex(m_kinematicChain->getJoint(1)->getName());

  //add the fly constraint (relative to tentacle's tip) to the tentacle
  m_ikConstraint=new SMRIKConstraint(SMRVector3(0,0,0),SMRVector3(0,0,0),currentJoint->getName());
}

void Tutorial05::onAddNodes()
{
  //add environment
  m_envNode.create(m_envRes);
  m_envNode.setTransform(0,0,0,0,0,0,0.23f,0.23f,0.23f);
  
  //add fly node
  m_flyNode.create(m_flyRes);
  m_flyNode.setTransform(0,0,0,0,0,0,0.2f,0.2f,0.2f);

  //create kinematic chain
  createKinematicChain();
  
  //create visual representation
  m_bones.create(m_kinematicChain, 0.30f);

  //create solver
  m_ikSolver=new SMRAnalyticalIKSolver(m_kinematicChain);
  m_ikSolver->addConstraintPtr(m_ikConstraint);
}

void Tutorial05::onUpdate()
{
  float tx,ty,tz;
  m_flyNode.getPosition(&tx,&ty,&tz);
  
  //roughly simulates the random fly of a fly
  if((rand()%100)>95)
  {
    m_dx=((SMRUtils::ranf()))/50.0f;
    m_dy=((SMRUtils::ranf()))/50.0f;
    m_dz=((SMRUtils::ranf()))/50.0f;
  }
  //change fly position
  m_flyNode.setPosition(tx+m_dx,ty+m_dy,tz+m_dz);
  if(tx<-1.0) m_dx=0.05f;
  if(tx>1.0) m_dx=-0.05f;
  if(ty<0.0) m_dy=0.05f;
  if(ty>4.0) m_dy=-0.05f;
  if(tz<-1.0) m_dz=0.05f;
  if(tz>1.0) m_dz=-0.05f;

  m_flyNode.getPosition(&tx,&ty,&tz);

  //update constraint position according to fly position
  m_ikConstraint->setPosition(SMRVector3(tx,ty,tz));

  //change fly position
/*  tx=sinf(m_time)*2;
  ty=sinf(m_time*0.5f)*2+5;
  tz=cosf(m_time)*2;

  m_flyNode.setPosition(tx,ty,tz);*/

  //update constraint position according to fly position
  //m_ikConstraint->setPosition(SMRVector3(tx,ty,tz));

  //compute inverse kinematics
  m_ikSolver->process();

  //update visual representations
  m_bones.update();
}

int main(int argc, char** argv)
{
  Tutorial05 *application=new Tutorial05(argv[0]); //will be deleted in HordeWindow::release
  HordeWindow::init((char*)"SMRTutorial05 - Analytical IK",640,480,false,application);
  HordeWindow::run();
  HordeWindow::release();
}