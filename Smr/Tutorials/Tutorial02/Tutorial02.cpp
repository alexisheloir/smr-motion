#include "SmrUtils.h"
#include "Smr.h"
#include "SmrInvJacSolver.h"

#include "HordeWindow.h"
#include "HordeApplication.h"
#include "HordeBones.h"

class Tutorial02:public HordeApplication
{
private:
  float m_dx,m_dy,m_dz;
  HordeResource m_envRes,m_flyRes;
  HordeNode m_envNode,m_flyNode;
  HordeBones m_bones;
  SMRKinematicChain *m_kinematicChain;
  SMRIKConstraint *m_ikConstraint;
  SMRInvJacSolver *m_ikSolver;

public:
  Tutorial02(const std::string &appPath);
  void onLoadResources();
  void onAddNodes();
  void onUpdate();
  void createKinematicChain();
};

Tutorial02::Tutorial02(const std::string &appPath):HordeApplication(appPath)
{
  //set up camera parameters
  //m_camera.set(5,3,19,7,15,10);
}

void Tutorial02::onLoadResources()
{
  m_dx=m_dy=m_dz=0;

  //environment
  m_envRes.load("models/platform/platform.scene.xml");
  
  //fly
  m_flyRes.load("models/fly/fly.scene.xml");

  //bones
  m_bones.loadResource();
}

void Tutorial02::createKinematicChain()
{
  //currentJoint and previous joint referencing for relevant parenting when creating tentacles joints
  SMRKinematicJoint *currentJoint,*prevJoint;
  prevJoint=NULL;

  //instanciate an empty kinematic chain
  m_kinematicChain=new SMRKinematicChain(RELATIVEMODE,TRANSLATIONFIRST,"simpleChain");

  for(int i=0;i<6;i++)
  {
    //instanciate a new SMRKinematicJoint
    currentJoint= (new SMRKinematicJoint());

    string name = "joint"+i;
    currentJoint->setName(name);

    //add DOF along X axis
    currentJoint->addDOF(SMRDOF::XAXIS,-M_PI/2.0f,M_PI/2.0f,SMRUtils::degToRad(0));
    //add DOF along Y axis
    currentJoint->addDOF(SMRDOF::YAXIS,-M_PI/16.0f,M_PI/16.0f,SMRUtils::degToRad(0));
    //add DOF along Z axis
    currentJoint->addDOF(SMRDOF::ZAXIS,-M_PI/2.0f,M_PI/2.0f,SMRUtils::degToRad(0));
    
    //set up joint gain (the more distal,the more gain)
    currentJoint->getDOF(0)->setGain(0.015*i);
    currentJoint->getDOF(1)->setGain(0.015*i);
    currentJoint->getDOF(2)->setGain(0.015*i);

    //set up joint translation parameters in local frame (bone length)
    currentJoint->setPosition(0,1.0f,0);
    
    //take care of parenting issues
    if(prevJoint) 
    {
      currentJoint->setParentName(prevJoint->getName());
    }

    //add the new joint into the inverse kinematics chain
    m_kinematicChain->insertJoint(currentJoint);

    //keep a reference of the newly created joint for the next turn (parenting issue)
    prevJoint=currentJoint;
  }

  m_kinematicChain->setStartJointIndex(m_kinematicChain->getJoint(0)->getName());

  //force the root joint not to move (tentacle basis should be fix)
  m_kinematicChain->getJoint(0)->getDOF(0)->setUpperBound(0.0);
  m_kinematicChain->getJoint(0)->getDOF(0)->setLowerBound(0.0);
  m_kinematicChain->getJoint(0)->getDOF(1)->setUpperBound(0.0);
  m_kinematicChain->getJoint(0)->getDOF(1)->setLowerBound(0.0);
  m_kinematicChain->getJoint(0)->getDOF(2)->setUpperBound(0.0);
  m_kinematicChain->getJoint(0)->getDOF(2)->setLowerBound(0.0);

  //add the fly constraint (relative to tentacle's tip) to the tentacle
  m_ikConstraint=new SMRIKConstraint(SMRVector3(0,0,0),SMRVector3(0,0,0),currentJoint->getName());
}

void Tutorial02::onAddNodes()
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
  m_bones.create(m_kinematicChain, 0.15f);

  //create solver
  m_ikSolver=new SMRInvJacSolver(m_kinematicChain);
  m_ikSolver->addConstraintPtr(m_ikConstraint);
}

void Tutorial02::onUpdate()
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
  if(tx<-2.0) m_dx=0.05f;
  if(tx>2.0) m_dx=-0.05f;
  if(ty<2.0) m_dy=0.05f;
  if(ty>4.0) m_dy=-0.05f;
  if(tz<-2.0) m_dz=0.05f;
  if(tz>2.0) m_dz=-0.05f;

  m_flyNode.getPosition(&tx,&ty,&tz);

  //update constraint position according to fly position
  m_ikConstraint->setPosition(SMRVector3(tx,ty,tz));

  //compute inverse kinematics
  m_ikSolver->process();

  //update visual representations
  m_bones.update();
}

int main(int argc, char** argv)
{
  Tutorial02 *application=new Tutorial02(argv[0]); //will be deleted in HordeWindow::release
  HordeWindow::init((char*)"SMRTutorial02 - Inverse Kinematics",640,480,false,application);
  HordeWindow::run();
  HordeWindow::release();
}
