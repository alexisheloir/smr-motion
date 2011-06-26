#include "SmrUtils.h"
#include "Smr.h"

#include "HordeWindow.h"
#include "HordeApplication.h"
#include "HordeBones.h"

class Tutorial01:public HordeApplication
{
private:
  HordeResource m_envRes,m_boneRes;
  HordeNode m_envNode;
  HordeBones m_bones;
  SMRKinematicChain *m_kinematicChain;

public:
  Tutorial01(const std::string &appPath);
  void onLoadResources();
  void onAddNodes();
  void onUpdate();
  void createKinematicChain();
  void updateKinematicChain();
};

Tutorial01::Tutorial01(const std::string &appPath):HordeApplication(appPath)
{
  //set up camera parameters
  //m_camera.set(5,3,19,7,15,10);
}

void Tutorial01::onLoadResources()
{
  //environment
  m_envRes.load("models/platform/platform.scene.xml");
  
  //bones
  m_bones.loadResource();
}

void Tutorial01::createKinematicChain()
{
  //instantiate an empty kinematic chain
  m_kinematicChain=new SMRKinematicChain(RELATIVEMODE,TRANSLATIONFIRST,"kinematicChain");

  SMRKinematicJoint *currentJoint,*prevJoint;
  prevJoint=NULL;

  for(int i=0;i<4;i++)
  {
    //instantiate a new SMRKinematicJoint
    currentJoint = (new SMRKinematicJoint());
  
    //give a name to the joint
    char name[100];
    sprintf(name,"joint%d",i);
    //currentJoint->setName(name+i);

    //set up joint translation parameters in local frame (bone length)
    currentJoint->setPosition(0,2,0);

    //take care of parenting issues
    if(prevJoint) 
    {
      currentJoint->setParentName(prevJoint->getName());
    }

    //add the new joint into the kinematics chain
    m_kinematicChain->insertJoint(currentJoint);

    //keep a reference of the newly created joint for the next turn (parenting issue)
    prevJoint=currentJoint;
  }

  //set root joint
  m_kinematicChain->setStartJointIndex(m_kinematicChain->getJoint(0)->getName());
}

void Tutorial01::updateKinematicChain()
{
  SMRKinematicJoint* joint;

  //rotate first joint
  joint=m_kinematicChain->getJoint(0);
  joint->setOrientation(SMRQuaternion(SMRVector3(1,0,0),sinf(m_time)));

  //rotate second joint
  joint=m_kinematicChain->getJoint(1);
  joint->setOrientation(SMRQuaternion(SMRVector3(1,0,0),sinf(m_time)));

  //rotate third joint
  joint=m_kinematicChain->getJoint(2);
  joint->setOrientation(SMRQuaternion(SMRVector3(1,0,0),sinf(m_time)));
}

void Tutorial01::onAddNodes()
{
  //add environment
  m_envNode.create(m_envRes);
  m_envNode.setTransform(0,0,0,0,0,0,0.23f,0.23f,0.23f);
  
  //create kinematic chain
  createKinematicChain();
  
  //create visual representation
  m_bones.create(m_kinematicChain, 0.30f);
}

void Tutorial01::onUpdate()
{
  //update kinematic chain
  updateKinematicChain();

  //update visual representations
  m_bones.update();
}

int main(int argc, char** argv)
{
  Tutorial01 *application=new Tutorial01(argv[0]); //will be deleted in HordeWindow::release
  HordeWindow::init((char*)"SMRTutorial01 - Kinematic Chain",800,600,false,application);
  HordeWindow::run();
  HordeWindow::release();
}
