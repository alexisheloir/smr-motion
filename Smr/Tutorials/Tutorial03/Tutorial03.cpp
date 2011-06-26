#include "SmrUtils.h"
#include "Smr.h"
#include "SmrInvJacSolver.h"

#include "HordeWindow.h"
#include "HordeApplication.h"
#include "HordeBones.h"

class Tutorial03:public HordeApplication
{
private:
  float m_dx,m_dy,m_dz;
  HordeResource m_envRes,m_flyRes,m_tentacleRes;
  HordeNode m_envNode,m_flyNode,m_tentacleNode;
  SMRKinematicChain *m_kinematicChain;
  SMRIKConstraint *m_ikConstraint;
  SMRInvJacSolver *m_ikSolver;

public:
  Tutorial03(const std::string &appPath);
  void onLoadResources();
  void onAddNodes();
  void onUpdate();
  void createKinematicChain();
  void updateTentacle();
};

Tutorial03::Tutorial03(const std::string &appPath):HordeApplication(appPath)
{
  //set up camera parameters
  //m_camera.set(5,3,19,7,15,10);
}

void Tutorial03::onLoadResources()
{
  m_dx=m_dy=m_dz=0;

  //environment
  m_envRes.load("models/platform/platform.scene.xml");
  
  //fly
  m_flyRes.load("models/fly/fly.scene.xml");

  //tentacle
  m_tentacleRes.load("models/tentacle/tentacle.scene.xml");
}

void Tutorial03::createKinematicChain()
{
  float tx,ty,tz,rx,ry,rz;
  //currentJoint and previous joint referencing for relevant parenting when creating tentacles joints
  SMRKinematicJoint *currentJoint,*prevJoint;
  prevJoint=NULL;
  //instanciate an empty kinematic chain
  m_kinematicChain=new SMRKinematicChain(RELATIVEMODE,TRANSLATIONFIRST,"tentacle");

  //get Hordes m_kinematicChain first joint
  HordeNode firstNode= m_tentacleNode.getFirstChild();
  //will help computing Smr joints gain 
  int i=0;
  //while Horde tentacles joints are found
  while(firstNode.isValid())
  {
    //instanciate a new SMRKinematicJoint
    currentJoint= (new SMRKinematicJoint());
    //get joint translation parameters in parents local frame (bone length)
    firstNode.getTransform(&tx,&ty,&tz,&rx,&ry,&rz);
    //add DOF along X axis
    currentJoint->addDOF(SMRDOF::XAXIS,-M_PI/16.0f,M_PI/16.0f,SMRUtils::degToRad(rx));
    //add DOF along Y axis
    currentJoint->addDOF(SMRDOF::YAXIS,-M_PI/4.0f,M_PI/4.0f,SMRUtils::degToRad(ry));
    //add DOF along Z axis
    currentJoint->addDOF(SMRDOF::ZAXIS,-M_PI/4.0f,M_PI/4.0f,SMRUtils::degToRad(rz));
    //set up joint gain (the more distal,the more gain)
    currentJoint->getDOF(0)->setGain(0.01*i);
    currentJoint->getDOF(1)->setGain(0.01*i);
    currentJoint->getDOF(2)->setGain(0.01*i);

    //give a name to the joint
    currentJoint->setName(firstNode.getName());
    //set up joint translation parameters in local frame (bone length)
    currentJoint->setPosition(tx,ty,tz);
    //take care of parenting issues
    if(prevJoint) 
    {
      currentJoint->setParentName(prevJoint->getName());
    }
    //and add the brand new joint into the inverse kinematics chain !
    m_kinematicChain->insertJoint(currentJoint);
    //keep a reference of the newly created joint for the next turn (parenting issue)
    prevJoint=currentJoint;
    //get the next joint according to Horde structure
    firstNode=firstNode.getFirstChild();
    //increase the gain increment
    i++;
  }
  //m_kinematicChain->setStartJointIndex(m_kinematicChain->getJoint(0)->getName());
  m_kinematicChain->setStartJointIndexInt(0);

  //force the root joint not to move (tentacle basis should be fix)
  m_kinematicChain->getJoint(0)->getDOF(0)->setUpperBound(0);
  m_kinematicChain->getJoint(0)->getDOF(0)->setLowerBound(0);
  m_kinematicChain->getJoint(0)->getDOF(1)->setUpperBound(0);
  m_kinematicChain->getJoint(0)->getDOF(1)->setLowerBound(0);
  m_kinematicChain->getJoint(0)->getDOF(2)->setUpperBound(0);
  m_kinematicChain->getJoint(0)->getDOF(2)->setLowerBound(0);

  //and add the fly constraint (relative to tentacle's tip) to the tentacle
  m_ikConstraint=new SMRIKConstraint(SMRVector3(0,0,0),SMRVector3(0,0,0),currentJoint->getName());
}

void Tutorial03::updateTentacle()
{
  //set of floats storing Horde's tentacle joints state
  float tx,ty,tz,rx,ry,rz,foo;
  double rrx,rry,rrz;

  //tentacle bone node(joint)
  HordeNode tentacleBone;

  // and its corresponding node in Smr
  SMRKinematicJoint* ikJoint;

  //get the transformation parameters from the SmrTentacle
  for(unsigned int i=0;i<m_kinematicChain->getNumJoints()-1;i++)
  {
    ikJoint=m_kinematicChain->getJoint(i);

    //get Horde tentacle's corresponding joint joint (Horde) 
    tentacleBone.findJoint(ikJoint->getName().c_str());
    tentacleBone.getTransform(&tx,&ty,&tz,&rx,&ry,&rz);

    //put the rotation values into euler angles (in degrees)
    ikJoint->getOrientation().toEulerAngles(rrx,rry,rrz);

    //update tentacle's joint according to Smr kinematic chain equivalent
    tentacleBone.setTransform(tx,ty,tz,SMRUtils::radToDeg(((float)(rrx))),SMRUtils::radToDeg(((float)(rry))),SMRUtils::radToDeg(((float)(rrz))));
  }
  // orientate Horde tentacles root joint correctly (because the tentacle collada model is not properly exported in Blender)
  ikJoint=m_kinematicChain->getJoint(0);
  tentacleBone.findJoint(ikJoint->getName().c_str());

  tentacleBone.getTransform(&tx,&ty,&tz,&foo,&ry,&rz);
  tentacleBone.setTransform(ty,ty,tz,90,ry,rz);
}

void Tutorial03::onAddNodes()
{
  //add environment
  m_envNode.create(m_envRes);
  m_envNode.setTransform(0,0,0,0,0,0,0.23f,0.23f,0.23f);
  
  //add fly node
  m_flyNode.create(m_flyRes);
  m_flyNode.setTransform(0,4,0,0,0,0,0.2f,0.2f,0.2f);

  //add tentacle
  m_tentacleNode.create(m_tentacleRes);

  //create kinematic chain
  createKinematicChain();

  //create solver
  m_ikSolver=new SMRInvJacSolver(m_kinematicChain);
  m_ikSolver->addConstraintPtr(m_ikConstraint);
}

void Tutorial03::onUpdate()
{
  float tx,ty,tz;
  m_flyNode.getPosition(&tx,&ty,&tz);
  
  //roughly simulates the random fly of a fly
  if((rand()%100)>95)
  {
    m_dx=((SMRUtils::ranf()))/100.0f;
    m_dy=((SMRUtils::ranf()))/100.0f;
    m_dz=((SMRUtils::ranf()))/100.0f;
  }
  //change fly position
  if(tx<-1.0) m_dx=  0.01f;
  if(tx>1.0)  m_dx= -0.01f;
  if(ty<1.0)  m_dy=  0.01f;
  if(ty>2.2)  m_dy= -0.01f;
  if(tz<-1.0) m_dz=  0.01f;
  if(tz>1.0)  m_dz= -0.01f;
  m_flyNode.setPosition(tx+m_dx,ty+m_dy,tz+m_dz);

  m_flyNode.getPosition(&tx,&ty,&tz);

  SMRQuaternion rotCorrection;
  rotCorrection.fromEulerAngles(-M_PI/2.0, 0.0, 0.0);
  //update constraint position according to fly position
  SMRVector3 targetPosition(tx,ty,tz);
  rotCorrection.rotate(targetPosition);  
  m_ikConstraint->setPosition(targetPosition);

  //compute inverse kinematics
  m_ikSolver->process();

  //update tentacle's joint (IK)
  updateTentacle();
}

int main(int argc, char** argv)
{
  Tutorial03 *application=new Tutorial03(argv[0]); //will be deleted in HordeWindow::release
  HordeWindow::init((char*)"SMRTutorial03 - Skinning",640,480,false,application);
  HordeWindow::run();
  HordeWindow::release();
}
