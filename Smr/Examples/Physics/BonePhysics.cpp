/**
Bone physical simulation
instanciate an environment object,
display a bone and a plane
set the plane as static object
the bone as dynamic oject
make the bone fall on the plane
**/

#include "btBulletDynamicsCommon.h"
#include "SmrUtils.h"
#include "Smr.h"

#include "HordeApplication.h"
#include "HordeWindow.h"
#include "HordeBones.h"

//#include "btAxisSweep3.h"
//#include "btIDebugDraw.h"

#define _NUM_INTERNAL_STATES 5

class DebugDrawer: public btIDebugDraw
{
public:
  DebugDrawer()
  {}
  ~DebugDrawer()
  {}

  void 	drawLine (const btVector3 &from, const btVector3 &to, const btVector3 &color)
  {    
    glBegin(GL_LINES);
      glColor3f(color.getX(), color.getY(), color.getZ());
      glVertex3f(from.getX(), from.getY(), from.getZ()); // origin of the line
      glVertex3f(to.getX(), to.getY(), to.getZ()); // ending point of the line
    glEnd( );
  }

  void 	drawContactPoint (const btVector3 &PointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime, const btVector3 &color){}

  void reportErrorWarning(const char *warningString){}
  void draw3dText(const btVector3 &location, const char *textString){}
  void setDebugMode(int debugMode){};
  int getDebugMode(void) const {return 1;}

};
//
class SMRRigidBody
{
public:
  SMRRigidBody()
  {
  }

  SMRRigidBody(float _x, float _y, float _z, btCollisionShape* _collisionShape, float _mass=0.0f, bool _isDynamic=false): m_collisionShape(_collisionShape), m_localInertia(btVector3(0,0,0)), m_isDynamic(_isDynamic), m_mass(_mass)
  {

    if (m_collisionShape && m_isDynamic)
    {
      m_collisionShape->calculateLocalInertia(m_mass,m_localInertia);
    }

    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(_x,_y,_z));

    m_motionState = new btDefaultMotionState(transform);
    m_rbInfo = new btRigidBody::btRigidBodyConstructionInfo(m_mass,m_motionState,m_collisionShape,m_localInertia);
    m_body = new btRigidBody(*m_rbInfo);
  }

  btCollisionShape* getCollisionShape(void)
  {
    return m_collisionShape;
  }

  btRigidBody* getRigidBody(void)
  {
    return m_body;
  }

  void setOrigin(float _x, float _y, float _z)
  {
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(_x,_y,_z));
    m_motionState->setWorldTransform(transform);
  }

protected: 
  btTransform* m_transform;
  btCollisionShape* m_collisionShape;
  btVector3 m_localInertia;
  bool m_isDynamic;
  float m_mass;
  btDefaultMotionState* m_motionState;
  btRigidBody::btRigidBodyConstructionInfo* m_rbInfo;
  btRigidBody* m_body;
};

class SMRRigidBone: public SMRRigidBody
{
public:
  SMRRigidBone()
  {
  }

  ~SMRRigidBone()
  {
    delete(m_collisionShape);
  }


  void pushConfiguration()
  {
    btRigidBody* body = m_body;

    m_body->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);

    btTransform trans;
    body->getMotionState()->getWorldTransform(trans);
    SMRQuaternion orientation( double(trans.getRotation().getW()), double(trans.getRotation().getX()), double(trans.getRotation().getY()), double(trans.getRotation().getZ()));
    orientation.normalize();
    SMRVector3 position(double(trans.getOrigin().getX()),double(trans.getOrigin().getY()),double(trans.getOrigin().getZ()));
    SMRVector3 offset(0.0,1.0,0.0);
    orientation.rotate(offset);
    m_relatedJoint->setPosition(position-offset);
    m_relatedJoint->setOrientation( orientation );
  }

  void pullConfiguration()
  {
    // first, declare the rigid bone as a kinematic body.
    int previousCollisionFlags = m_body->getCollisionFlags();
    m_body->setCollisionFlags(m_body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);

    //get the position of the kinematic bone
    SMRVector3 position = m_relatedJoint->getPosition();
    //get the orientation of the kinematic bone
    SMRQuaternion orientation = m_relatedJoint->getOrientation();

    //convert the SMR stuff into bullet stuff
    btTransform transform;
    btQuaternion quat(orientation.m_x,orientation.m_y,orientation.m_z,orientation.m_w);
    btVector3 pos(btScalar(position.m_x),btScalar(position.m_y),btScalar(position.m_z));

    transform.setRotation(quat);
    transform.setOrigin(pos);

    //what is this?
    m_motionState->setWorldTransform(transform);
    
  }

  SMRRigidBone(float _x, float _y, float _z, SMRJoint* _relatedJoint, float _mass=0.0f, bool _isDynamic=false):SMRRigidBody(_x,_y,_z, NULL, _mass,_isDynamic), m_relatedJoint(_relatedJoint)
  {

    m_collisionShape =  new btBoxShape(btVector3(0.3,btScalar(0.8),0.3));

    if (m_collisionShape && m_isDynamic)
    {
      //printf("zogzog\n");
      m_collisionShape->calculateLocalInertia(m_mass,m_localInertia);
    }

    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(_x,_y,_z));

    m_motionState = new btDefaultMotionState(transform);
    m_rbInfo = new btRigidBody::btRigidBodyConstructionInfo(m_mass,m_motionState,m_collisionShape,m_localInertia);
    m_body = new btRigidBody(*m_rbInfo);
  }

  btCollisionShape* getCollisionShape(void)
  {
    return m_collisionShape;
  }

  btRigidBody* getRigidBody(void)
  {
    return m_body;
  }

  void setOrigin(float _x, float _y, float _z)
  {
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(_x,_y,_z));
    m_motionState->setWorldTransform(transform);
  }

protected:
  SMRJoint *m_relatedJoint;
};

class SMRPhysicsEnvironment
{
public:
  SMRPhysicsEnvironment()
  {
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
    m_overlappingPairCache = new btDbvtBroadphase();
    m_solver = new btSequentialImpulseConstraintSolver;
    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_solver, m_collisionConfiguration);
    m_dynamicsWorld->setGravity(btVector3(0,-9,0));

  }

  void stepSimulation(float _step)
  {
    m_dynamicsWorld->stepSimulation(_step,_NUM_INTERNAL_STATES);
    //print positions of all objects
    for (int j=m_dynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--)
    {
      btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[j];
      btRigidBody* body = btRigidBody::upcast(obj);
      if (body && body->getMotionState())
      {
        btTransform trans;
        body->getMotionState()->getWorldTransform(trans);
        //printf("%f %f %f\n",float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()));
      }
    }
  }

  void addCollistionShape( btCollisionShape* _collisionShape)
  {
    m_collisionShapes.push_back(_collisionShape);
  }

  void addRigidBody(SMRRigidBody &_rigidBody)
  {
    addCollistionShape( _rigidBody.getCollisionShape() );
    m_dynamicsWorld->addRigidBody(_rigidBody.getRigidBody());  
  }

  void addRigidBody(SMRRigidBone &_rigidBone)
  {
    addCollistionShape( _rigidBone.getCollisionShape() );
    m_dynamicsWorld->addRigidBody(_rigidBone.getRigidBody());  
  }

  btDiscreteDynamicsWorld* m_dynamicsWorld;

protected:
  btDefaultCollisionConfiguration* m_collisionConfiguration;
  btCollisionDispatcher* m_dispatcher;
  btBroadphaseInterface* m_overlappingPairCache;
  btSequentialImpulseConstraintSolver* m_solver;
  btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
};


class TutorialPhysics:public HordeApplication
{
private:
  HordeResource m_envRes; 
  HordeResource m_boneRes;
  HordeNode m_envNode;
  HordeBones m_bones;
  SMRKinematicChain *m_kinematicChain;

  btCollisionShape* m_groundCollisionShape;
  btCollisionShape *m_boneCollisionShape;

  DebugDrawer m_debugDrawer;
  SMRPhysicsEnvironment m_environment;

public:
  SMRRigidBody *m_groundRigidBody;
  SMRRigidBone *m_boneRigidBody;
  SMRRigidBone *m_boneRigidBody1;
  TutorialPhysics( const std::string &appPath );
  ~TutorialPhysics();
  void release();
  void onLoadResources();
  void onAddNodes();
  void onUpdate();
  void createKinematicChain();
  void updateKinematicChain();
  void updateRigidBodies();
  void intermediateDraw();
};

TutorialPhysics::TutorialPhysics( const std::string &appPath ):HordeApplication(appPath)
{
  m_environment.m_dynamicsWorld->setDebugDrawer(&m_debugDrawer);

  //set up camera parameters
  //m_camera.set(5,3,19,7,15,10);

  m_groundCollisionShape = new btBoxShape(btVector3(btScalar(5.),btScalar(0.6),btScalar(5.)));
  m_boneCollisionShape = new btBoxShape(btVector3(btScalar(0.3),btScalar(1.0),btScalar(0.3)));

  m_groundRigidBody = new SMRRigidBody(0.0f, -0.4f, 0.0f, m_groundCollisionShape, 0.0f, false);

  m_environment.addRigidBody(*m_groundRigidBody);

}

TutorialPhysics::~TutorialPhysics()
{

}

void TutorialPhysics::release()
{
  delete(m_groundCollisionShape);
  delete(m_boneCollisionShape);
  delete(m_groundRigidBody);
  delete(m_boneRigidBody);
  delete(m_kinematicChain);
  m_bones.clear();
  HordeApplication::release();
}


void TutorialPhysics::onLoadResources()
{
  //environment
  m_envRes.load("models/platform/platform.scene.xml");
  
  //bones
  m_bones.loadResource();
}

void TutorialPhysics::createKinematicChain()
{
  //instantiate an empty kinematic chain
  m_kinematicChain=new SMRKinematicChain(RELATIVEMODE,TRANSLATIONFIRST,"kinematicChain");

  SMRKinematicJoint *currentJoint,*prevJoint;
  prevJoint=NULL;

  for(int i=0;i<2;i++)
  {
    //instantiate a new SMRKinematicJoint
    currentJoint = (new SMRKinematicJoint());
    currentJoint->addDOF(SMRDOF::XAXIS, 0, 2*M_PI, 0);
    currentJoint->addDOF(SMRDOF::YAXIS, 0, 2*M_PI, 0);
    currentJoint->addDOF(SMRDOF::ZAXIS, 0, 2*M_PI, 0);

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

    //add the new joint into the kinematics chain
    m_kinematicChain->insertJoint(currentJoint);

    //keep a reference of the newly created joint for the next turn (parenting issue)
    prevJoint=currentJoint;
  }

  m_kinematicChain->getJoint(0)->setPosition(0.0,10.0,0.0);
  m_kinematicChain->getJoint(0)->setOrientation(SMRQuaternion(SMRVector3(0,0,1),sinf(M_PI/4)));

  //set root joint
  m_kinematicChain->setStartJointIndex(m_kinematicChain->getJoint(0)->getName());

  //m_kinematicChain->setMode(ABSOLUTEMODE);

}

void TutorialPhysics::updateRigidBodies()
{
  m_kinematicChain->setMode(ABSOLUTEMODE);

  m_boneRigidBody->pullConfiguration();
  m_boneRigidBody1->pullConfiguration();

  m_kinematicChain->setMode(RELATIVEMODE);
}

void TutorialPhysics::updateKinematicChain()
{

  m_kinematicChain->setMode(ABSOLUTEMODE);

  m_boneRigidBody->pushConfiguration();
  m_boneRigidBody1->pushConfiguration();

  m_kinematicChain->setMode(RELATIVEMODE);

}

void TutorialPhysics::onAddNodes()
{
  //add environment
  m_envNode.create(m_envRes);
  m_envNode.setTransform(0,0,0,0,0,0,0.23f,0.23f,0.23f);
  
  //create kinematic chain
  createKinematicChain();

  m_boneRigidBody = new SMRRigidBone(0.0f, 10.0f, 0.0f, m_kinematicChain->getJoint(0), 0.8f, true);
  m_boneRigidBody1 = new SMRRigidBone(0.0f, 12.0f, 0.0f, m_kinematicChain->getJoint(1), 0.8f, true);

  m_environment.addRigidBody(*m_boneRigidBody);
  m_environment.addRigidBody(*m_boneRigidBody1);


  btVector3 pivotInA(0.0f, 1.0f, 0.0f);
	btVector3 axisInA(0,0,1);

	btVector3 pivotInB = m_boneRigidBody1->getRigidBody() ?
    m_boneRigidBody1->getRigidBody()->getCenterOfMassTransform().inverse()(m_boneRigidBody->getRigidBody()->getCenterOfMassTransform()(pivotInA)) :
    pivotInA;

  btVector3 axisInB = m_boneRigidBody1->getRigidBody()? 
			(m_boneRigidBody1->getRigidBody()->getCenterOfMassTransform().getBasis().inverse()*(m_boneRigidBody1->getRigidBody()->getCenterOfMassTransform().getBasis() * axisInA)) : 
		  m_boneRigidBody->getRigidBody()->getCenterOfMassTransform().getBasis() * axisInA;
  
  btTypedConstraint* hinge = new btHingeConstraint(*m_boneRigidBody->getRigidBody(),*m_boneRigidBody1->getRigidBody(),pivotInA,pivotInB,axisInA,axisInB);
  //btTypedConstraint* p2p = new btPoint2PointConstraint(*m_boneRigidBody->getRigidBody(),*m_boneRigidBody1->getRigidBody(),pivotInA,pivotInB);

  //m_environment.m_dynamicsWorld->addConstraint(p2p);
  m_environment.m_dynamicsWorld->addConstraint(hinge);

  this->updateRigidBodies();
  m_environment.stepSimulation(1.f/60.f);
  //create visual representation
  m_bones.create(m_kinematicChain, 0.30f);

}

void TutorialPhysics::onUpdate()
{

  m_environment.stepSimulation(1.f/60.f);
  //body->getMotionState()->getWorldTransform(trans);

  //update kinematic chain
  updateKinematicChain();

  //update visual representations
  m_bones.update();
}

void TutorialPhysics::intermediateDraw()
{
  m_environment.m_dynamicsWorld->debugDrawWorld();
}

int main(int argc, char** argv)
{

  TutorialPhysics *application=new TutorialPhysics( ( argv[0]) ); //will be deleted in HordeWindow::release
  char *title = (char*)"SMRPhysics tutorial - falling bone";
  HordeWindow::init(title,800,600,false,application);
  HordeWindow::run();
  HordeWindow::release();

}