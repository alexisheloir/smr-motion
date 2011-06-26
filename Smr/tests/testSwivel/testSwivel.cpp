// Module:  LOG4CPLUS
// File:    loggingserver.cxx
// Created: 5/2003
// Author:  Tad E. Smith
//
//
// Copyright (C) Tad E. Smith  All rights reserved.
//
// This software is published under the terms of the Apache Software
// License version 1.1, a copy of which has been included with this
// distribution in the LICENSE.APL file.
//
#include "SmrUtils.h"
#include "Smr.h"
#include "SmrSwivel.h"

#include "HordeApplication.h"
#include "HordeWindow.h"
#include "HordeBones.h"


class TestSwivel:public HordeApplication
{
  private:
    HordeResource m_envRes,m_boneRes;
    HordeBones m_bones;
    HordeNode m_envNode;
    SMRKinematicChain *m_kinematicChain;
    SMRSwivel *m_swivel;
    float m_angle;
  public:
    TestSwivel();
    ~TestSwivel();
    void release();
    void onLoadResources();
    void onAddNodes();
    void onUpdate();
    void createKinematicChain();
};

TestSwivel::TestSwivel():m_angle(0.0f)
{
}



TestSwivel::~TestSwivel()
{
}

void TestSwivel::release()
{
  delete(m_kinematicChain); // so that electric fence can check memory consistency
  delete(m_swivel);
  m_bones.clear();
  HordeApplication::release();
}

void TestSwivel::createKinematicChain()
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
    string name("joint");
    stringstream num("");
    num << i;
    name = name + num.str();

    currentJoint->setName(name);

    //set up joint translation parameters in local frame (bone length)
    currentJoint->setPosition(0,2,0);

    //take care of parenting issues
    if(prevJoint) 
    {
      currentJoint->setParentName(prevJoint->getName());
    }

    currentJoint->addDOF( SMRDOF::YAXIS, 0, 0, 0, 0 );
    currentJoint->addDOF( SMRDOF::XAXIS, -2*M_PI/1.0, +2*M_PI/1.0, 0, 0 );
    currentJoint->addDOF( SMRDOF::YAXIS, 0, 0, 0, 0 );
    currentJoint->addDOF( SMRDOF::ZAXIS, -2*M_PI/1.0, +2*M_PI/1.0, 0, 0 );
    currentJoint->addDOF( SMRDOF::YAXIS, -2*M_PI/1.0, +2*M_PI/1.0, 0, 0 );

    //add the new joint into the kinematics chain
    m_kinematicChain->insertJoint(currentJoint);

    //keep a reference of the newly created joint for the next turn (parenting issue)
    prevJoint=currentJoint;
  }

  SMRQuaternion rot;
  rot.fromEulerAngles(0.0,0.0,M_PI/2.0);
  m_kinematicChain->getJoint(1)->setOrientation(rot);

  //set root joint
  m_kinematicChain->setStartJointIndex(m_kinematicChain->getJoint(0)->getName());
}

void TestSwivel::onLoadResources()
{
  //environment
  m_envRes.load("models/platform/platform.scene.xml");

  //bones
  m_bones.loadResource();
}

void TestSwivel::onAddNodes()
{
  //add environment
  m_envNode.create(m_envRes);
  m_envNode.setTransform(0,0,0,0,0,0,0.23f,0.23f,0.23f);

  //create kinematic chain
  createKinematicChain();
  
  //create visual representation
  m_bones.create(m_kinematicChain, 0.30f);

  //create solver
  m_swivel=new SMRSwivel(m_kinematicChain,"joint0","joint2","joint1");
}

void TestSwivel::onUpdate()
{
  m_swivel->setAngle(m_angle);
  m_swivel->process();
  m_bones.update();
  m_angle += 0.003f;
  if (m_angle > 2*M_PI) m_angle=0.0f;
  // Show text
  stringstream angle;
  angle << "angle: " << m_angle;
  //Horde3DUtils::showText( angle.str().c_str(), 0, 0.90f, 0.03f, 0, m_fontMatRes );
}

int main()
{
  TestSwivel *application=new TestSwivel();
  HordeWindow::init("TestSwivel",800,600,false,application);
  HordeWindow::run();
  HordeWindow::release();
}
