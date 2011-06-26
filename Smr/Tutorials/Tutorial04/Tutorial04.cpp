#include "SmrUtils.h"
#include "Smr.h"

#include "HordeApplication.h"
#include "HordeWindow.h"
#include "HordeBones.h"

class Tutorial04:public HordeApplication
{
private:
  HordeResource m_envRes;
  HordeNode m_envNode;
  HordeBones m_bones;
  SMRMotion m_motion;

  unsigned int m_frameIndex;
  int m_framePause;
  int m_speed;

public:
  Tutorial04(const std::string &appPath);
  ~Tutorial04();
  void onLoadResources();
  void onAddNodes();
  void onUpdate();
};

Tutorial04::Tutorial04(const std::string &appPath):HordeApplication(appPath)
{
  //set up camera parameters
  //m_camera.set(5,3,19,7,15,10);

  m_frameIndex=0;
  m_framePause=0;
  m_speed=1;
}

Tutorial04::~Tutorial04()
{
  cout << "That's all Folks!!!" << endl;
}


void Tutorial04::onLoadResources()
{
  //environment
  m_envRes.load("models/platform/platform.scene.xml");
  
  //bones
  m_bones.loadResource();
}

void Tutorial04::onAddNodes()
{
  //add environment
  m_envNode.create(m_envRes);
  m_envNode.setTransform(0,0,0,0,0,0,0.23f,0.23f,0.23f);
  
  //load acclaim skeleton and motion file
  m_motion=loadMotionFromAcclaim(getFileName("../../../Data/acclaim/08.asf"),getFileName("../../../Data/acclaim/08_normal_walk.amc"));
  
  //get a reference skeleton to create the visual bone representation
  SMRSkeleton skeleton=m_motion.getSkeleton(1);

  //set the rotation order to translation first
  skeleton.setRotationOrder(TRANSLATIONFIRST);

  //create the bones for visually representing the skeleton
  m_bones.create(&skeleton,0.15f);
}

void Tutorial04::onUpdate()
{
  //increment frame counter
  m_framePause++;
  if(m_framePause>m_speed)
  {
    m_framePause=0;
    m_frameIndex++;
  }
  if(m_frameIndex>=m_motion.getNumFrames()) m_frameIndex=0;

  //get skeleton of current frame
  SMRSkeleton skeleton=m_motion.getSkeleton(m_frameIndex);
  skeleton.setRotationOrder(TRANSLATIONFIRST);

  //update visual representations
  m_bones.update(&skeleton);
}

int main(int argc, char** argv)
{
  Tutorial04 *application=new Tutorial04(argv[0]); //will be deleted in HordeWindow::release
  HordeWindow::init((char*)"SMRTutorial04 - Loading Motion Files",640,480,false,application);
  HordeWindow::run();
  HordeWindow::release();
}