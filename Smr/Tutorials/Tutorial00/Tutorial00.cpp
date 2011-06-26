#include "SmrUtils.h"
#include "HordeWindow.h"
#include "HordeApplication.h"

class Tutorial00:public HordeApplication
{
private:
  HordeResource m_envRes;
  HordeNode m_envNode;

public:
  Tutorial00(const std::string &appPath);
  void onLoadResources();
  void onAddNodes();
};

Tutorial00::Tutorial00(const std::string &appPath):HordeApplication(appPath)
{
  //set up camera parameters
  //m_camera.set(5,3,19,7,15,10);
}

void Tutorial00::onLoadResources()
{
  //environment
  m_envRes.load("models/platform/platform.scene.xml");
}

void Tutorial00::onAddNodes()
{
  //add environment
  m_envNode.create(m_envRes);
  m_envNode.setTransform(0,0,0,0,0,0,0.23f,0.23f,0.23f);
}

int main(int argc, char** argv)
{
  Tutorial00 *application=new Tutorial00(argv[0]); //will be deleted in HordeWindow::release
  HordeWindow::init((char*)"SMRTutorial00 - Basic Application",640,480,false,application);
  HordeWindow::run();
  HordeWindow::release();
}
