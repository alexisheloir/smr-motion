#include "HordeWindow.h"
#include <stdlib.h>
#include <sstream>
#include <math.h>
#include <iomanip>
#include <iostream>

using namespace std;

char *HordeWindow::m_caption;
int HordeWindow::m_appWidth;
int HordeWindow::m_appHeight;
bool HordeWindow::m_fullScreen;
bool HordeWindow::m_running;
double HordeWindow::m_t0;
int HordeWindow::m_mx0,HordeWindow::m_my0;
HordeApplication *HordeWindow::m_application;
int HordeWindow::m_frames;
float HordeWindow::m_fps;

int GLFWCALL HordeWindow::windowCloseCallback()
{
  m_running=false;
  return 0;
}

void GLFWCALL HordeWindow::keyCallback(int key,int action)
{
  if(!m_running) return;

  if(action==GLFW_PRESS)
  {
    int width=m_appWidth,height=m_appHeight;
    
    switch(key)
    {
      case GLFW_KEY_ESC:
      {
        m_running=false;
      }
      break;
      case GLFW_KEY_SPACE:
      {
        m_application->onKeyPress(key);
      }
      break;
      case GLFW_KEY_F1:
      {
        m_application->release();
        glfwCloseWindow();
        
        //Toggle fullscreen mode
        m_fullScreen=!m_fullScreen;
        if(m_fullScreen)
        {
          GLFWvidmode mode;
          glfwGetDesktopMode(&mode);
          
          float aspect=mode.Width/(float)mode.Height;
          if((int)(aspect*100)==133||(int)(aspect*100)==125) //Standard
          {
            width=1280; height=1024;
          }
          else if((int)(aspect*100)==160) //Widescreen
          {
            width=1280; height=800;
          }
          else //Unknown
          {
            //Use desktop resolution
            width=mode.Width; height=mode.Height;
          }
        }
        
        if(!setupWindow(width,height,m_fullScreen))
        {
          glfwTerminate();
          exit(-1);
        }
        
        m_application->init();
        m_application->resize(width,height);
        m_t0=glfwGetTime();
      }
      break;
      default:
      {
        m_application->onKeyPress(key);
      }
      break;
    }
  }

  if(key>=0) m_application->onKeyStateChange(key,action==GLFW_PRESS);
}

void GLFWCALL HordeWindow::mousePosCallback(int x,int y)
{
  if(!m_running)
  {
    m_mx0=x; m_my0=y;
    return;
  }
  m_application->mouseMoveEvent((float)(x-m_mx0),(float)(m_my0-y));
  m_mx0=x; m_my0=y;
}

bool HordeWindow::setupWindow(int width,int height,bool fullscreen)
{
  //Create OpenGL window
  if(!glfwOpenWindow(width,height,8,8,8,8,24,8,fullscreen ? GLFW_FULLSCREEN : GLFW_WINDOW))
  {
    glfwTerminate();
    return false;
  }

  if(!fullscreen) glfwSetWindowTitle(m_caption);
  
  //Disable vertical synchronization
  glfwSwapInterval(0);

  //Set listeners
  glfwSetWindowCloseCallback(windowCloseCallback);
  glfwSetKeyCallback(keyCallback);
  glfwSetMousePosCallback(mousePosCallback);
  return true;
}

int HordeWindow::init(char *caption,int appWidth,int appHeight,bool fullScreen,HordeApplication *application)
{
  m_caption=caption;
  m_appWidth=appWidth;
  m_appHeight=appHeight;
  m_fullScreen=fullScreen;

  //Initialize GLFW
  glfwInit();
  if(!setupWindow(m_appWidth,m_appHeight,m_fullScreen)) return -1;
  
  //Initalize application and engine
  m_application=application;
  if(!m_application->init())
  {
    //Fake message box
    glfwCloseWindow();
    glfwOpenWindow(800,16,8,8,8,8,24,8,GLFW_WINDOW);
    glfwSetWindowTitle("Unable to initalize engine-Make sure you have an OpenGL 2.0 compatible graphics card");
    glfwSleep(5);
    
    std::cout<<"Unable to initalize engine"<<std::endl;
    std::cout<<"Make sure you have an OpenGL 2.0 compatible graphics card";
    glfwTerminate();
    h3dSetOption(H3DOptions::MaxLogLevel,0.0);
    return -1;
  }
  m_application->resize(m_appWidth,m_appHeight);
  glfwDisable(GLFW_MOUSE_CURSOR);

  m_frames=0;
  m_fps=30.0f;
  m_t0=glfwGetTime();
  m_running=true;

  return 0;
}

void HordeWindow::run()
{
  //Game loop
  while(m_running)
  {
    //Calc FPS
    ++m_frames;
    if(m_frames>=3)
    {
      double t=glfwGetTime();
      m_fps=m_frames/(float)(t-m_t0);
      m_frames=0;
      m_t0=t;
    }

    //Render
    m_application->mainLoop(m_fps);
    glfwSwapBuffers();
  }
}

void HordeWindow::release()
{
  glfwEnable(GLFW_MOUSE_CURSOR);
  //Quit
  m_application->release();
  delete m_application;
  glfwTerminate();
}
