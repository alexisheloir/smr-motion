#ifndef HORDEWINDOW_H
#define HORDEWINDOW_H

#include "GL/glfw.h"
#include "Horde3D.h"
#include "Horde3DUtils.h"

#include "HordeApplication.h"

/**
* \class HordeWindow
* \brief Tutorial class.
**/
class HordeWindow
{
public:
  static char *m_caption;
  static int m_appWidth;
  static int m_appHeight;
  static bool m_fullScreen;
  static bool m_running;
  static double m_t0;
  static int m_mx0,m_my0;
  static HordeApplication *m_application;
  static int m_frames;
  static float m_fps;

private:
  static int GLFWCALL windowCloseCallback();
  static void GLFWCALL keyCallback(int key,int action);
  static void GLFWCALL mousePosCallback(int x,int y);
  static bool setupWindow(int width,int height,bool fullscreen);

public:
  static int init(char *caption,int appWidth,int appHeight,bool fullScreen,HordeApplication *application);
  static void run();
  static void release();
};

#endif

