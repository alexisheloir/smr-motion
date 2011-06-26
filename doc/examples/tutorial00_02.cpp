int main()
{
  Tutorial00 *application=new Tutorial00();
  HordeWindow::init("SMRTutorial00 - Basic Application",800,600,false,application);
  HordeWindow::run();
  HordeWindow::release();
}