class Tutorial00:public HordeApplication
{
private:
  HordeResource m_envRes;
  HordeNode m_envNode;

public:
  Tutorial00();
  void onLoadResources();
  void onAddNodes();
};