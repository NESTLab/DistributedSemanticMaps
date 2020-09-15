#ifndef POINT_CLOUD_QTUSER_FUNCTIONS_H
#define POINT_CLOUD_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/collective_perception_controller/collective_perception_controller.h>

using namespace argos;

class CPointCloudQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CPointCloudQTUserFunctions();

   virtual ~CPointCloudQTUserFunctions() {}

   void Draw(CFootBotEntity& c_entity);

   void DrawInWorld();
   
};

#endif
