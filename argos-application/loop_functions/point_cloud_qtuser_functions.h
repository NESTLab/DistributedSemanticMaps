#ifndef POINT_CLOUD_QTUSER_FUNCTIONS_H
#define POINT_CLOUD_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/collective_perception_controller/collective_perception_controller.h>
#include <argos3/plugins/simulator/entities/point_cloud_entity.h>
#include <string>
#include <unordered_map>
#include <unordered_set>

using namespace argos;

class CPointCloudQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CPointCloudQTUserFunctions();

   virtual ~CPointCloudQTUserFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   void Draw(CFootBotEntity& c_entity);

   void DrawInWorld();
   
private:

   std::unordered_map<std::string, CCollectivePerception*> m_mapControllers;
   std::vector<CPointCloudEntity*> m_vecPointClouds;
   std::unordered_set<std::string> m_setVotedPointClouds;
   std::unordered_set<std::string> m_setObservedPointClouds;
};

#endif
