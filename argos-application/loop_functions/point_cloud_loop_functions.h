#ifndef POINT_CLOUD_LOOP_FUNCTIONS_H
#define POINT_CLOUD_LOOP_FUNCTIONS_H

#include <iostream>
#include <fstream>

/* The controller */
#include <controllers/collective_perception_controller/collective_perception_controller.h>

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/configuration/tinyxml/tinyxml.h>
#include <argos3/plugins/simulator/entities/point_cloud_entity.h>

using namespace argos;

class CPointCloudLoopFunctions : public CLoopFunctions {

public:
   
   CPointCloudLoopFunctions();
   virtual ~CPointCloudLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void PreStep();
   virtual void PostStep();
   
   void PlaceCluster(const CVector2& c_center,UInt32 un_pointClouds,
                     Real f_density);
   virtual void Destroy();
   virtual void PostExperiment();

   void SplitString(std::string str, std::vector<std::string>& buffer);
   void SplitStringToReal(std::string str, std::vector<Real>& buffer);
   void SplitStringToUInt8(std::string str, std::vector<UInt8>& buffer);

private:

   std::vector<CPointCloudEntity*> m_pcPointClouds;
   std::vector<CCollectivePerception*> m_vecControllers;
   std::vector<CFootBotEntity*> m_vecRobots;

   std::ofstream m_ofOutputFile;
   UInt16 m_unClock;
};

#endif