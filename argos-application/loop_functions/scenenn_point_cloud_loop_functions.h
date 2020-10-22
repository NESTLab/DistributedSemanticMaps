#ifndef SCENENN_POINT_CLOUD_LOOP_FUNCTIONS_H
#define SCENENN_POINT_CLOUD_LOOP_FUNCTIONS_H

/* The controller */
#include <controllers/collective_perception_controller/collective_perception_controller.h>

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/configuration/tinyxml/tinyxml.h>
#include <argos3/plugins/simulator/entities/point_cloud_entity.h>
#include <fstream>

using namespace argos;

class CSceneNNPointCloudLoopFunctions : public CLoopFunctions {

public:
   
   CSceneNNPointCloudLoopFunctions();
   virtual ~CSceneNNPointCloudLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset() {}
   virtual void PreStep() {}
   virtual void PostStep();

   void SplitString(std::string str, std::vector<std::string>& buffer);
   void SplitStringToReal(std::string str, std::vector<Real>& buffer);
   void SplitStringToUInt8(std::string str, std::vector<UInt8>& buffer);

private:

   std::vector<CPointCloudEntity*> m_pcPointClouds;
   std::ofstream m_ofOutputFile;

};

#endif