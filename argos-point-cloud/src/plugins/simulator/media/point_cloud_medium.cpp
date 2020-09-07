#include "point_cloud_medium.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>
#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

   /****************************************/
   /****************************************/

   // CPointCloudMedium::CPointCloudMedium() {
   // }

   /****************************************/
   /****************************************/

   // CPointCloudMedium::~CPointCloudMedium() {
   // }

   /****************************************/
   /****************************************/

   void CPointCloudMedium::Init(TConfigurationNode& t_tree) {
      try {
         CMedium::Init(t_tree);
         /* Get the positional index method */
         std::string strPosIndexMethod("grid");
         GetNodeAttributeOrDefault(t_tree, "index", strPosIndexMethod, strPosIndexMethod);
         /* Get the arena center and size */
         CVector3 cArenaCenter;
         CVector3 cArenaSize;
         TConfigurationNode& tArena = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "arena");
         GetNodeAttribute(tArena, "size", cArenaSize);
         GetNodeAttributeOrDefault(tArena, "center", cArenaCenter, cArenaCenter);
         /* Create the positional index for point cloud entities */
         if(strPosIndexMethod == "grid") {
            size_t punGridSize[3];
            if(!NodeAttributeExists(t_tree, "grid_size")) {
               punGridSize[0] = cArenaSize.GetX();
               punGridSize[1] = cArenaSize.GetY();
               punGridSize[2] = cArenaSize.GetZ();
            }
            else {
               std::string strPosGridSize;
               GetNodeAttribute(t_tree, "grid_size", strPosGridSize);
               ParseValues<size_t>(strPosGridSize, 3, punGridSize, ',');
            }
            CGrid<CPointCloudEntity>* pcGrid = new CGrid<CPointCloudEntity>(
               cArenaCenter - cArenaSize * 0.5f, cArenaCenter + cArenaSize * 0.5f,
               punGridSize[0], punGridSize[1], punGridSize[2]);
            m_pcPointCloudEntityGridUpdateOperation = new CPointCloudEntityGridUpdater(*pcGrid);
            pcGrid->SetUpdateEntityOperation(m_pcPointCloudEntityGridUpdateOperation);
            m_pcPointCloudEntityIndex = pcGrid;
         }
         else {
            THROW_ARGOSEXCEPTION("Unknown method \"" << strPosIndexMethod << "\" for the positional index.");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error in initialization of the point cloud medium", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPointCloudMedium::PostSpaceInit() {
      Update();
   }

   /****************************************/
   /****************************************/

   void CPointCloudMedium::Reset() {
      m_pcPointCloudEntityIndex->Reset();
   }

   /****************************************/
   /****************************************/

   void CPointCloudMedium::Destroy() {
      delete m_pcPointCloudEntityIndex;
      if(m_pcPointCloudEntityGridUpdateOperation != NULL) {
         delete m_pcPointCloudEntityGridUpdateOperation;
      }
   }

   /****************************************/
   /****************************************/

   void CPointCloudMedium::Update() {
      /* Update positional index of point cloud entities */
      m_pcPointCloudEntityIndex->Update();

   }

   /****************************************/
   /****************************************/

   void CPointCloudMedium::AddEntity(CPointCloudEntity& c_entity) {
      m_pcPointCloudEntityIndex->AddEntity(c_entity);
   }

   /****************************************/
   /****************************************/

   void CPointCloudMedium::RemoveEntity(CPointCloudEntity& c_entity) {
      m_pcPointCloudEntityIndex->RemoveEntity(c_entity);
   }

   /****************************************/
   /****************************************/

   REGISTER_MEDIUM(CPointCloudMedium,
                   "point_cloud",
                   "Nathalie Majcherczyk [nathalie.majcherczyk@gmail.com]",
                   "1.0",
                   "Manages the point clouds.",
                   "This medium is required to manage the point cloud entities, thus allowing the\n"
                   "associated camera sensors to work properly. If you intend to use a camera\n"
                   "sensor that detects point clouds, you must add this medium to the XML\n"
                   "configuration file.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "<point_cloud id=\"point_cloud\" />\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "None for the time being\n",
                   "Under development"
      );

   /****************************************/
   /****************************************/

}