#include "point_cloud_qtuser_functions.h"

static const CColor COLOR_TABLE[16] = {CColor::GRAY10,
                                       CColor::WHITE,
                                       CColor::RED,
                                       CColor::GREEN,
                                       CColor::BLUE,
                                       CColor::MAGENTA,
                                       CColor::CYAN,
                                       CColor::YELLOW,
                                       CColor::ORANGE,
                                       CColor::BROWN,
                                       CColor::PURPLE,
                                       CColor::GRAY50,
                                       CColor::GRAY40,
                                       CColor::GRAY30,
                                       CColor::GRAY20,
                                       CColor::GRAY80};

/****************************************/
/****************************************/

CPointCloudQTUserFunctions::CPointCloudQTUserFunctions() {
   RegisterUserFunction<CPointCloudQTUserFunctions, CFootBotEntity>(&CPointCloudQTUserFunctions::Draw);
}

void CPointCloudQTUserFunctions::Init(TConfigurationNode& t_tree) {

   CSpace& cSpace = CSimulator::GetInstance().GetSpace();
   CSpace::TMapPerType& cRobots = cSpace.GetEntitiesByType("foot-bot");
   for (CSpace::TMapPerType::iterator it = cRobots.begin(); it != cRobots.end(); it++) {
      CFootBotEntity* cRobot = any_cast<CFootBotEntity*>(it->second);
      CCollectivePerception* pcController = &dynamic_cast<CCollectivePerception&>(cRobot->GetControllableEntity().GetController());
      m_mapControllers[cRobot->GetId()] = pcController;
   }

   CSpace::TMapPerType& tPCMap = cSpace.GetEntitiesByType("point_cloud");
   for(CSpace::TMapPerType::iterator it = tPCMap.begin();
       it != tPCMap.end();
       ++it) {
      CPointCloudEntity* cPC = any_cast<CPointCloudEntity*>(it->second);
      m_vecPointClouds.push_back(cPC);
   }

}

/****************************************/
/****************************************/

void CPointCloudQTUserFunctions::Draw(CFootBotEntity& c_entity) {

   CCollectivePerception* cController = m_mapControllers[c_entity.GetId()];

   /* Get controller variables */
   uint16_t unNodeID = cController->GetNodeID();
   std::vector<STuple> tStoringQueue = cController->m_cMySM.StoredTuples();
   std::vector<STuple> tRoutingQueue = cController->m_cMySM.RoutingTuples();
   // std::unordered_map<uint32_t, std::unordered_map<std::string, std::any>> mapQueries = cController.m_cMySM.m_mapQueries;

   /* Draw RId */
   DrawText(CVector3(-0.1, 0, 0.1),   // position
            c_entity.GetId()); // text

   /* Draw NodeID */
   CColor cColor;
   uint16_t idx = unNodeID / BUCKET_SIZE;
   if (idx > 15) idx = 15;
   cColor = COLOR_TABLE[idx];
   DrawText(CVector3(-0.1, 0, 0.2),   // position
            (ToString(unNodeID)).c_str(),
            cColor, QFont("Calibri", 18, QFont::Bold)); // text

   CVector3 cPos(0, 0, 0.3);
   std::string strText;
   STuple sTuple;

   /* Draw Stored Tuples Queue */
   while(!tStoringQueue.empty())
   {
      sTuple = tStoringQueue.back();
      strText = "[" + ToString(sTuple.Key.Hash) + ", " 
                 + ToString(sTuple.Value.Type) + ", (" 
                 + ToString(sTuple.Value.Location.X) + ","
                 + ToString(sTuple.Value.Location.Y) + ","
                 + ToString(sTuple.Value.Location.Z) + ")"
                 + "]";
      cColor = COLOR_TABLE[(sTuple.Key.Hash - 1)/BUCKET_SIZE];
      DrawText(cPos,
            strText.c_str(),
            cColor);
      cPos += CVector3(0.0, 0.0, 0.1);
      tStoringQueue.pop_back();
   }
   if(!tRoutingQueue.empty())
   {
     DrawText(cPos,
              (ToString("_________________")).c_str(),
              cColor);
     cPos += CVector3(0.0, 0.0, 0.1);
   }

   /* Draw Routing Queue */
   while(!tRoutingQueue.empty())
   {
      sTuple = tRoutingQueue.back();
      strText = "[" + ToString(sTuple.Key.Hash) + ", " 
                 + ToString(sTuple.Value.Payload.Category) + "]";
      cColor = COLOR_TABLE[(sTuple.Key.Hash - 1)/BUCKET_SIZE];
      DrawText(cPos,
            strText.c_str(),
            cColor);
      cPos += CVector3(0.0, 0.0, 0.1);
      tRoutingQueue.pop_back();
   }

   /* Draw Requests */
   /* TO DO ?*/
}

void CPointCloudQTUserFunctions::DrawInWorld() {

   std::unordered_set<SLocation, SLocation> setVotedLocations;
   std::unordered_set<SLocation, SLocation> setObservedLocations;

   /* Get all new consolidation votes and observations */
   for(auto it = m_mapControllers.begin(); it != m_mapControllers.end(); ++it)
   {
      CCollectivePerception* pcController = it->second;
      std::vector<SEventData> vecVotes = pcController->GetVotingDecisions();
      std::vector<SEventData> vecObs = pcController->GetObservations();
      for(auto event : vecVotes)
      {
         setVotedLocations.insert(SLocation(event.Location.X, event.Location.Y, 0.0));
      }
      for (auto obs : vecObs)
      {
         setObservedLocations.insert(SLocation(obs.Location.X, obs.Location.Y, 0.0));
      }
   }

   for(auto cPC : m_vecPointClouds) {
      CVector3 cPosition = cPC->GetEmbodiedEntity().GetOriginAnchor().Position;
      SLocation sLoc(cPosition.GetX(), cPosition.GetY(), 0.0);

      /* Change color of voted point cloud*/
      // if(m_setVotedPointClouds.count(cPC->GetId()))
      // {
      //    DrawRay(CRay3(cPC->GetFrontCorners()[0], cPC->GetFrontCorners()[3]), CColor::RED, 5.0);
      //    DrawRay(CRay3(cPC->GetFrontCorners()[1], cPC->GetFrontCorners()[2]), CColor::RED, 5.0);
      // }
      // /* Only check if point not already in list of observed point clouds */
      // else if (setVotedLocations.count(sLoc))
      if(!m_setVotedPointClouds.count(cPC->GetId()) && setVotedLocations.count(sLoc))
      {
         m_setVotedPointClouds.emplace(cPC->GetId());
         cPC->SetColor(CColor::BLACK);
      }

      /* Same for observations*/
      if(!m_setObservedPointClouds.count(cPC->GetId()) && setObservedLocations.count(sLoc))
      {
         m_setObservedPointClouds.emplace(cPC->GetId());
         cPC->SetColor(CColor::GRAY70);
      }

      DrawText(cPosition + CVector3(.0,.0,.4),
               cPC->GetId());
      DrawText(cPosition + CVector3(.0,.0,.2),
               cPC->GetCategory());

      for (auto corner : cPC->GetFrontCorners())
      {
         DrawPoint(corner, CColor::WHITE);
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CPointCloudQTUserFunctions, "point_cloud_qtuser_functions")
