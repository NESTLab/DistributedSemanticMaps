#include "point_cloud_qtuser_functions.h"
#include <argos3/plugins/simulator/entities/point_cloud_entity.h>

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

/****************************************/
/****************************************/

void CPointCloudQTUserFunctions::Draw(CFootBotEntity& c_entity) {

   CCollectivePerception& cController = dynamic_cast<CCollectivePerception&>(c_entity.GetControllableEntity().GetController());

   /* Get controller variables */
   uint16_t unNodeID = cController.GetNodeID();
   std::vector<STuple> tStoringQueue = cController.m_cMySM.StoredTuples();
   std::vector<STuple> tRoutingQueue = cController.m_cMySM.RoutingTuples();
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
                 + ToString(sTuple.Value.Payload.Category) + ", (" 
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

   CSpace& cSpace = CSimulator::GetInstance().GetSpace();
   CSpace::TMapPerType& tPCMap = cSpace.GetEntitiesByType("point_cloud");

   for(CSpace::TMapPerType::iterator it = tPCMap.begin();
       it != tPCMap.end();
       ++it) {
      CPointCloudEntity& cPC = *any_cast<CPointCloudEntity*>(it->second);
      // DrawText(cPC.GetEmbodiedEntity().GetOriginAnchor().Position + CVector3(.0,.0,.5),
      //          cPC.GetId());

      for (auto corner : cPC.GetFrontCorners())
      {
         DrawPoint(corner);
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CPointCloudQTUserFunctions, "point_cloud_qtuser_functions")
