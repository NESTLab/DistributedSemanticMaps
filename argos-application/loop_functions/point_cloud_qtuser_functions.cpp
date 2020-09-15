#include "point_cloud_qtuser_functions.h"
#include <argos3/plugins/simulator/entities/point_cloud_entity.h>

/****************************************/
/****************************************/

CPointCloudQTUserFunctions::CPointCloudQTUserFunctions() {
   RegisterUserFunction<CPointCloudQTUserFunctions, CFootBotEntity>(&CPointCloudQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CPointCloudQTUserFunctions::Draw(CFootBotEntity& c_entity) {

   CCollectivePerception& cController = dynamic_cast<CCollectivePerception&>(c_entity.GetControllableEntity().GetController());
   // UInt32 unNodeKey = cController.GetNodeKey();
   // CKheperaPointCloud::TStoringQueue tStoringQueue = cController.GetStoringQueue();
   // CKheperaPointCloud::TRoutingQueue tRoutingQueue = cController.GetRoutingQueue();

   /* Draw RId */
   DrawText(CVector3(-0.1, 0, 0.1),   // position
            c_entity.GetId()); // text

   // /* Draw NodeKey */
   // CColor cColor;
   // UInt32 idx = unNodeKey / BUCKET_SIZE;
   // if (idx > 11) idx = 11;
   // cColor = COLOR_TABLE[idx];
   // DrawText(CVector3(-0.1, 0, 0.2),   // position
   //          (ToString(unNodeKey)).c_str(),
   //          cColor); // text

   // CVector3 cPos(0, 0, 0.3);
   // std::string strText;

   // CKheperaPointCloud::STuple sTuple;
   // /* Draw Stored Tuples Queue */
   // while(!tStoringQueue.empty())
   // {
   //    sTuple = tStoringQueue.back();
   //    strText = "(" + ToString(sTuple.Key.Prefix) + "|" 
   //               + ToString(sTuple.Key.Suffix) + ", " 
   //               + ToString(sTuple.Value) + ")";
   //    cColor = COLOR_TABLE[(sTuple.Key.Prefix - 1)/BUCKET_SIZE];
   //    DrawText(cPos,
   //          strText.c_str(),
   //          cColor);
   //    cPos += CVector3(0.0, 0.0, 0.1);
   //    tStoringQueue.pop_back();
   // }
   // if(!tRoutingQueue.empty())
   // {
   //   DrawText(cPos,
   //            (ToString("_________________")).c_str(),
   //            cColor);
   //   cPos += CVector3(0.0, 0.0, 0.1);
   // }
   // /* Draw Routing Queue */
   // while(!tRoutingQueue.empty())
   // {
   //    sTuple = tRoutingQueue.back();
   //    strText = "(" + ToString(sTuple.Key.Prefix) + "|" 
   //               + ToString(sTuple.Key.Suffix) + ", " 
   //               + ToString(sTuple.Value) + ")";
   //    cColor = COLOR_TABLE[(sTuple.Key.Prefix - 1)/BUCKET_SIZE];
   //    DrawText(cPos,
   //          strText.c_str(),
   //          cColor);
   //    cPos += CVector3(0.0, 0.0, 0.1);
   //    tRoutingQueue.pop_back();
   // }
}

void CPointCloudQTUserFunctions::DrawInWorld() {

   CSpace& cSpace = CSimulator::GetInstance().GetSpace();
   CSpace::TMapPerType& tPCMap = cSpace.GetEntitiesByType("point_cloud");

   for(CSpace::TMapPerType::iterator it = tPCMap.begin();
       it != tPCMap.end();
       ++it) {
      CPointCloudEntity& cPC = *any_cast<CPointCloudEntity*>(it->second);
      DrawText(cPC.GetEmbodiedEntity().GetOriginAnchor().Position + CVector3(.0,.0,.5),
               cPC.GetId());
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CPointCloudQTUserFunctions, "point_cloud_qtuser_functions")
