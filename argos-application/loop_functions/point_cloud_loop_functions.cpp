#include "point_cloud_loop_functions.h"
#include <sstream>
#include <vector>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/plugins/simulator/media/point_cloud_medium.h>
#include <argos3/plugins/simulator/entities/point_cloud_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d_point_cloud_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

CPointCloudLoopFunctions::CPointCloudLoopFunctions() : m_unClock(0) {}

/* The format of the oriented bounding box is: box center (x, y, z), 
*  box dimension (along x, y, z), and a quaternion (x, y, z, w) that 
*  represents the rotation of the box.  Please note that the obbox 
*  are computed using eigen decomposition, so it is only accurate when
*  the object is elongated enough. */

void CPointCloudLoopFunctions::Init(TConfigurationNode& t_node) {
    
    // /* Get xml tags */ 
    // TConfigurationNode& tPointCloud = GetNode(t_node, "point_cloud");

    // /* Get filename of point cloud file */
    // std::string strFile;
    // GetNodeAttribute(tPointCloud, "input_file", strFile);
    // /* Load xml file */
    // ticpp::Document tInputFile;
    // tInputFile.LoadFile(strFile);
    // TConfigurationNode tFileRoot = *tInputFile.FirstChildElement();
    // TConfigurationNodeIterator cCurrNode;
    // /* Iterate through all nodes */
    // int count = 0;
    // for(cCurrNode = cCurrNode.begin(&tFileRoot);
    //       cCurrNode != cCurrNode.end(); ++cCurrNode)
    // {
    //     if(cCurrNode->Value() == "label"){
    //         try {
    //             std::string strPCEntityId, strCategory, 
    //                         strColor, strBoundingBox, strPose;
    //             /* Get point cloud information */
    //             GetNodeAttribute(*cCurrNode, "id", strPCEntityId);
    //             GetNodeAttribute(*cCurrNode, "nyu_class", strCategory);
    //             GetNodeAttribute(*cCurrNode, "color", strColor);
    //             GetNodeAttribute(*cCurrNode, "obbox", strBoundingBox);
    //             // GetNodeAttribute(*cCurrNode, "aabbox", strBoundingBox);
    //             // GetNodeAttribute(*cCurrNode, "local_pose", strPose);

    //             /* Extract color vector */
    //             std::vector<UInt8> vecColor;
    //             SplitStringToUInt8(strColor, vecColor);

    //             if(strCategory == "floor" || strCategory == "ceiling" || 
    //              strCategory == "unknown" ||strCategory == "") continue;

    //             /* Get enum category from string */
    //             // CPointCloudEntity::ECategory eCategory = CPointCloudEntity::categoryMap()[strCategory];

    //             /* Get bounding box coordinates */
    //             std::vector<Real> vecBox;
    //             SplitStringToReal(strBoundingBox, vecBox);

    //             if(vecBox[2] < 0.0 || vecBox[2] > 3.0) continue;

    //             LOG << strPCEntityId << " " << strCategory << " " << strBoundingBox 
    //             << " Color " << strColor << std::endl;

    //             /* Get pose quaternion components */
    //             // std::vector<Real> vecPose;
    //             // SplitStringToReal(strPose, vecPose);

    //             /* Create the new point cloud entity*/
    //             CPointCloudEntity* pcObject = new CPointCloudEntity(
    //                 strPCEntityId,
    //                 CVector3(vecBox[0], vecBox[1], vecBox[2]), //position
    //                 // CQuaternion(vecPose[0], vecPose[1], vecPose[2], vecPose[3]), // orientation
    //                 CQuaternion(vecBox[6], vecBox[7], vecBox[8], vecBox[9]), // orientation
    //                 CVector3(vecBox[3], vecBox[4], vecBox[5]), //size
    //                 strCategory, // category 
    //                 CColor(vecColor[0], vecColor[1], vecColor[2]) // color
    //                 );

    //             /* Add it to the simulation */
    //             AddEntity(*pcObject);
    //             /* Set physics model */
    //             CDynamics2DEngine* pcEngine = &dynamic_cast<CDynamics2DEngine&>(CSimulator::GetInstance().GetPhysicsEngine("dyn2d"));
    //             CDynamics2DPointCloudModel* pcModel = new CDynamics2DPointCloudModel(*pcEngine, *pcObject);
    //             pcObject->GetEmbodiedEntity().AddPhysicsModel(ToString("dyn2d"), *pcModel);
    //             /* Set medium */
    //             CPointCloudMedium* pcPointCloudMedium = &CSimulator::GetInstance().GetMedium<CPointCloudMedium>(ToString("point_clouds"));
    //             pcObject->SetMedium(*pcPointCloudMedium);

    //             m_pcPointClouds.push_back(pcObject);
    //             ++count;

    //         }
    //             catch(CARGoSException& ex) {
    //                 THROW_ARGOSEXCEPTION_NESTED("Error initializing point cloud", ex);
    //         }
    //     }
    //     // if (count > 30) break;
    // }
    std::string strOutputFileName("outputfile.dat");

    m_ofOutputFile.open(strOutputFileName, std::ios_base::trunc | std::ios_base::out);

    CSpace::TMapPerType& cRobots = GetSpace().GetEntitiesByType("foot-bot");
    for (CSpace::TMapPerType::iterator it = cRobots.begin(); it != cRobots.end(); it++) {
        CFootBotEntity* cRobot = any_cast<CFootBotEntity*>(it->second);
        CCollectivePerception* cController = &dynamic_cast<CCollectivePerception&>(cRobot->GetControllableEntity().GetController());
        m_vecControllers.push_back(cController);
        m_vecRobots.push_back(cRobot);
    }
    CSpace::TMapPerType& cPointClouds = GetSpace().GetEntitiesByType("point_cloud");
    for (CSpace::TMapPerType::iterator it = cPointClouds.begin(); it != cPointClouds.end(); it++) {
        CPointCloudEntity& cPointCloud = *any_cast<CPointCloudEntity*>(it->second);
        CVector3 cPos = cPointCloud.GetEmbodiedEntity().GetOriginAnchor().Position;
        SLocation sLocation = SLocation(cPos.GetX(), cPos.GetY(), cPos.GetZ());
        m_mapActualCategories[sLocation] = cPointCloud.GetCategory();
    }
    m_unNumRobots = cRobots.size();
}


void CPointCloudLoopFunctions::PlaceCluster(const CVector2& c_center,
                                            UInt32 un_pointClouds,
                                            Real f_density) {
   try {
    //   /* Calculate side of the region in which the robots are scattered */
    //   Real fHalfSide = Sqrt((FB_AREA * un_robots) / f_density) / 2.0f;
    //   CRange<Real> cAreaRange(-fHalfSide, fHalfSide);
    //   /* Place robots */
    //   UInt32 unTrials;
    //   CFootBotEntity* pcFB;
    //   std::ostringstream cFBId;
    //   CVector3 cFBPos;
    //   CQuaternion cFBRot;
    //   /* Create a RNG (it is automatically disposed of by ARGoS) */
    //   CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
    //   /* For each robot */
    //   for(size_t i = 0; i < un_robots; ++i) {
    //      /* Make the id */
    //      cFBId.str("");
    //      cFBId << "fb" << (i + un_id_start);
    //      /* Create the robot in the origin and add it to ARGoS space */
    //      pcFB = new CFootBotEntity(
    //         cFBId.str(),
    //         FB_CONTROLLER);
    //      AddEntity(*pcFB);
    //      /* Try to place it in the arena */
    //      unTrials = 0;
    //      bool bDone;
    //      do {
    //         /* Choose a random position */
    //         ++unTrials;
    //         cFBPos.Set(pcRNG->Uniform(cAreaRange) + c_center.GetX(),
    //                    pcRNG->Uniform(cAreaRange) + c_center.GetY(),
    //                    0.0f);
    //         cFBRot.FromAngleAxis(pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
    //                              CVector3::Z);
    //         bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
    //      } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
    //      if(!bDone) {
    //         THROW_ARGOSEXCEPTION("Can't place " << cFBId.str());
    //      }
    //   }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While placing point clouds in a cluster", ex);
   }
}


void CPointCloudLoopFunctions::SplitStringToUInt8(std::string str, std::vector<UInt8>& buffer) {
    std::vector<std::string> tokens;
    SplitString(str, tokens);
    for (auto token : tokens)
        buffer.push_back(std::atoi(token.c_str()));
}

void CPointCloudLoopFunctions::SplitStringToReal(std::string str, std::vector<Real>& buffer) {
    std::vector<std::string> tokens;
    SplitString(str, tokens);
    std::vector<Real>res;
    for (auto token : tokens)
        buffer.push_back(std::atof(token.c_str()));
    
}

void CPointCloudLoopFunctions::SplitString(std::string str, std::vector<std::string>& buffer) {
    std::vector<std::string> result; 
    std::istringstream iss(str); 
    for(std::string s; iss >> s; ) 
        buffer.push_back(s);
}


/****************************************/
/****************************************/

void CPointCloudLoopFunctions::Reset() {
   m_ofOutputFile.close();
   m_vecControllers.clear();
   m_vecRobots.clear();
}

void CPointCloudLoopFunctions::Destroy() {
    m_ofOutputFile.close();
}

// /****************************************/
// /****************************************/

void CPointCloudLoopFunctions::PreStep() {
    m_unClock = GetSpace().GetSimulationClock();
}

/****************************************/
/****************************************/
void CPointCloudLoopFunctions::PostStep() {
    // UInt16 unTotalMessages = 0;
    // m_ofOutputFile << m_unClock << ' ' << m_unNumRobots << '\n';
    // for (size_t i = 0; i < m_vecControllers.size(); i++) {
    //     unTotalMessages += m_vecControllers[i]->GetMessageCount();
    //     std::vector<SEventData>& vecVotingDecisions = m_vecControllers[i]->GetVotingDecisions();
    //     std::vector<CCollectivePerception::STimingInfo>& vecTimingInfo = m_vecControllers[i]->GetTimingInfo();
    //     m_ofOutputFile << m_vecControllers[i]->GetId() << ' ' << vecVotingDecisions.size() << '\n';

    //     for (int i = 0; i < vecVotingDecisions.size(); i++) {
    //         SEventData sVotingDecision = vecVotingDecisions[i];
    //         CCollectivePerception::STimingInfo sTimingInfo = vecTimingInfo[i];
    //         std::string strActualCategory = m_mapActualCategories[sVotingDecision.Location];
    //         m_ofOutputFile << sVotingDecision.Payload.Category << ' ' << 
    //             strActualCategory << ' ' << sVotingDecision.Payload.Radius << ' ' <<
    //             sTimingInfo.LastUpdate - sTimingInfo.Start << '\n';
    //     }

    //     m_vecControllers[i]->ClearVotingDecisions();
    //     m_vecControllers[i]->ClearTimingInfo();
    //     m_vecControllers[i]->SetMessageCount(0);
    // }
    // m_ofOutputFile << unTotalMessages << '\n'; 
    
}

void CPointCloudLoopFunctions::PostExperiment() {
    // for (size_t i = 0; i < m_vecControllers.size(); i++) {
        
    // }
}

REGISTER_LOOP_FUNCTIONS(CPointCloudLoopFunctions, "point_cloud_loop_functions");