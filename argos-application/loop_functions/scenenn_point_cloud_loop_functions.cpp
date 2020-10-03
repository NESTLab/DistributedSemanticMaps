#include "scenenn_point_cloud_loop_functions.h"
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

CSceneNNPointCloudLoopFunctions::CSceneNNPointCloudLoopFunctions() {}

/* The format of the oriented bounding box is: box center (x, y, z), 
*  box dimension (along x, y, z), and a quaternion (x, y, z, w) that 
*  represents the rotation of the box.  Please note that the obbox 
*  are computed using eigen decomposition, so it is only accurate when
*  the object is elongated enough. */

void CSceneNNPointCloudLoopFunctions::Init(TConfigurationNode& t_node) {
    
    /* Get xml tags */ 
    TConfigurationNode& tPointCloud = GetNode(t_node, "point_cloud");

    /* Get filename of point cloud file */
    std::string strFile;
    GetNodeAttribute(tPointCloud, "input_file", strFile);
    /* Load xml file */
    ticpp::Document tInputFile;
    tInputFile.LoadFile(strFile);
    TConfigurationNode tFileRoot = *tInputFile.FirstChildElement();
    TConfigurationNodeIterator cCurrNode;
    /* Iterate through all nodes */
    int count = 0;
    for(cCurrNode = cCurrNode.begin(&tFileRoot);
          cCurrNode != cCurrNode.end(); ++cCurrNode)
    {
        if(cCurrNode->Value() == "label"){
            try {
                std::string strPCEntityId, strCategory, 
                            strColor, strBoundingBox, strPose;
                /* Get point cloud information */
                GetNodeAttribute(*cCurrNode, "id", strPCEntityId);
                GetNodeAttribute(*cCurrNode, "nyu_class", strCategory);
                GetNodeAttribute(*cCurrNode, "color", strColor);
                GetNodeAttribute(*cCurrNode, "obbox", strBoundingBox);
                // GetNodeAttribute(*cCurrNode, "aabbox", strBoundingBox);
                // GetNodeAttribute(*cCurrNode, "local_pose", strPose);

                /* Extract color vector */
                std::vector<UInt8> vecColor;
                SplitStringToUInt8(strColor, vecColor);

                if(strCategory == "floor" || strCategory == "ceiling" || 
                 strCategory == "unknown" ||strCategory == "") continue;

                /* Get enum category from string */
                // CPointCloudEntity::ECategory eCategory = CPointCloudEntity::categoryMap()[strCategory];

                /* Get bounding box coordinates */
                std::vector<Real> vecBox;
                SplitStringToReal(strBoundingBox, vecBox);

                if(vecBox[2] < 0.0 || vecBox[2] > 3.0) continue;

                LOG << strPCEntityId << " " << strCategory << " " << strBoundingBox 
                << " Color " << strColor << std::endl;

                /* Get pose quaternion components */
                // std::vector<Real> vecPose;
                // SplitStringToReal(strPose, vecPose);

                /* Create the new point cloud entity*/
                CPointCloudEntity* pcObject = new CPointCloudEntity(
                    strPCEntityId,
                    CVector3(vecBox[0], vecBox[1], vecBox[2]), //position
                    // CQuaternion(vecPose[0], vecPose[1], vecPose[2], vecPose[3]), // orientation
                    CQuaternion(vecBox[6], vecBox[7], vecBox[8], vecBox[9]), // orientation
                    CVector3(vecBox[3], vecBox[4], vecBox[5]), //size
                    strCategory, // category 
                    CColor(vecColor[0], vecColor[1], vecColor[2]) // color
                    );

                /* Add it to the simulation */
                AddEntity(*pcObject);
                /* Set physics model */
                CDynamics2DEngine* pcEngine = &dynamic_cast<CDynamics2DEngine&>(CSimulator::GetInstance().GetPhysicsEngine("dyn2d"));
                CDynamics2DPointCloudModel* pcModel = new CDynamics2DPointCloudModel(*pcEngine, *pcObject);
                pcObject->GetEmbodiedEntity().AddPhysicsModel(ToString("dyn2d"), *pcModel);
                /* Set medium */
                CPointCloudMedium* pcPointCloudMedium = &CSimulator::GetInstance().GetMedium<CPointCloudMedium>(ToString("point_clouds"));
                pcObject->SetMedium(*pcPointCloudMedium);

                m_pcPointClouds.push_back(pcObject);
                ++count;

            }
                catch(CARGoSException& ex) {
                    THROW_ARGOSEXCEPTION_NESTED("Error initializing point cloud", ex);
            }
        }
        // if (count > 30) break;
    }
}

void CSceneNNPointCloudLoopFunctions::SplitStringToUInt8(std::string str, std::vector<UInt8>& buffer) {
    std::vector<std::string> tokens;
    SplitString(str, tokens);
    for (auto token : tokens)
        buffer.push_back(std::atoi(token.c_str()));
}

void CSceneNNPointCloudLoopFunctions::SplitStringToReal(std::string str, std::vector<Real>& buffer) {
    std::vector<std::string> tokens;
    SplitString(str, tokens);
    std::vector<Real>res;
    for (auto token : tokens)
        buffer.push_back(std::atof(token.c_str()));
    
}

void CSceneNNPointCloudLoopFunctions::SplitString(std::string str, std::vector<std::string>& buffer) {
    std::vector<std::string> result; 
    std::istringstream iss(str); 
    for(std::string s; iss >> s; ) 
        buffer.push_back(s);
}


/****************************************/
/****************************************/

// void CSceneNNPointCloudLoopFunctions::Reset() {
//    ;
// }

// /****************************************/
// /****************************************/

// void CSceneNNPointCloudLoopFunctions::PreStep() {
//     ;
// }

/****************************************/
/****************************************/
void CSceneNNPointCloudLoopFunctions::PostStep() {
}

REGISTER_LOOP_FUNCTIONS(CSceneNNPointCloudLoopFunctions, "scenenn_point_cloud_loop_functions");