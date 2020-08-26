#include "point_cloud_loop_functions.h"
#include <sstream>
#include <vector>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/plugins/simulator/media/point_cloud_medium.h>
#include <argos3/plugins/simulator/entities/point_cloud_entity.h>

CPointCloudLoopFunctions::CPointCloudLoopFunctions() {}

/* The format of the oriented bounding box is: box center (x, y, z), 
*  box dimension (along x, y, z), and a quaternion (x, y, z, w) that 
*  represents the rotation of the box.  Please note that the obbox 
*  are computed using eigen decomposition, so it is only accurate when
*  the object is elongated enough. */

void CPointCloudLoopFunctions::Init(TConfigurationNode& t_node) {
    
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
                GetNodeAttribute(*cCurrNode, "aabbox", strBoundingBox);
                GetNodeAttribute(*cCurrNode, "local_pose", strPose);

                /* Extract color vector */
                std::vector<UInt8> vecColor;
                SplitStringToUInt8(strColor, vecColor);

                /* Get enum category from string */
                CPointCloudEntity::ECategory eCategory = CPointCloudEntity::categoryMap()[strCategory];

                /* Get bounding box coordinates */
                std::vector<Real> vecBox;
                SplitStringToReal(strBoundingBox, vecBox);

                /* Get pose quaternion components */
                std::vector<Real> vecPose;
                SplitStringToReal(strPose, vecPose);

                CPointCloudMedium cObject;
                // CPointCloudEntity cthing;

                /* Create the new point cloud entity*/
                // CPointCloudEntity* pcObject = new CPointCloudEntity(
                //     strPCEntityId,
                //     CVector3(vecBox[0], vecBox[1], vecBox[2]), //position
                //     CQuaternion(vecPose[0], vecPose[1], vecPose[2], vecPose[3]), // orientation
                //     CVector3(vecBox[3], vecBox[4], vecBox[5]), //size
                //     eCategory, // category 
                //     CColor(vecColor[0], vecColor[1], vecColor[2]) // color
                //     );
                // /* Add it to the simulation */
                // AddEntity(*pcObject);
                // m_pcPointClouds.push_back(pcObject);
            }
                catch(CARGoSException& ex) {
                    THROW_ARGOSEXCEPTION_NESTED("Error initializing point cloud", ex);
            }
        }
    }
}

void CPointCloudLoopFunctions::SplitStringToUInt8(std::string str, std::vector<UInt8>& buffer) {
    std::vector<std::string> tokens;
    SplitString(str, tokens);
    for (auto token : tokens)
        buffer.push_back(FromString<UInt8>(token));
}

void CPointCloudLoopFunctions::SplitStringToReal(std::string str, std::vector<Real>& buffer) {
    std::vector<std::string> tokens;
    SplitString(str, tokens);
    std::vector<Real>res;
    for (auto token : tokens)
        buffer.push_back(FromString<Real>(token));
}

void CPointCloudLoopFunctions::SplitString(std::string str, std::vector<std::string>& buffer) {
    std::vector<std::string> result; 
    std::istringstream iss(str); 
    for(std::string s; iss >> s; ) 
        buffer.push_back(s);
}


/****************************************/
/****************************************/

// void CPointCloudLoopFunctions::Reset() {
//    ;
// }

// /****************************************/
// /****************************************/

// void CPointCloudLoopFunctions::PreStep() {
//     ;
// }

// /****************************************/
// /****************************************/
// void CPointCloudLoopFunctions::PostStep() {
//     ;
// }

REGISTER_LOOP_FUNCTIONS(CPointCloudLoopFunctions, "point_cloud_loop_functions");