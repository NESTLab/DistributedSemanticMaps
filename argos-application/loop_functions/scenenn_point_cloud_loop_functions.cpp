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

static const std::string POINTCLOUD_TABLE[13] = {"bed",
                                    "bin",  
                                    "cabinet", 
                                    "chair", 
                                    "desk", 
                                    "display",
                                    "door", 
                                    "pillow", 
                                    "shelf", 
                                    "sink",
                                    "sofa", 
                                    "table",
                                    "toilet"};
static const CRange<UInt32> cIndexRange(0, 12);

static const std::string EASY_TABLE[6] = 
                                    {"bin",
                                    "cabinet", 
                                    "chair", 
                                    "door",
                                    "shelf",
                                    "sofa"};
static const CRange<UInt32> cEasyRange(0, 5);

static const std::string HARD_TABLE[6] = 
                                    {"bed",
                                    "toilet",
                                    "table",
                                    "desk",
                                    "pillow",
                                    "sink"};
static const CRange<UInt32> cHardRange(0, 5);


CSceneNNPointCloudLoopFunctions::CSceneNNPointCloudLoopFunctions() {}

/* The format of the oriented bounding box is: box center (x, y, z), 
*  box dimension (along x, y, z), and a quaternion (x, y, z, w) that 
*  represents the rotation of the box.  Please note that the obbox 
*  are computed using eigen decomposition, so it is only accurate when
*  the object is elongated enough. */

void CSceneNNPointCloudLoopFunctions::Init(TConfigurationNode& t_node) {
    
    /* Create file to write environment to */
    std::string strOutputFileName("environment_mixed.dat");
    m_ofOutputFile.open(strOutputFileName, std::ios_base::trunc | std::ios_base::out);


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

                /* Replace unknown label by other label at random */
                if(strCategory == "unknown")
                {
                    /* Make it any object at random */
                    CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
                    strCategory =  POINTCLOUD_TABLE[pcRNG->Uniform(cIndexRange)];
                }

                /* Ignore untrained classifier types and 
                 object types with prediction accuracy < 50% */
                if(CCategoryMap::StringToCategoryMap.count(strCategory) == 0 ||
                    strCategory == "box" || strCategory == "bag")
                {
                    LOG << "Not importing " << strCategory << std::endl;
                    continue;
                } 

                /* Get bounding box coordinates */
                std::vector<Real> vecBox;
                SplitStringToReal(strBoundingBox, vecBox);

                Real fSizeX = Min(vecBox[3], 0.7);
                Real fSizeY = Min(vecBox[4], 0.7);
                Real fSizeZ = Min(vecBox[5], 0.5);

                /* Ignore very small objects */
                if(fSizeX < 0.05 || fSizeY < 0.05 || fSizeZ < 0.05)
                {
                    LOG << "Not importing " << strCategory << " bc too small" << std::endl;
                    continue;
                } 

                LOG << strPCEntityId << " " << strCategory << " " << strBoundingBox 
                << " Color " << strColor << '\n';

                /* Get pose quaternion components */
                // std::vector<Real> vecPose;
                // SplitStringToReal(strPose, vecPose);


                /* Create the new point cloud entity*/
                CPointCloudEntity* pcObject = new CPointCloudEntity(
                    strPCEntityId,
                    CVector3(-5,-5,0), //position
                    // CQuaternion(vecPose[0], vecPose[1], vecPose[2], vecPose[3]), // orientation
                    CQuaternion(),//vecBox[6], vecBox[7], vecBox[8], vecBox[9]), // orientation
                    CVector3(fSizeX, fSizeY, fSizeZ), //size
                    strCategory, // category 
                    CColor(vecColor[0], vecColor[1], vecColor[2]) // color
                );


                /* Add it to the simulation */
                AddEntity(*pcObject);
                /* Set physics model */
                CDynamics2DEngine* pcEngine = &dynamic_cast<CDynamics2DEngine&>(CSimulator::GetInstance().GetPhysicsEngine("dyn2d"));
                CDynamics2DPointCloudModel* pcModel = new CDynamics2DPointCloudModel(*pcEngine, *pcObject);
                pcObject->GetEmbodiedEntity().AddPhysicsModel(ToString("dyn2d"), *pcModel);

                if(MoveEntity(pcObject->GetEmbodiedEntity(), CVector3(vecBox[0], vecBox[1], 0.), CQuaternion()))
                {
                    /* Write to file */
                    m_ofOutputFile << "<point_cloud id=\"" << strPCEntityId 
                                <<"\" size=\" " << fSizeX << "," << fSizeY << "," << fSizeZ
                                <<"\" category=\""<< strCategory
                                <<"\" color=\""<< vecColor[0] << "," << vecColor[1] << "," << vecColor[2] << ",1" <<"\"" << '\n'; 
                    m_ofOutputFile << "medium=\"point_clouds\">" << '\n';
                    m_ofOutputFile << "\t<body position=\""<< vecBox[0] << "," << vecBox[1] << "," << "0" <<"\" orientation=\"0,0,0\"/>" << '\n';
                    m_ofOutputFile << "</point_cloud>" << '\n';
                    m_ofOutputFile << '\n';
                }

                pcObject->CalculateFaceCorners();

                /* Set medium */
                CPointCloudMedium* pcPointCloudMedium = &CSimulator::GetInstance().GetMedium<CPointCloudMedium>(ToString("point_clouds"));
                pcObject->SetMedium(*pcPointCloudMedium);
                pcPointCloudMedium->AddEntity(*pcObject);
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