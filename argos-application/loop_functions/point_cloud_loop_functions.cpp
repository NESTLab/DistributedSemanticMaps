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

    UInt16 unMinVotes, unSeed, unStorageMemory, unRoutingMemory, unHashing; 

    /* Parse the controller parameters */
    TConfigurationNode& tConfRoot = GetSimulator().GetConfigurationRoot();
    TConfigurationNode& tControllers = GetNode(tConfRoot, "controllers");
    TConfigurationNode& tController = GetNode(tControllers, "collective_perception_controller");
    TConfigurationNode& tParams = GetNode(tController, "params");
    GetNodeAttribute(tParams, "min_votes", unMinVotes);
    GetNodeAttribute(tParams, "storage", unStorageMemory);
    GetNodeAttribute(tParams, "routing", unRoutingMemory);
    GetNodeAttribute(tParams, "bucket", unHashing);

    /* Get random seed */
    unSeed = CSimulator::GetInstance().GetRandomSeed();

    /* Get robot entities and their number */
    CSpace::TMapPerType& cRobots = GetSpace().GetEntitiesByType("foot-bot");
    m_unNumRobots = cRobots.size();

    std::string strOutputFileName = "outputfile_" 
                                  + ToString(unMinVotes) + "_"
                                  + ToString(m_unNumRobots) + "_"
                                  + ToString(unSeed) + "_"
                                  + ToString(unStorageMemory) + "_"
                                  + ToString(unRoutingMemory) + "_"
                                  + ToString(unHashing) + ".dat";

    std::string strHistogramFileName = "histogramfile_" 
                                    + ToString(unMinVotes) + "_"
                                    + ToString(m_unNumRobots) + "_"
                                    + ToString(unSeed) + "_"
                                    + ToString(unStorageMemory) + "_"
                                    + ToString(unRoutingMemory) + "_"
                                    + ToString(unHashing) + ".dat";

    std::string strVcsbppFileName = "vcsbppfile_" 
                                  + ToString(unMinVotes) + "_"
                                  + ToString(m_unNumRobots) + "_"
                                  + ToString(unSeed) + "_"
                                  + ToString(unStorageMemory) + "_"
                                  + ToString(unRoutingMemory) + "_"
                                  + ToString(unHashing) + ".dat";

    m_ofOutputFile.open(strOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_ofHistogramFile.open(strHistogramFileName, std::ios_base::trunc | std::ios_base::out);
    m_ofVcsbppFile.open(strVcsbppFileName, std::ios_base::trunc | std::ios_base::out);
    m_unStorageCapacity = 0;
    m_unRoutingCapacity = 0;


    for (CSpace::TMapPerType::iterator it = cRobots.begin(); it != cRobots.end(); it++) {
        CFootBotEntity* cRobot = any_cast<CFootBotEntity*>(it->second);
        CCollectivePerception* cController = &dynamic_cast<CCollectivePerception&>(cRobot->GetControllableEntity().GetController());
        m_vecControllers.push_back(cController);
        cController->SetNumStoredTuples(0);
        m_unRoutingCapacity += cController->GetRoutingCapacity();
        m_unStorageCapacity += cController->GetStorageCapacity();
        cController->ResetBytesSent();
        m_vecRobots.push_back(cRobot);
    }
    m_ofHistogramFile << cRobots.size() << '\n';
    CSpace::TMapPerType& cPointClouds = GetSpace().GetEntitiesByType("point_cloud");
    for (CSpace::TMapPerType::iterator it = cPointClouds.begin(); it != cPointClouds.end(); it++) {
        CPointCloudEntity& cPointCloud = *any_cast<CPointCloudEntity*>(it->second);
        CVector3 cPos = cPointCloud.GetEmbodiedEntity().GetOriginAnchor().Position;
        SLocation sLocation = SLocation(cPos.GetX(), cPos.GetY(), cPos.GetZ());
        m_mapActualCategories[sLocation] = cPointCloud.GetCategory();
    }
    m_ofOutputFile << cPointClouds.size() << '\n';


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
    UInt16 unTotalMessages = 0;
    m_ofOutputFile << m_unClock << ' ' << m_unNumRobots << '\n';
    m_ofVcsbppFile << m_unClock << ' ' << m_unNumRobots << '\n';
    m_ofHistogramFile << m_unClock << '\n';

    UInt32 unTotalConsolidations = 0;
    UInt32 unTotalTuples = 0;

    std::unordered_set<std::string> setObservedCategories;

    for (size_t i = 0; i < m_vecControllers.size(); i++) {
        unTotalMessages += m_vecControllers[i]->GetMessageCount();
        unTotalTuples += m_vecControllers[i]->GetNumStoredTuples();

        std::vector<SEventData>& vecVotingDecisions = m_vecControllers[i]->GetVotingDecisions();
        std::vector<CCollectivePerception::STimingInfo>& vecTimingInfo = m_vecControllers[i]->GetTimingInfo();

        m_ofOutputFile << m_vecControllers[i]->GetId() << ' ' << m_vecControllers[i]->GetBytesSent() << ' ' << 
            vecVotingDecisions.size() << '\n';
        m_vecControllers[i]->ResetBytesSent();
        m_ofHistogramFile << m_vecControllers[i]->GetNodeID() << ' ';

        m_ofVcsbppFile << m_vecControllers[i]->GetId() << ' ' << m_vecControllers[i]->GetNodeID()
        << ' ' << m_vecControllers[i]->GetTuples().size();

        m_ofVcsbppFile << ' ' << m_vecControllers[i]->GetNeighbors().size() << '\n';

        for (int i = 0; i < vecVotingDecisions.size(); i++) {
            SEventData sVotingDecision = vecVotingDecisions[i];
            CCollectivePerception::STimingInfo sTimingInfo = vecTimingInfo[i];
            std::string strActualCategory = m_mapActualCategories[sVotingDecision.Location];
            if (m_mapVotedCategories.find(sVotingDecision.Location) == m_mapVotedCategories.end()) {
                m_mapVotedCategories[sVotingDecision.Location] = sVotingDecision.Payload.Category;
            }

            m_ofOutputFile << sVotingDecision.Payload.Category << ' ' << 
                strActualCategory << ' ' << sVotingDecision.Payload.Radius << ' ' <<
                sTimingInfo.LastUpdate - sTimingInfo.Start << ' ' <<
                sVotingDecision.Location.X << ' ' << sVotingDecision.Location.Y << 
                ' ' << sVotingDecision.Location.Z << '\n';
        }

        std::vector<STuple> vecTuples = m_vecControllers[i]->GetTuples();
        m_ofHistogramFile << vecTuples.size() << '\n';

        for (STuple sTuple : vecTuples) {
            m_setObservedCategories.insert(sTuple.Value.Location);
            m_ofHistogramFile << sTuple.Key.Identifier << ' ' << sTuple.Key.Hash << '\n';
            if (sTuple.Value.Type == "collective_label") {
                unTotalConsolidations++;
            }
        }
        // m_vecControllers[i]->ClearVotingDecisions(); // cleared in controller, using it in qtuser loop fcts
        m_vecControllers[i]->ClearTimingInfo();
        m_vecControllers[i]->SetMessageCount(0);
        m_vecControllers[i]->SetNumStoredTuples(0);
    }
    float fLoad = unTotalTuples / static_cast<float>(m_unStorageCapacity);
    m_ofOutputFile << fLoad << ' ' << m_setObservedCategories.size() << '\n'; 
    // std::cout << unTotalConsolidations << ' ' << m_mapVotedCategories.size() << ' ' << m_mapActualCategories.size() << '\n';
}

/****************************************/
/****************************************/

bool CPointCloudLoopFunctions::IsExperimentFinished() {
    if(m_mapVotedCategories.size() == NUM_POINT_CLOUDS || m_unClock > MAX_TIME)
        return true;
    return false;
}

/****************************************/
/****************************************/

void CPointCloudLoopFunctions::PostExperiment() {
    
}

REGISTER_LOOP_FUNCTIONS(CPointCloudLoopFunctions, "point_cloud_loop_functions");