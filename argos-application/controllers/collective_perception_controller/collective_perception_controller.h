#ifndef COLLECTIVE_PERCEPTION_CONTROLLER_H
#define COLLECTIVE_PERCEPTION_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_camera_sensor.h>

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>

#include <swarmmesh.h>

#include <queue>
#include <sstream>
#include <unordered_map>
#include <algorithm>
#include <functional>

/* TODO: make this configurable */
const uint16_t BUCKET_SIZE = 5;
const uint16_t RECORDING_TIMEOUT = 30;
const uint16_t QUERY_TIMEOUT = 200;
const uint16_t UPDATE_TIMEOUT = 10;
const float CONSOLIDATION_QUOTA = 2;
const float NOISE_THRESHOLD = 0.3;

/****************************************/
/****************************************/

class CMySwarmMesh;
class CCollectivePerception;
class CHashEventDataType;
class CTypeFilter;
class CLocationFilter;

/****************************************/
/****************************************/

using namespace argos;

/* Structure representing the point cloud */
struct SPointCloud {

   // /* Center of point cloud */
   // SLocation Center;

   /* Radius of sphere enclosing bounding box */
   float Radius;
   /* Object category */
   std::string Category;

   /* Default constructor */
   SPointCloud() {}

   /* Parameterized constructor */
   SPointCloud(float radius,
   std::string cat) : Category(cat), Radius(radius) {}

   /* Copy operator */
   SPointCloud& operator=(const SPointCloud s_pc) {
      Radius = s_pc.Radius;
      Category = s_pc.Category;
   }
};


/* Structure representing location */
struct SLocation {
   float X;
   float Y;
   float Z;

   /* Default constructor */
   SLocation() = default;

   /**
    * Construct a new SLocation object
    * 
    * @param f_x 
    * @param f_y 
    * @param f_z 
    */
   SLocation(float f_x, float f_y, float f_z) : X(f_x), Y(f_y), Z(f_z) {}

   /* Copy operator */
   SLocation& operator=(const SLocation& s_location) {
      X = s_location.X;
      Y = s_location.Y;
      Z = s_location.Z;
      return *this;
   }

   /* Hashing location */
   size_t operator()(const SLocation& s_location) const {
      auto hashX = std::hash<float>{}(X);
      auto hashY = std::hash<float>{}(Y);
      auto hashZ = std::hash<float>{}(Z);
      return hashX ^ hashY ^ hashZ;
   }

   /* Equality operator */
   bool operator==(const SLocation& s_location) const {
      return s_location.X == X && s_location.Y == Y && s_location.Z == Z;
   }

};

/* Structure representing events */
struct SEventData {
   /* Type of event */
   std::string Type;
   /* Value associated to the event */
   SPointCloud Payload;
   /* Spatial location of the event */
   SLocation Location;
};

typedef typename swarmmesh::CSwarmMesh<SEventData>::STuple STuple;

/****************************************/
/****************************************/

SEventData UnpackEventDataType(const std::vector<uint8_t>& vec_buffer, size_t& un_offset);

void PackEventDataType(std::vector<uint8_t>& vec_buffer, const SEventData& s_value);

/****************************************/
/****************************************/

class CHashEventDataType {

   private:
   uint16_t m_unRobotId = 0;
   uint16_t m_unTupleCount = 0;
   
   public:
      CHashEventDataType() : 
         m_unRobotId(0),
         m_unTupleCount(0) {}

      void Init(uint16_t un_robot_id) {m_unRobotId = un_robot_id;}
      swarmmesh::SKey operator()(SEventData& s_value);
};

class CMySwarmMesh : public swarmmesh::CSwarmMesh<SEventData> {
private:
   CHashEventDataType m_cHashEvent;

public:
   CMySwarmMesh() :
      CSwarmMesh(UnpackEventDataType,
                 PackEventDataType) {
                    RegisterFilter<CTypeFilter>(this);
                    RegisterFilter<CLocationFilter>(this);
                 }
   void Init(uint16_t un_robot_id, uint16_t un_msgSize);
   
   ~CMySwarmMesh() {
   }

};

/****************************************/
/****************************************/
/* User Defined Filters */

/**
 * Functor to filter tuples according to their type
 * 
 */
class CTypeFilter : public swarmmesh::CSwarmMesh<SEventData>::CFilterOperation {
   private:
      std::string m_strEventType;
   public: 
      CTypeFilter(swarmmesh::CSwarmMesh<SEventData>* pc_sm) : 
         swarmmesh::CSwarmMesh<SEventData>::CFilterOperation(pc_sm) {}
      
      ~CTypeFilter() {}
      bool operator()(const swarmmesh::CSwarmMesh<SEventData>::STuple&) override;
      void Serialize(std::vector<uint8_t>&) override;
      size_t Deserialize(const std::vector<uint8_t>&, size_t) override;
      void Init(const std::unordered_map<std::string, std::any>&) override;
      std::unordered_map<std::string, std::any> GetParams() override;
};

/**
 * Functor to filter tuples according to their location
 * 
 */
class CLocationFilter : public swarmmesh::CSwarmMesh<SEventData>::CFilterOperation {
   private:
      SLocation m_sEventLocation;
      float m_fRadius;
   public: 
      CLocationFilter(swarmmesh::CSwarmMesh<SEventData>* pc_sm) : 
         swarmmesh::CSwarmMesh<SEventData>::CFilterOperation(pc_sm) {}
      
      ~CLocationFilter() {}
      bool operator()(const swarmmesh::CSwarmMesh<SEventData>::STuple&) override;
      void Serialize(std::vector<uint8_t>&) override;
      size_t Deserialize(const std::vector<uint8_t>&, size_t) override;
      void Init(const std::unordered_map<std::string, std::any>&) override;
      std::unordered_map<std::string, std::any> GetParams() override;
};


using namespace argos;

class CCollectivePerception : public CCI_Controller {

public:

   /* Class constructor. */
   CCollectivePerception();

   /* Class destructor. */
   //virtual ~CCollectivePerception() {}

   /*
    * This function initializes the controller.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    */
   virtual void Destroy() {}

/** 
 * Motion related members
 **/

public:

   /* This function implements diffuse motion */
   void Diffuse();

private:


   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the RAB sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the RAB actuator */
   CCI_RangeAndBearingActuator* m_pcRABA;
   /* Pointer to the positioning sensor */
   CCI_PositioningSensor* m_pcPositioning;

   CCI_CameraSensor* m_pcCamera;

   /*
    * The following variables are used as parameters for the diffusion
    * algorithm.
    */
   /* Maximum tolerance for the angle between the robot heading 
      direction and the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
      the robot and the closest obstacle. */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   CRange<CRadians> m_cGoStraightAngleRange;

/**
 * Data management related members
 **/ 

public:

   /* Structure representing neighbors */
   struct SNeighbor
   {
      UInt16 RId;
      Real Distance;
      CRadians Bearing;
      bool operator==(const SNeighbor& x) {
            return (RId == x.RId);}
   };

   /* Structure for keeping track of query replies */
   struct STimingInfo
   {
      UInt16 Start;
      UInt16 LastUpdate;
      size_t NumReplies;
      bool Done = false;
      STimingInfo() {}
      STimingInfo(UInt16 un_start, UInt16 un_update) :
         Start(un_start), LastUpdate(un_update) {}
   };

   /*TODO: make private and add getters*/

   /* Data structure object */
   CMySwarmMesh m_cMySM; 

   /* Get node ID */
   inline uint16_t GetNodeID() {return m_cMySM.Partition();} 

   /* Map of query id to query */
   std::unordered_map<uint32_t, std::unordered_map<std::string, std::any>> m_mapQueries;

   /* Map of query id to timing info */
   std::unordered_map<uint32_t, STimingInfo> m_mapQueryTimings;

   /* Get Message Count */
   inline UInt16 GetMessageCount() {return m_unMessageCount;}

   /* Reset Message Count */
   inline void SetMessageCount(UInt16 un_message_count) {m_unMessageCount = un_message_count; }

   /* Get all consolidated/voted decisions */
   std::vector<SEventData>& GetVotingDecisions();

   /* Get timing info for all voting decisions */
   std::vector<STimingInfo>& GetTimingInfo();

   /* Reset vector of voting decisions */
   inline void ClearVotingDecisions() { m_vecVotingDecisions.clear(); }

   /* Reset vector of timing info */
   inline void ClearTimingInfo() { m_vecTimingInfo.clear(); }

private:

   /* The robot numeric id */
   UInt16 m_unRobotId;

   /* Number of recorded events */
   UInt16 m_unTupleCount;

   /* Number of messages sent */
   UInt16 m_unMessageCount;

   /* Pointer to random number generator */
   CRandom::CRNG* m_pcRNG;

   std::vector<SNeighbor> m_vecNeighbors;

   /* Internal clock */
   UInt16 m_unClock;
   UInt16 m_unTimeLastRecording;
   UInt16 m_unTimeLastQuery;

   /* Vector of decisions made by the robot after majority voting */
   std::vector<SEventData> m_vecVotingDecisions;

   /* Vector of timing info for all voting events */
   std::vector<STimingInfo> m_vecTimingInfo;

   /* Returns the list of events recorded by the robot
      at the current time step */
   std::queue<SEventData> RecordEvents();

   /* Queries SwarmMesh based on its own tuples */
   void RequestObservations();

   /* Aggregates results of queries on SwarmMesh */
   void AggregateObservations();

   SEventData ConsolidateObservations(const std::vector<STuple>& vec_tuples, const SLocation& s_loc);

   /* Returns world coordinates for a point given its coordinate relative 
   to this robot */
   CVector2 ComputeAbsolutePosition(const CVector2& c_coordTuple);

   void ProcessInMsgs();

   void ProcessOutMsgs();

};

#endif
