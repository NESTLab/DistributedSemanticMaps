#include "collective_perception_controller.h"
#include <argos3/plugins/robots/generic/control_interface/ci_camera_sensor_point_cloud_detector_algorithm.h>
#include <argos3/plugins/robots/generic/control_interface/ci_camera_sensor_algorithm.h>


/****************************************/
/****************************************/

/**
 * Unpack SEventData object from the byte buffer
 * 
 * @param vec_buffer The byte buffer
 * @param un_offset Offset into the buffer
 * @return SEventData The unpacked object
 */
SEventData UnpackEventDataType(const std::vector<uint8_t>& vec_buffer, size_t& un_offset) {
  
   // LOG << vec_buffer.size() << " " << un_offset << std::endl;
   SEventData sValue;
   
   sValue.Type = swarmmesh::UnpackString(vec_buffer, un_offset);
   // LOG << sValue.Type << std::endl;

   sValue.Payload = {swarmmesh::UnpackFloat(vec_buffer, un_offset),
                     swarmmesh::UnpackString(vec_buffer, un_offset)};

   // LOG << sValue.Payload.Radius << std::endl;

   sValue.Location = {swarmmesh::UnpackFloat(vec_buffer, un_offset), 
                     swarmmesh::UnpackFloat(vec_buffer, un_offset),
                     swarmmesh::UnpackFloat(vec_buffer, un_offset)};

   // LOG << sValue.Location.X << " " << sValue.Location.Y << std::endl;
   return sValue;
}

/**
 * Serialize the given event data into the byte buffer
 * 
 * @param vec_buffer The byte buffer
 * @param s_event The event to be serialized
 */
void PackEventDataType(std::vector<uint8_t>& vec_buffer, const SEventData& s_event) {

   swarmmesh::PackString(vec_buffer, s_event.Type);

   swarmmesh::PackFloat(vec_buffer,s_event.Payload.Radius);
   swarmmesh::PackString(vec_buffer, s_event.Payload.Category);

   swarmmesh::PackFloat(vec_buffer, s_event.Location.X);
   swarmmesh::PackFloat(vec_buffer, s_event.Location.Y);
   swarmmesh::PackFloat(vec_buffer, s_event.Location.Z);
}

/****************************************/
/****************************************/

/* Per class accuracy with BGA-DGCNN
30.1	box
48.2	bag
72.7	bed
72.9	toilet
74.1	table
77.3	desk
78.1	pillow
79.2	sink
80.4	display
80.5	shelf
81.9	bin
84.4	cabinet
91	sofa
92.4	door
92.6	chair
*/

swarmmesh::SKey CHashEventDataType::operator()(SEventData& s_value) {
   
   std::string strCategory = s_value.Type;

   /* Data hashing based on point cloud type */
   uint32_t unHash;
   if(strCategory == "chair") {unHash = 1;}
   else if(strCategory == "door") {unHash = 1 + 1 * BUCKET_SIZE;}
   else if(strCategory == "sofa") {unHash = 1 + 2 * BUCKET_SIZE;}
   else if(strCategory == "cabinet") {unHash = 1 + 3 * BUCKET_SIZE;}
   else if(strCategory  == "bin") {unHash = 1 + 4 * BUCKET_SIZE;}
   else if(strCategory  == "shelf") {unHash = 1 + 5 * BUCKET_SIZE;}
   else if(strCategory  == "display") {unHash = 1 + 6 * BUCKET_SIZE;}
   else if(strCategory  == "sink") {unHash = 1 + 7 * BUCKET_SIZE;}
   else if(strCategory  == "pillow") {unHash = 1 + 8 * BUCKET_SIZE;}
   else if(strCategory  == "desk") {unHash = 1 + 9 * BUCKET_SIZE;}
   else if(strCategory  ==  "table") {unHash = 1 + 10 * BUCKET_SIZE;}
   else if(strCategory  == "toilet") {unHash = 1 + 11 * BUCKET_SIZE;}
   else if(strCategory  ==  "bed") {unHash = 1 + 12 * BUCKET_SIZE;}
   else if(strCategory  ==  "bag") {unHash = 1 + 13 * BUCKET_SIZE;}
   else if(strCategory  ==  "box") {unHash = 1 + 14 * BUCKET_SIZE;}
   else  unHash = 0;

   /* Unique tuple identifier based on robot id and 
      tuple count */
   ++m_unTupleCount;
   uint32_t unIdentifier = ((uint32_t) m_unRobotId << 16) + m_unTupleCount;
   
   return swarmmesh::SKey(unHash, unIdentifier);
}

void CMySwarmMesh::Init(uint16_t un_robotId) {
   m_cHashEvent.Init(un_robotId);
   CSwarmMesh::Init(un_robotId, m_cHashEvent); 
}

/****************************************/
/****************************************/

CCollectivePerception::CCollectivePerception() :
   m_unClock(0),
   m_unTimeLastRecording(0),
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRNG(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CCollectivePerception::Init(TConfigurationNode& t_node) 
{
   /* Get sensor/actuator handles */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   m_pcRABA   = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
   m_pcRABS   = GetSensor  <CCI_RangeAndBearingSensor  >("range_and_bearing");
   m_pcPositioning = GetSensor  <CCI_PositioningSensor  >("positioning");

   m_pcCamera = GetSensor <CCI_CameraSensor>("cameras");


   m_pcRNG = CRandom::CreateRNG("argos");

   /* Parse the configuration file */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   /* Initialize SwarmMesh variables */
   const std::string& strRobotId = GetId();
   m_unRobotId = FromString<UInt16>(strRobotId.substr(1));
   m_unTupleCount = 0; 
   m_cMySM.Init(m_unRobotId);
   ProcessOutMsgs();
}

/****************************************/
/****************************************/

void CCollectivePerception::Diffuse() 
{
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = 
   m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }
}

/****************************************/
/****************************************/

void CCollectivePerception::ControlStep() 
{

   /* Process incoming messages */
   ProcessInMsgs();

   /* Move */
   Diffuse();

   /* Record events */
   std::queue<SEventData> sEvents = RecordEvents();   

   /* Write events into swarmmesh */
   while(!sEvents.empty())
   {
      LOG << "Event" << std::endl;
      /* Retrieve event to write in SwarmMesh */
      SEventData sEvent = sEvents.front();
      sEvents.pop();
      ++m_unTupleCount;
      /* Perform a put operation in SwarmMesh */
      m_cMySM.Put(sEvent);
   }

   /* Tell SwarmMesh to queue messages for routing data */
   m_cMySM.Route();

   /* Process outgoing messages */
   ProcessOutMsgs();

   /* Update clock */
   ++m_unClock;

}

/****************************************/
/****************************************/

std::queue<SEventData> CCollectivePerception::RecordEvents() 
{
   std::queue<SEventData> sEvents;
   
   /* Time out on recording events */
   if (m_unClock < m_unTimeLastRecording + RECORDING_TIMEOUT) 
      return sEvents;

   /* Get camera readings */
   const CCI_CameraSensor::SInterface::TVector& tInterfaces = m_pcCamera->GetInterfaces();
   CCI_CameraSensorPointCloudDetectorAlgorithm* pAlgorithm = 
   &dynamic_cast<CCI_CameraSensorPointCloudDetectorAlgorithm&>(*tInterfaces[0].Algorithms[0]);
   const std::vector<CCI_CameraSensorPointCloudDetectorAlgorithm::SReading>& tReadings = pAlgorithm->GetReadings();

   for (auto reading : tReadings)
   {
      SEventData event;
      event.Type = reading.Category;
      float fRadius = sqrt(2) / 2 * Distance(reading.Center, reading.Corners[0]);
      event.Payload = SPointCloud(fRadius, reading.Category);
      event.Location = {reading.Center.GetX(), reading.Center.GetY(), reading.Center.GetZ()};
      sEvents.push(event);
      m_unTimeLastRecording = m_unClock;
   }

   
   if(tReadings.size() > 0){
      LOG << "Robot " << m_unRobotId << std::endl;
      LOG << tReadings[0].Category << " at " << tReadings[0].Center << std::endl;
   }

   return sEvents;
}

/****************************************/
/****************************************/

void CCollectivePerception::ProcessInMsgs()
{

   m_cMySM.ResetNeighbors();

   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

   for(size_t i = 0; i < tPackets.size(); ++i) {
      /* Copy packet into temporary buffer */
      CByteArray cData = tPackets[i].Data;
      /* Get robot id and update neighbor information */
      UInt16 unRobotId = cData.PopFront<uint16_t>();
      /* Record neighbors in argos */
      SNeighbor sNeighbor;
      sNeighbor.RId = unRobotId;
      sNeighbor.Distance = tPackets[i].Range;
      sNeighbor.Bearing = tPackets[i].HorizontalBearing;
      m_vecNeighbors.push_back(sNeighbor);
      /* Record neighbor to SwarmMesh */
      m_cMySM.AddNeighbor(unRobotId,
                          sNeighbor.Distance,
                          sNeighbor.Bearing.GetValue());
      /* Convert CByteArray to STL vector */
      std::vector<std::uint8_t> vecBuffer;
      while(cData.Size()) vecBuffer.push_back((uint8_t) cData.PopFront<uint8_t>());
      /* Process swarmmesh messages */
      size_t offset = 0;
      m_cMySM.Deserialize(vecBuffer, offset);
   }

}

/****************************************/
/****************************************/

void CCollectivePerception::ProcessOutMsgs()
{

   std::vector<uint8_t> vecBuffer;

   /* Pre-pend robot id to any message*/
   swarmmesh::PackUInt16(vecBuffer, m_unRobotId);

   /* Fill buffer with swarmmesh messages */
   m_cMySM.Serialize(vecBuffer);

   /* Convert stl vector to CByteArray */
   CByteArray cBuffer;
   for (auto elem : vecBuffer) cBuffer << elem;

   /* Pad buffer with zeros to match fixed packet size */
   while(cBuffer.Size() < m_pcRABA->GetSize()) cBuffer << static_cast<UInt8>(0);

   m_pcRABA->SetData(cBuffer);

}

/****************************************/
/****************************************/

/* This statement notifies ARGoS of the existence of the controller. */
REGISTER_CONTROLLER(CCollectivePerception, "collective_perception_controller")
