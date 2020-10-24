#include "collective_perception_controller.h"
#include <argos3/plugins/robots/generic/control_interface/ci_camera_sensor_point_cloud_detector_algorithm.h>
#include <argos3/plugins/robots/generic/control_interface/ci_camera_sensor_algorithm.h>

/****************************************/
/****************************************/

std::map<std::string, ECategory> CCategoryMap::StringToCategoryMap =
{
      {"bag", BAG},
      {"bed", BED}, 
      {"bin", BIN},
      {"box", BOX},
      {"cabinet", CABINET},
      {"chair", CHAIR},
      {"collective_label", COLLECTIVE_LABEL},
      {"desk", DESK},
      {"display", DISPLAY},
      {"door", DOOR},
      {"pillow", PILLOW},
      {"shelf", SHELF},
      {"sink", SINK},
      {"sofa", SOFA},
      {"table", TABLE},
      {"toilet", TOILET},
      {"unknown", UNKNOWN}
};

std::map<ECategory, std::string> CCategoryMap::CategoryToStringMap =
{
      {BAG, "bag"},
      {BED, "bed"}, 
      {BIN, "bin", },
      {BOX, "box"},
      {CABINET, "cabinet"},
      {CHAIR, "chair"},
      {COLLECTIVE_LABEL, "collective_label"},
      {DESK, "desk"},
      {DISPLAY, "display"},
      {DOOR, "door"},
      {PILLOW, "pillow"},
      {SHELF, "shelf"},
      {SINK, "sink"},
      {SOFA, "sofa"},
      {TABLE, "table"},
      {TOILET, "toilet"},
      {UNKNOWN, "unknown"}
};

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
  
   SEventData sValue;
   
   sValue.Type = CCategoryMap::CategoryToStringMap[static_cast<ECategory>(swarmmesh::UnpackUInt8(vec_buffer, un_offset))];
   //swarmmesh::UnpackString(vec_buffer, un_offset);

   // LOG << sValue.Type << '\n';

   sValue.Payload = {swarmmesh::UnpackFloat(vec_buffer, un_offset),
                     CCategoryMap::CategoryToStringMap[static_cast<ECategory>(swarmmesh::UnpackUInt8(vec_buffer, un_offset))]};
                     // swarmmesh::UnpackString(vec_buffer, un_offset)};

   // LOG << sValue.Payload.Category << '\n';

   sValue.Location = {swarmmesh::UnpackFloat(vec_buffer, un_offset), 
                     swarmmesh::UnpackFloat(vec_buffer, un_offset),
                     swarmmesh::UnpackFloat(vec_buffer, un_offset)};

   return sValue;
}

/**
 * Serialize the given event data into the byte buffer
 * 
 * @param vec_buffer The byte buffer
 * @param s_event The event to be serialized
 */
void PackEventDataType(std::vector<uint8_t>& vec_buffer, const SEventData& s_event) {

   // LOG << s_event.Type << ' ' << static_cast<uint8_t>(CCategoryMap::StringToCategoryMap[s_event.Type]) << '\n';

   swarmmesh::PackUInt8(vec_buffer, static_cast<uint8_t>(CCategoryMap::StringToCategoryMap[s_event.Type]));
   // swarmmesh::PackString(vec_buffer, s_event.Type);

   swarmmesh::PackFloat(vec_buffer,s_event.Payload.Radius);

   // LOG << static_cast<uint8_t>(CCategoryMap::StringToCategoryMap[s_event.Payload.Category]) << '\n';
   swarmmesh::PackUInt8(vec_buffer, static_cast<uint8_t>(CCategoryMap::StringToCategoryMap[s_event.Payload.Category]));
   // swarmmesh::PackString(vec_buffer, s_event.Payload.Category);

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
   uint16_t unHash;
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
   /* Consolidated observation */
   else if (strCategory == "collective_label") {unHash = 1 + 15 * BUCKET_SIZE;}
   else  unHash = 0;

   /* Unique tuple identifier based on robot id and 
      tuple count */
   ++m_unTupleCount;
   uint32_t unIdentifier = ((uint32_t) m_unRobotId << 16) + m_unTupleCount;
   
   return swarmmesh::SKey(unHash, unIdentifier);
}

void CMySwarmMesh::Init(uint16_t un_robotId, uint16_t un_rabSize) {
   m_cHashEvent.Init(un_robotId);
   CSwarmMesh::Init(un_robotId, m_cHashEvent, un_rabSize); 
}

/****************************************/
/****************************************/

std::unordered_map<std::string, std::any> CTypeFilter::GetParams() {
   std::unordered_map<std::string, std::any> mapFilterParams;
   mapFilterParams["type"] = m_strEventType;
   return mapFilterParams;
}

void CTypeFilter::Init(const std::unordered_map<std::string, std::any>& map_filterParams) {
   m_strEventType = std::any_cast<std::string>(map_filterParams.at("type"));
}

bool CTypeFilter::operator()(const swarmmesh::CSwarmMesh<SEventData>::STuple& s_tuple) {
   return s_tuple.Value.Type == m_strEventType;
}

void CTypeFilter::Serialize(std::vector<uint8_t>& vec_buffer) {
   swarmmesh::PackString(vec_buffer, m_strEventType);
}

size_t CTypeFilter::Deserialize(const std::vector<uint8_t>& vec_buffer, size_t un_offset) {
   m_strEventType = swarmmesh::UnpackString(vec_buffer, un_offset);
   return un_offset;
}

/****************************************/
/****************************************/

std::unordered_map<std::string, std::any> CLocationFilter::GetParams() {
   std::unordered_map<std::string, std::any> mapFilterParams;
   mapFilterParams["radius"] = m_fRadius;
   mapFilterParams["location"] = m_sEventLocation;
   return mapFilterParams;
}

void CLocationFilter::Init(const std::unordered_map<std::string, std::any>& map_filterParams) {
   m_fRadius = std::any_cast<float>(map_filterParams.at("radius"));
   m_sEventLocation = std::any_cast<SLocation>(map_filterParams.at("location"));
}

bool CLocationFilter::operator()(const swarmmesh::CSwarmMesh<SEventData>::STuple& s_tuple) {
   CVector3 cEventCoord = CVector3(m_sEventLocation.X, m_sEventLocation.Y, m_sEventLocation.Z);
   CVector3 cTupleCoord = CVector3(s_tuple.Value.Location.X, s_tuple.Value.Location.Y, s_tuple.Value.Location.Z);

   return Distance(cEventCoord, cTupleCoord) <= m_fRadius;
}

void CLocationFilter::Serialize(std::vector<uint8_t>& vec_buffer) {
   swarmmesh::PackFloat(vec_buffer, m_fRadius);
   swarmmesh::PackFloat(vec_buffer, m_sEventLocation.X);
   swarmmesh::PackFloat(vec_buffer, m_sEventLocation.Y);
   swarmmesh::PackFloat(vec_buffer, m_sEventLocation.Z);
}

size_t CLocationFilter::Deserialize(const std::vector<uint8_t>& vec_buffer, size_t un_offset) {
   m_fRadius = swarmmesh::UnpackFloat(vec_buffer, un_offset);
   m_sEventLocation = {swarmmesh::UnpackFloat(vec_buffer, un_offset), 
                       swarmmesh::UnpackFloat(vec_buffer, un_offset),
                       swarmmesh::UnpackFloat(vec_buffer, un_offset)};
   return un_offset;
}

/****************************************/
/****************************************/

CCollectivePerception::CCollectivePerception() :
   m_unMessageCount(0),
   m_unClock(0),
   m_unTimeLastRecording(0),
   m_unTimeLastQuery(0),
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
   uint16_t unMsgSize = m_pcRABA->GetSize();
   m_cMySM.Init(m_unRobotId, unMsgSize);
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
   ClearVotingDecisions();
   ClearObservations();
   m_vecHashes.clear();

   /* Process incoming messages */
   ProcessInMsgs();

   /* Move */
   Diffuse();

   /* Record events */
   std::queue<SEventData> sEvents = RecordEvents();   

   /* Write events into swarmmesh */
   while(!sEvents.empty())
   {
      m_unMessageCount++;
      /* Retrieve event to write in SwarmMesh */
      SEventData sEvent = sEvents.front();
      sEvents.pop();
      ++m_unTupleCount;
      /* Perform a put operation in SwarmMesh */
      m_vecHashes.push_back(m_cMySM.Put(sEvent));
      m_vecObservations.push_back(sEvent);
      LOG << "Put as new observation \n";
   }

   /* Request observations from SwarmMesh */
   RequestObservations();

   /* Process collective data in SwarmMesh*/
   AggregateObservations();

   m_unNumRoutingTuples += m_cMySM.RoutingTuples().size();

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

   /* Go through readings */
   for (auto reading : tReadings)
   {
      /* Create event */
      SEventData event;
      /* Write observed category as tuple type*/
      event.Type = reading.Category;
      /* Compute sphere radius as corner to center distance */
      float fRadius = Distance(reading.Center, reading.Corners[0]);
      /* Record category and radius as payload */
      event.Payload = SPointCloud(fRadius, reading.Category);
      event.Location = {(float) reading.Center.GetX(), (float) reading.Center.GetY(), 
      (float) reading.Center.GetZ()};
      /* Add to queue of events*/
      sEvents.push(event);
      /* Save current time for recording time-out */
      m_unTimeLastRecording = m_unClock;
   }

   /* Debugging info */
   if(tReadings.size() > 0){
      LOG << m_strId << " sees " << tReadings[0].Category << " at " << tReadings[0].Center << '\n';
   }

   return sEvents;
}

/****************************************/
/****************************************/


void CCollectivePerception::RequestObservations()
{
   /* Vector of stored tuples sorted in descending order 
      of data importance */
   std::vector<STuple>& vecTuples = m_cMySM.StoredTuples();
   m_unNumStoredTuples += vecTuples.size();

   /* Minimum number of observations in a given locations to 
      trigger a query on the shared memory */
   UInt16 unMinLocalObservations = CONSOLIDATION_QUOTA;

   /* No requests if zero threshold */
   if(m_unClock < m_unTimeLastQuery + QUERY_TIMEOUT) return;

   if(vecTuples.size() == 0) return;

   // LOG << "Num tuples" << vecTuples.size() << '\n';
   // LOG << "Threshold" << unMinLocalObservations << '\n';

   auto it = vecTuples.begin();
   /* Exclude consolidated observations */
   while (it->Key.Hash > 1 + 14 * BUCKET_SIZE && it != vecTuples.end()) ++it;
   /* No requests if no raw observations */
   if (it == vecTuples.end()) return;

   std::unordered_map<std::string, std::any> mapFilterParams;

   while (it != vecTuples.end())
   {
      STuple sTuple = *it;
      float fRadius = sTuple.Value.Payload.Radius;
      SLocation sLocation(sTuple.Value.Location);
      /* Look for at least unMinLocalObservations
         in the region of the tuple */
      auto found = it;
      uint16_t count = 0;
      while(found != vecTuples.end())
      {
         /* Check if distance within noise radius */
         found = std::find_if(found, vecTuples.end(),
         [sTuple] (const STuple& s_tuple) { 
            CVector3 cTupleLocation(sTuple.Value.Location.X, 
            sTuple.Value.Location.Y, sTuple.Value.Location.Z);
            CVector3 cOtherLocation(s_tuple.Value.Location.X,
            s_tuple.Value.Location.Y, s_tuple.Value.Location.Z);
            return Distance(cTupleLocation, cOtherLocation) <= NOISE_THRESHOLD;}
         );
         if(found == vecTuples.end()) return;
         ++count;
         ++found;
      } 
      /* If we have found at least as many observations as min*/
      if(count >= unMinLocalObservations) 
      {
         float fRadius = NOISE_THRESHOLD;
         /* Create spatial request for tuple of most important type */
         mapFilterParams["radius"] = fRadius;  // noise under 0.3 
         mapFilterParams["location"] = sLocation;
         /* Save query */
         uint32_t unQueryId = m_cMySM.Filter((uint8_t) 1, mapFilterParams);
         m_mapQueries[unQueryId] = mapFilterParams;
         m_mapQueryTimings[unQueryId] = STimingInfo(m_unClock, m_unClock);

         LOG << m_strId << " made request for (" <<  sLocation.X << ", " 
         << sLocation.Y << ", " << sLocation.Z << ") \n";
         m_unTimeLastQuery = m_unClock;
         return;
      }
      ++it;
   }

}

void CCollectivePerception::AggregateObservations()
{
   /* Get vector of query ids on SwarmMesh */
   std::deque<uint32_t> vecQueries = m_cMySM.QueryIds();

   /* Get results of queries */
   std::unordered_map<uint32_t, std::vector<STuple>> mapResults = m_cMySM.QueryResults();
   /* Go through all SwarmMesh active queries */
   for (auto it = vecQueries.begin();
        it != vecQueries.end(); ++it)
   {
      /* Ignore timed-out queries */
      if(m_unClock - m_mapQueryTimings[*it].Start > QUERY_TIMEOUT) continue;

      /* If query results available */
      if(mapResults.count(*it) != 0)
      {

         /* If received new results to query */
         if (m_mapQueryTimings[*it].NumReplies != mapResults[*it].size()) {
            m_mapQueryTimings[*it].NumReplies = mapResults[*it].size();
            m_mapQueryTimings[*it].LastUpdate = m_unClock;
            SLocation sLoc = std::any_cast<SLocation>(m_mapQueries[*it].at("location"));
            LOG << "Got result for query for (" << sLoc.X << ", " <<
            sLoc.Y << ", " << sLoc.Z << ") \n"; 
         }
         /* No more expected results? */
         else if(m_unClock - m_mapQueryTimings[*it].LastUpdate > UPDATE_TIMEOUT
            && m_mapQueryTimings[*it].Done == false)
         {
            /* Delete the observations used */
            m_cMySM.Erase((uint8_t)1, m_mapQueries[*it]);

            /* Get location from emitted query */
            SLocation sLoc = std::any_cast<SLocation>(m_mapQueries[*it].at("location"));

            LOG << m_unRobotId << " deleting observations for " << sLoc.X << ", " <<
            sLoc.Y << ", " <<  sLoc.Z << '\n';
            

            // /* Write consolidated prediction */;
            SEventData sEvent = ConsolidateObservations(mapResults[*it], sLoc);
            m_vecVotingDecisions.push_back(sEvent);
            m_vecTimingInfo.push_back(m_mapQueryTimings[*it]);

            LOG << m_unRobotId << " writing label " << sEvent.Payload.Category
            << " for (" << sEvent.Location.X << ", " <<
            sEvent.Location.Y << ", " <<  sEvent.Location.Z << ") with "
            << (int) sEvent.Payload.Radius << " observations" << '\n';
            m_unMessageCount++;
            m_vecHashes.push_back(m_cMySM.Put(sEvent));

            /* Avoid consolidating again */
            m_mapQueryTimings[*it].Done = true;

         }
         
      }

   }
}

/****************************************/
/****************************************/

SEventData CCollectivePerception::ConsolidateObservations(
   const std::vector<STuple>& vec_tuples, const SLocation& s_loc)
{
   /* Majority voting */
   std::vector<STuple> vecSorted(vec_tuples);
   
   std::sort(vecSorted.begin(), vecSorted.end(),[](STuple const& lhs, STuple const& rhs){
      std::string strLhs = lhs.Value.Type;
      std::string strRhs = rhs.Value.Type;
      int res = strLhs.compare(strRhs);
      return res < 0;
   });
   /* Find most common element in vector */
   int nCount = 1;
   int nTopCount = 1;
   size_t unTopIndex = 0;

   for (size_t i = 1; i < vecSorted.size() ; ++i)
   {
      if(vecSorted[i].Value.Type == vecSorted[i-1].Value.Type) 
      {
         ++nCount;
         if(nCount > nTopCount) 
         {
            nTopCount = nCount;
            unTopIndex = i;
         }
      }
      else nCount = 1;
   }
   std::string strConsolidated = vecSorted[unTopIndex].Value.Payload.Category;
   SEventData sEvent;
   /* */
   sEvent.Type = "collective_label";
   /* */
   sEvent.Payload = SPointCloud(vecSorted.size(), strConsolidated);
   /* Location */
   sEvent.Location = s_loc;
   LOG << sEvent.Type << " " << unTopIndex << std::endl;
   return sEvent;
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

   m_unBytesSent += cBuffer.Size();

   /* Pad buffer with zeros to match fixed packet size */
   while(cBuffer.Size() < m_pcRABA->GetSize()) cBuffer << static_cast<UInt8>(0);

   m_pcRABA->SetData(cBuffer);

}

/****************************************/
/****************************************/

std::vector<SEventData>& CCollectivePerception::GetVotingDecisions() {
   return m_vecVotingDecisions;
}

/****************************************/
/****************************************/

std::vector<CCollectivePerception::STimingInfo>& CCollectivePerception::GetTimingInfo() {
   return m_vecTimingInfo;
}

/****************************************/
/****************************************/

/* This statement notifies ARGoS of the existence of the controller. */
REGISTER_CONTROLLER(CCollectivePerception, "collective_perception_controller")
