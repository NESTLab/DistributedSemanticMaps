/**
 * @file <argos3/plugins/robots/generic/simulator/camera_sensor_algorithms/camera_sensor_point_cloud_detector_algorithm.cpp>
 *
 * @author Nathalie Majcherczyk - <nmajcherczyk@wpi.edu>
 */

#include "camera_sensor_point_cloud_detector_algorithm.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/matrix/squarematrix.h>
#include <argos3/core/utility/math/matrix/transformationmatrix3.h>
#include "../../simulator/media/point_cloud_medium.h"
#include "../../simulator/entities/point_cloud_entity.h"

namespace argos {

   /****************************************/
   /****************************************/   

   CCameraSensorPointCloudDetectorAlgorithm::CCameraSensorPointCloudDetectorAlgorithm() :
      m_bShowRays(false),
      m_pcPointCloudIndex(nullptr) {}

   /****************************************/
   /****************************************/   
   
   void CCameraSensorPointCloudDetectorAlgorithm::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_CameraSensorAlgorithm::Init(t_tree);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Get tag medium from id specified in the XML */
         std::string strMedium;
         GetNodeAttribute(t_tree, "medium", strMedium);
         
         std::string strFileName;
         GetNodeAttribute(t_tree, "file", strFileName);
         m_pcPointCloudIndex = &(CSimulator::GetInstance().GetMedium<CPointCloudMedium>(strMedium).GetIndex());
      
         std::ifstream ifDistributionFile(strFileName);
         std::vector<double> vecProbabilities;
         if (ifDistributionFile.is_open()) {
            std::string strData;
            while (getline(ifDistributionFile, strData)) {
               vecProbabilities.push_back(std::stod(strData));
            }
            ifDistributionFile.close();
         } else {
            THROW_ARGOSEXCEPTION("Probability distribution file not found")
         }
         m_probPosteriorDistribution = std::discrete_distribution<int>(vecProbabilities.begin(), vecProbabilities.end());
         m_dNumSamples = static_cast<double>(vecProbabilities.size());
         m_pcRNG = CRandom::CreateRNG("argos");
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the tag detector algorithm", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CCameraSensorPointCloudDetectorAlgorithm::Update(const CSquareMatrix<3>& c_projection_matrix,
                                                  const std::array<CPlane, 6>& arr_frustum_planes,
                                                  const CTransformationMatrix3& c_camera_to_world_transform,
                                                  const CVector3& c_camera_location,
                                                  const CVector3& c_bounding_box_position,
                                                  const CVector3& c_bounding_box_half_extents) {
      /* Define a class for the update operation */
      CUpdateOperation cUpdateOperation(c_projection_matrix, arr_frustum_planes,
                                        c_camera_to_world_transform, c_camera_location,
                                        *this);
      /* Clear out readings from last update */
      m_vecReadings.clear();
      /* Clear out checked rays from last update */
      m_vecCheckedRays.clear();
      /* Run the operation */
      m_pcPointCloudIndex->ForEntitiesInBoxRange(c_bounding_box_position,
                                          c_bounding_box_half_extents,
                                          cUpdateOperation);
   }

   /****************************************/
   /****************************************/

   REGISTER_CAMERA_SENSOR_ALGORITHM(CCameraSensorPointCloudDetectorAlgorithm,
                                    "point_cloud_detector",
                                    "Nathalie Majcherczyk [nathalie.majcherczyk@gmail.com]",
                                    "1.0",
                                    "This algorithm detects nearby point clouds seen by the camera and\n"
                                    "returns the coordinates of their corners to the sensor",
                                    "This algorithm detects nearby tags seen by the camera and\n"
                                    "returns the coordinates of their corners to the sensor",
                                    "Under development");
}
