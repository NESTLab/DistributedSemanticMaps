/**
 * @file <argos3/plugins/robots/generic/simulator/camera_sensor_algorithms/camera_sensor_point_cloud_detector_algorithm.h>
 *
 * @author Nathalie Majcherczyk - <nathalie.majcherczyk@gmail.com>
 */

#ifndef CAMERA_SENSOR_POINT_CLOUD_DETECTOR_ALGORITHM_H
#define CAMERA_SENSOR_POINT_CLOUD_DETECTOR_ALGORITHM_H

namespace argos {
	class CCameraSensorPointCloudDetectorAlgorithm;
}

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>
#include <argos3/core/utility/math/ray3.h>
#include <argos3/core/utility/math/matrix/transformationmatrix3.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/plugins/simulator/entities/point_cloud_entity.h>
#include <argos3/plugins/robots/generic/simulator/camera_sensor_algorithm.h>
#include "../../control_interface/ci_camera_sensor_algorithms/ci_camera_sensor_point_cloud_detector_algorithm.h"

#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <random>

namespace argos {
   
   /**
    * This class provides the most general interface to a camera.
    * The camera sensor enables the user to extract information from the images
    * acquired by the simulated or by the physical camera.
    */
   class CCameraSensorPointCloudDetectorAlgorithm : public CCameraSensorSimulatedAlgorithm,
                                             public CCI_CameraSensorPointCloudDetectorAlgorithm {
      
   public:
      class CUpdateOperation : public CBaseUpdateOperation,
                               public CPositionalIndex<CPointCloudEntity>::COperation {

      public:
         /* constructor */
         CUpdateOperation(const CSquareMatrix<3>& c_projection_matrix,
                          const std::array<CPlane, 6>& arr_frustum_planes,
                          const CTransformationMatrix3& c_camera_to_world_transform,
                          const CVector3& c_camera_location,
                          CCameraSensorPointCloudDetectorAlgorithm& c_algorithm) :
            CBaseUpdateOperation(c_projection_matrix,
                                 arr_frustum_planes,
                                 c_camera_to_world_transform,
                                 c_camera_location),
            m_cAlgorithm(c_algorithm) {
            m_cOcclusionCheckRay.SetStart(c_camera_location);
         }
         /* destructor */
         virtual ~CUpdateOperation() {}
         /* operation */
         virtual bool operator()(CPointCloudEntity& c_pointCloud) {
            // if(GetAngleWithCamera(c_tag) > c_tag.GetObservableAngle()) {
            //    return true;
            // }

            /* Copy all bounding box corners */
            size_t i = 0;
            for (auto it = c_pointCloud.GetFrontCorners().begin(); 
            it != c_pointCloud.GetFrontCorners().end(); it++ )
            {
               m_arrBoundingBoxCorners[i] = *it;
               i++;
            }
            for (auto it = c_pointCloud.GetBackCorners().begin(); 
            it != c_pointCloud.GetBackCorners().end(); it++ )
            {
               m_arrBoundingBoxCorners[i] = *it;
               i++;
            }

            /* Check if front face is in frustum */
            for(const CVector3& c_corner : c_pointCloud.GetFrontCorners()) {
               if(IsPointInsideFrustum(c_corner) == false) {
                  /* corner is not inside the frustum */
                  return true;
               }
            }

            /* Check occlusions of front face */
            for(const CVector3& c_corner : c_pointCloud.GetFrontCorners()) {
               m_cOcclusionCheckRay.SetEnd(c_corner);
               if(GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem, m_cOcclusionCheckRay)) {
                  /* corner is occluded */
                  m_cAlgorithm.AddCheckedRay(true, m_cOcclusionCheckRay);
                  return true;
               }
               else {
                  m_cAlgorithm.AddCheckedRay(false, m_cOcclusionCheckRay);
               }
            }

            CVector3 cCenter = c_pointCloud.GetEmbodiedEntity().GetOriginAnchor().Position;

            double dAccuracy = m_cAlgorithm.m_probPosteriorDistribution(m_cAlgorithm.m_randEngine) 
                                 / m_cAlgorithm.m_dNumSamples;
            double dSampleAccuracy = m_cAlgorithm.m_pcRNG->Uniform(CRange<double>(0.0, 1.0));

	         std::string strCategory = c_pointCloud.GetCategory();
            if (dSampleAccuracy > dAccuracy) {
               strCategory = m_cAlgorithm.m_mapCategory.at(
                  m_cAlgorithm.m_mapIncorrectProbDistribution[strCategory](m_cAlgorithm.m_randEngine));
            }

            CQuaternion cOrientation = c_pointCloud.GetEmbodiedEntity().GetOriginAnchor().Orientation;

            m_cAlgorithm.AddReading(strCategory, cCenter, m_arrBoundingBoxCorners, cOrientation);
            return true;
         }

      private:
         std::array<CVector3, 8> m_arrBoundingBoxCorners;

         CRay3 m_cOcclusionCheckRay;
         SEmbodiedEntityIntersectionItem m_sIntersectionItem;
         CCameraSensorPointCloudDetectorAlgorithm& m_cAlgorithm;
      };

   public:

      CCameraSensorPointCloudDetectorAlgorithm();

      virtual ~CCameraSensorPointCloudDetectorAlgorithm() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update(const CSquareMatrix<3>& c_projection_matrix,
                          const std::array<CPlane, 6>& arr_frustum_planes,
                          const CTransformationMatrix3& c_camera_to_world_transform,
                          const CVector3& c_camera_location,
                          const CVector3& c_bounding_box_position,
                          const CVector3& c_bounding_box_half_extents);

      void AddCheckedRay(bool b_intersected, const CRay3& c_ray) {
         if(m_bShowRays) {
            m_vecCheckedRays.emplace_back(b_intersected, c_ray);
         }
      }

      void AddReading(const std::string& str_category,
                      const CVector3& c_center,
                      const std::array<CVector3, 8>& arr_corners,
                      const CQuaternion c_orientation) {
         m_vecReadings.emplace_back(str_category, c_center, arr_corners, c_orientation);
      }

      /**
       * Returns true if the rays must be shown in the GUI.
       * @return true if the rays must be shown in the GUI.
       */
      inline bool IsShowRays() {
         return m_bShowRays;
      }

      /**
       * Sets whether or not the rays must be shown in the GUI.
       * @param b_show_rays true if the rays must be shown, false otherwise
       */
      inline void SetShowRays(bool b_show_rays) {
         m_bShowRays = b_show_rays;
      }

   private:
      bool                           m_bShowRays;
      CPositionalIndex<CPointCloudEntity>*  m_pcPointCloudIndex;
      std::discrete_distribution<int> m_probPosteriorDistribution;
      std::default_random_engine m_randEngine;
      double m_dNumSamples;
      CRandom::CRNG* m_pcRNG;

      /**
       * Map of probability distributions over all incorrect predictions for each class
       * 
       */
      std::unordered_map<std::string, std::discrete_distribution<int>> m_mapIncorrectProbDistribution;
      
      const std::unordered_map<int, std::string> InitMapCategory() {
         return std::unordered_map<int, std::string>({
            {0, "bag"},
            {1, "bin"},
            {2, "box"},
            {3, "cabinet"},
            {4, "chair"},
            {5, "desk"},
            {6, "display"},
            {7, "door"}, 
            {8, "shelf"},
            {9, "table"},
            {10, "bed"},
            {11, "pillow"},
            {12, "sink"},
            {13, "sofa"},
            {14, "toilet"}
         });
      }
      const std::unordered_map<int, std::string> m_mapCategory = InitMapCategory();
   };
}         

#endif
