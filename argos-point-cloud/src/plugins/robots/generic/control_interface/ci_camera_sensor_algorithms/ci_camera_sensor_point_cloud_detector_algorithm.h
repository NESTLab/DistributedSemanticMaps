/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_camera_sensor_algorithms/ci_camera_sensor_point_cloud_detector_algorithm.h>
 *
 * @author Nathalie Majcherczyk - <nmajcherczyk@wpi.edu>
 */

#ifndef CI_CAMERAS_SENSOR_POINT_CLOUD_DETECTOR_ALGORITHM_H
#define CI_CAMERAS_SENSOR_POINT_CLOUD_DETECTOR_ALGORITHM_H

namespace argos {
	class CCI_CameraSensorPointCloudDetectorAlgorithm;
}

#include <argos3/plugins/robots/generic/control_interface/ci_camera_sensor_algorithm.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <array>
#include <string>
#include <argos3/plugins/simulator/entities/point_cloud_entity.h>

#ifdef ARGOS_WITH_LUA
extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}
#endif


namespace argos {
   
   class CCI_CameraSensorPointCloudDetectorAlgorithm : virtual public CCI_CameraSensorAlgorithm {
      
   public:

      struct SReading {
         /* Category of the point cloud */
         std::string Category;
         /* Coordinates of the center of the bounding box 
          * in 3D */
         CVector3 Center;
         /* Coordinates of the corners of the bounding box 
          * in world coordinates */
         std::array<CVector3, 8> Corners;
         /**
          * Constructor
          * @param str_category Point cloud category
          * @param c_center Center of the bounding box
          * @param c_corners Corners of the bounding box
          */
         SReading(const std::string& str_category,
                  const CVector3& c_center,
                  const std::array<CVector3, 8>& c_corners) :
            Category(str_category),
            Center(c_center),
            Corners(c_corners) {}
      };

   public:
      
      /**
       * Constructor
       */
      CCI_CameraSensorPointCloudDetectorAlgorithm() {}
      
      /**
       * Destructor
       */
      virtual ~CCI_CameraSensorPointCloudDetectorAlgorithm() {}
      
      const std::vector<SReading>& GetReadings() const {
         return m_vecReadings;
      }
      
#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
      
      virtual void ReadingsToLuaState(lua_State* pt_lua_state);

      virtual const std::string& GetId() {
         static std::string strId("point_cloud_detector");
         return strId;
      }
#endif

   protected:

      std::vector<SReading> m_vecReadings;
      
   };
   
}

#endif
