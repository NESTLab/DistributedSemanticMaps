/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_point_cloud_model.h>
 *
 * @author Nathalie Majcherczyk - <nathalie.majcherczyk@gmail.com>
 */

#ifndef DYNAMICS2D_POINT_CLOUD_MODEL_H
#define DYNAMICS2D_POINT_CLOUD_MODEL_H

namespace argos {
   class CDynamics2DStretchableObjectModel;
   class CDynamics2DPointCloudModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_stretchable_object_model.h>
#include <argos3/plugins/simulator/entities/point_cloud_entity.h>

namespace argos {

   class CDynamics2DPointCloudModel : public CDynamics2DStretchableObjectModel {

   public:

      CDynamics2DPointCloudModel(CDynamics2DEngine& c_engine,
                          CPointCloudEntity& c_entity);
      virtual ~CDynamics2DPointCloudModel() {}
   };

}

#endif