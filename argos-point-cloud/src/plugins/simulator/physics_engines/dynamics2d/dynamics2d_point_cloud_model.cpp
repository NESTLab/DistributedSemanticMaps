/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_point_cloud_model.cpp>
 *
 * @author Nathalie Majcherczyk - <nathalie.majcherczyk@gmail.com>
 */

#include "dynamics2d_point_cloud_model.h"
// #include "dynamics2d_gripping.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics2DPointCloudModel::CDynamics2DPointCloudModel(CDynamics2DEngine& c_engine,
                                            CPointCloudEntity& c_entity) :
      CDynamics2DStretchableObjectModel(c_engine, c_entity) {
      DEBUG("Id %s", c_entity.GetId().c_str());
      /* Get the size of the entity */
      CVector3 cHalfSize = c_entity.GetSize() * 0.5f;
      /* Create a polygonal object in the physics space */
      /* Start defining the vertices
         NOTE: points must be defined in a clockwise winding
      */
      cpVect tVertices[] = {
         cpv(-cHalfSize.GetX(), -cHalfSize.GetY()),
         cpv(-cHalfSize.GetX(),  cHalfSize.GetY()),
         cpv( cHalfSize.GetX(),  cHalfSize.GetY()),
         cpv( cHalfSize.GetX(), -cHalfSize.GetY())
      };
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      /*
       * Create body and shapes
       */
      cpBody* ptBody;
      /* The box is not movable */
      /* Create a static body */
      ptBody = cpBodyNewStatic();
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      cpBodySetAngle(ptBody, cZAngle.GetValue());
      /* Create the shape */
      cpShape* ptShape =
      cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                     cpPolyShapeNew(ptBody,
                                       4,
                                       tVertices,
                                       cpvzero));
      ptShape->e = 0.0; // No elasticity
      ptShape->u = 0.1; // Little contact friction to help sliding away
      /* This shape is normal (not grippable, not gripper) */
      ptShape->collision_type = CDynamics2DEngine::SHAPE_NORMAL;
      /* Set the body so that the default methods work as expected */
      SetBody(ptBody, c_entity.GetSize().GetZ());
   }
   
   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CPointCloudEntity, CDynamics2DPointCloudModel);

   /****************************************/
   /****************************************/

}