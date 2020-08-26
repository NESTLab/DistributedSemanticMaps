/**
 * @file <argos3/plugins/simulator/entities/point_cloud_entity.cpp>
 *
 * @author Nathalie Majcherczyk - <nmajcherczyk@wpi.edu>
 */

#include "point_cloud_entity.h"
#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include "../media/point_cloud_medium.h"

namespace argos {

   /****************************************/
   /****************************************/

   CPointCloudEntity::CPointCloudEntity():
      CComposableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcPointCloudMedium(NULL) {
          Disable();
      }

   /****************************************/
   /****************************************/

   CPointCloudEntity::CPointCloudEntity(const std::string& str_id,
                          const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          const CVector3& c_size,
                          ECategory e_category,
                          CColor c_color) :
      CComposableEntity(NULL, str_id),
      m_pcEmbodiedEntity(
         new CEmbodiedEntity(this,
                             "body_0",
                             c_position,
                             c_orientation,
                             false)),
      m_cSize(c_size),
      m_eCategory(e_category),
      m_cColor(c_color),
      m_pcPointCloudMedium(NULL) {
      AddComponent(*m_pcEmbodiedEntity);
      Disable();
   }

   /****************************************/
   /****************************************/

   void CPointCloudEntity::CalculateFaceCorners() {
      
      /* Coordinates of front face corners in box aligned reference frame */
      std::array<CVector3, 4> arrCornerOffsets;
      arrCornerOffsets[0] = 0.5f * m_cSize; // top right corner
      arrCornerOffsets[1] = CVector3(arrCornerOffsets[0].GetX(), -arrCornerOffsets[0].GetY(),
                                     arrCornerOffsets[0].GetZ()); // top left corner
      arrCornerOffsets[2] = CVector3(arrCornerOffsets[0].GetX(), arrCornerOffsets[0].GetY(),
                                    -arrCornerOffsets[0].GetZ()); // bottom right corner
      arrCornerOffsets[3] = CVector3(arrCornerOffsets[0].GetX(), -arrCornerOffsets[0].GetY(),
                                    -arrCornerOffsets[0].GetZ()); // bottom left corner
      /* Coordinates of back face corners in box aligned reference frame */
      std::array<CVector3, 4> arrBackCornerOffsets;
      arrBackCornerOffsets[0] = CVector3(-arrCornerOffsets[0].GetX(), arrCornerOffsets[0].GetY(),
                                          arrCornerOffsets[0].GetZ()); //top right corner
      arrBackCornerOffsets[1] = CVector3(-arrCornerOffsets[0].GetX(), -arrCornerOffsets[0].GetY(),
                                          arrCornerOffsets[0].GetZ()); // left corner
      arrBackCornerOffsets[2] = CVector3(-arrCornerOffsets[0].GetX(), arrCornerOffsets[0].GetY(),
                                         -arrCornerOffsets[0].GetZ()); // bottom right corner
      arrBackCornerOffsets[3] = CVector3(-arrCornerOffsets[0].GetX(), -arrCornerOffsets[0].GetY(),
                                         -arrCornerOffsets[0].GetZ()); // bottom left corner

      /* Convert front face corner coordinates to world reference frame */
      std::transform(std::begin(arrCornerOffsets),
                     std::end(arrCornerOffsets),
                     std::begin(m_arrFrontFaceCorners),
                     [=] (const CVector3& c_corner_offset) {
         CVector3 cCorner(c_corner_offset);
         cCorner.Rotate(m_pcEmbodiedEntity->GetOriginAnchor().Orientation);
         return (cCorner + m_pcEmbodiedEntity->GetOriginAnchor().Position);
      });

      /* Convert front face corner coordinates to world reference frame */
      std::transform(std::begin(arrBackCornerOffsets),
                     std::end(arrBackCornerOffsets),
                     std::begin(m_arrBackFaceCorners),
                     [=] (const CVector3& c_corner_offset) {
         CVector3 cCorner(c_corner_offset);
         cCorner.Rotate(m_pcEmbodiedEntity->GetOriginAnchor().Orientation);
         return (cCorner + m_pcEmbodiedEntity->GetOriginAnchor().Position);
      });

   }

   /****************************************/
   /****************************************/

   void CPointCloudEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Parse XML to get the size */
         GetNodeAttribute(t_tree, "size", m_cSize);
         /* Parse XML to get the category attribute */  
         std::string strCategory;       
         GetNodeAttribute(t_tree, "category", strCategory);
         m_eCategory = categoryMap() [strCategory];
         /* Parse XML to get the mass */
         GetNodeAttribute(t_tree, "color", m_cColor);
         /* Create embodied entity using parsed data */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));
         m_pcEmbodiedEntity->SetMovable(false);
         UpdateComponents();
         /* Set world coordinates of front face */
         CalculateFaceCorners();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize point cloud entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPointCloudEntity::Reset() {
      m_eCategory = ECategory::UNKNOWN;
      m_cColor = CColor::BLACK;
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CPointCloudEntity::SetEnabled(bool b_enabled) {
      /* Perform generic enable behavior */
      CEntity::SetEnabled(b_enabled);
      if(b_enabled) {
         /* Enable entity in medium */
         if(m_pcPointCloudMedium && GetIndex() >= 0)
            m_pcPointCloudMedium->AddEntity(*this);
      }
      else {
         /* Disable entity in medium */
         if(m_pcPointCloudMedium)
            m_pcPointCloudMedium->RemoveEntity(*this);
      }
   }

   /****************************************/
   /****************************************/

   CPointCloudMedium& CPointCloudEntity::GetMedium() const {
      if(m_pcPointCloudMedium == nullptr) {
         THROW_ARGOSEXCEPTION("Point cloud entity \"" << GetContext() + GetId() <<
                              "\" has no medium associated.");
      }
      return *m_pcPointCloudMedium;
   }

   /****************************************/
   /****************************************/

   void CPointCloudEntity::SetMedium(CPointCloudMedium& c_medium) {
      if(m_pcPointCloudMedium != nullptr && m_pcPointCloudMedium != &c_medium)
         m_pcPointCloudMedium->RemoveEntity(*this);
      m_pcPointCloudMedium = &c_medium;
   }


   /****************************************/
   /****************************************/

   void CPointCloudEntitySpaceHashUpdater::operator()(CAbstractSpaceHash<CPointCloudEntity>& c_space_hash,
                                               CPointCloudEntity& c_element) {
      /* Calculate the position of the tag in the space hash */
      c_space_hash.SpaceToHashTable(m_nI, m_nJ, m_nK, c_element.GetEmbodiedEntity().GetOriginAnchor().Position);
      /* Update the corresponding cell */
      c_space_hash.UpdateCell(m_nI, m_nJ, m_nK, c_element);
   }

   /****************************************/
   /****************************************/

   CPointCloudEntityGridUpdater::CPointCloudEntityGridUpdater(CGrid<CPointCloudEntity>& c_grid) :
      m_cGrid(c_grid) {}

   /****************************************/
   /****************************************/

   bool CPointCloudEntityGridUpdater::operator()(CPointCloudEntity& c_entity) {
      try {
         /* Calculate the position of the tag in the space hash */
         m_cGrid.PositionToCell(m_nI, m_nJ, m_nK, c_entity.GetEmbodiedEntity().GetOriginAnchor().Position);
         /* Update the corresponding cell */
         m_cGrid.UpdateCell(m_nI, m_nJ, m_nK, c_entity);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While updating the point cloud grid for point cloud \"" <<
                                     c_entity.GetContext() + c_entity.GetId() << "\"", ex);
      }
      /* Continue with the other entities */
      return true;
   }

   /****************************************/
   /****************************************/

   /****************************************/
   /****************************************/

   class CSpaceOperationAddCPointCloudEntity : public CSpaceOperationAddEntity {
   public:
      void ApplyTo(CSpace& c_space, CPointCloudEntity& c_entity) {
         /* Add entity to space - this ensures that the tag entity
          * gets an id before being added to the tag medium */
         c_space.AddEntity(c_entity);
         /* Enable the tag entity, if it's enabled - this ensures that
          * the entity gets added to the tag if it's enabled */
         c_entity.SetEnabled(c_entity.IsEnabled());
      }
   };

   class CSpaceOperationRemoveCPointCloudEntity : public CSpaceOperationRemoveEntity {
   public:
      void ApplyTo(CSpace& c_space, CPointCloudEntity& c_entity) {
         /* Disable the entity - this ensures that the entity is
          * removed from the tag medium */
         c_entity.Disable();
         /* Remove the tag entity from space */
         c_space.RemoveEntity(c_entity);
      }
   };

   REGISTER_SPACE_OPERATION(CSpaceOperationAddEntity,
                            CSpaceOperationAddCPointCloudEntity,
                            CPointCloudEntity);
   REGISTER_SPACE_OPERATION(CSpaceOperationRemoveEntity,
                            CSpaceOperationRemoveCPointCloudEntity,
                            CPointCloudEntity);

   // /****************************************/
   // /****************************************/

   REGISTER_ENTITY(CPointCloudEntity,
                   "point_cloud",
                   "Nathalie Majcherczyk [nmajcherczyk@wpi.edu]",
                   "1.0",
                   "A point cloud with a bounding box.",
                   "It can be used to import objects of different types (40 NYU-D categories).\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "To declare an object (i.e., a wall) you need the following:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <point_cloud id=\"box1\" size=\"0.75,0.1,0.5\" category=\"chair\"\n"
                   "                 color=\"120,130,140\">\n"
                   "      <body position=\"0.4,2.3,0\" orientation=\"45,0,0\" />\n"
                   "    </point_cloud>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'size' attribute specifies the size of the bouding box along the three axes\n" 
                   "in the X,Y,Z order. When you add a box, imagine it initially unrotated and\n"
                   "centered in the origin. The size, then, corresponds to the extent along the X,\n"
                   "Y and Z axes.\n"
                   "The 'category' attribute specifies the type of object.\n"
                   "The 'body/position' attribute specifies the position of the base of the box in\n"
                   "the arena. The three values are in the X,Y,Z order.\n"
                   "The 'body/orientation' attribute specifies the orientation of the 3D box. All\n"
                   "rotations are performed with respect to the center of mass. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n",
                   "Usable"
      );

   // /****************************************/
   // /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CPointCloudEntity);

   // /****************************************/
   // /****************************************/

}