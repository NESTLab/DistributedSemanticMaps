/**
 * @file <argos3/plugins/simulator/entities/point_cloud_entity.h>
 *
 * @author Nathalie Majcherczyk <nmajcherczyk@wpi.edu>
 */

#ifndef POINT_CLOUD_ENTITY_H
#define POINT_CLOUD_ENTITY_H

namespace argos {
   class CPointCloudEntity;
   class CPointCloudMedium;
}

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>

#include <argos3/core/utility/datatypes/set.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

#include <argos3/core/simulator/space/positional_indices/space_hash.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>

#include <string>
#include <map>
#include <array>

namespace argos {

   class CPointCloudEntity : public CComposableEntity {

   public:
   
      /* NYU-D v2 labels */
      enum ECategory {
         BAG = 0,
         BATHTUB,
         BED,
         BLINDS,
         BOX,
         BOOKS,
         BOOKSHELF,
         CABINET,
         CEILING,
         CHAIR,
         CLOTHES,
         COUNTER,
         CURTAIN,
         DESK,
         DOOR,
         DRESSER,
         FLOOR,
         FLOOR_MAT,
         FRIDGE,
         FURNITURE,
         LAMP,
         MIRROR,
         NIGHT_STAND,
         PAPER,
         PERSON,
         PICTURE,
         PILLOW,
         PROP,
         SHELVES,
         SHOWER,
         SHOWER_CURTAIN,
         SINK,
         SOFA,
         STRUCTURE,
         TABLE,
         TELEVISION,
         TOILET,
         TOWEL,
         WALL,
         WHITEBOARD,
         WINDOW,
         UNKNOWN
      };

      /* String switch paradigm */
      struct categoryMap : public std::map<std::string, ECategory>
      {
         categoryMap()
         {
            this->operator[]("bag") =  BAG;
            this->operator[]("bathtub") = BATHTUB;
            this->operator[]("bed") = BED;
            this->operator[]("blinds") =  BLINDS;
            this->operator[]("box") =  BOX;
            this->operator[]("books") = BOOKS;
            this->operator[]("bookshelf") = BOOKSHELF;
            this->operator[]("cabinet") = CABINET;
            this->operator[]("ceiling") = CEILING;
            this->operator[]("chair") = CHAIR;
            this->operator[]("clothes") = CLOTHES;
            this->operator[]("counter") = COUNTER;
            this->operator[]("curtain") = CURTAIN;
            this->operator[]("desk") =  DESK;
            this->operator[]("door") = DOOR;
            this->operator[]("dresser") = DRESSER;            
            this->operator[]("floor") =  FLOOR;
            this->operator[]("floor mat") = FLOOR_MAT;
            this->operator[]("fridge") = FRIDGE;
            this->operator[]("furniture") =  FURNITURE;
            this->operator[]("lamp") = LAMP;
            this->operator[]("mirror") = MIRROR;
            this->operator[]("night stand") =  NIGHT_STAND;
            this->operator[]("paper") = PAPER;
            this->operator[]("person") = PERSON;
            this->operator[]("picture") =  PICTURE;
            this->operator[]("pillow") = PILLOW;
            this->operator[]("prop") = PROP;            
            this->operator[]("shelves") = SHELVES;
            this->operator[]("shower") = SHOWER;            
            this->operator[]("shower curtain") =  SHOWER_CURTAIN;
            this->operator[]("sink") = SINK;
            this->operator[]("sofa") = SOFA;
            this->operator[]("structure") =  STRUCTURE;
            this->operator[]("table") = TABLE;
            this->operator[]("television") = TELEVISION;
            this->operator[]("toilet") =  TOILET;
            this->operator[]("towel") = TOWEL;
            this->operator[]("wall") = WALL;
            this->operator[]("whiteboard") =  WHITEBOARD;
            this->operator[]("window") = WINDOW;
            this->operator[]("unknown") = UNKNOWN; 
            this->operator[]("") = UNKNOWN;           
         };

         ~categoryMap(){}

      };

   public:

      ENABLE_VTABLE();

      typedef std::vector<CPointCloudEntity*> TList;
      typedef CSet<CPointCloudEntity*> TSet;

      CPointCloudEntity();

      CPointCloudEntity(const std::string& str_id,
                 const CVector3& c_position,
                 const CQuaternion& c_orientation,
                 const CVector3& c_size,
                 ECategory e_category = UNKNOWN,
                 CColor c_color = CColor::BLACK);

      virtual ~CPointCloudEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline const CEmbodiedEntity& GetEmbodiedEntity() const {
         return *m_pcEmbodiedEntity;
      }

      inline const CVector3& GetSize() const {
         return m_cSize;
      }

      inline void SetSize(const CVector3& c_size) {
         m_cSize = c_size;
      }

      inline const std::array<CVector3, 4>& GetFrontCorners() const {
         return m_arrFrontFaceCorners;
      }

      inline const std::array<CVector3, 4>& GetBackCorners() const {
         return m_arrBackFaceCorners;
      }

      inline ECategory GetCategory() {
         return m_eCategory;
      }

      inline void SetCategory(ECategory e_category) {
         m_eCategory = e_category;
      }

      inline CColor GetColor() const {
         return m_cColor;
      }

      inline void SetColor(CColor c_color) {
         m_cColor = c_color;
      }

      inline size_t GetNumCategories() const {
         return m_unNumCategories;
      }

      void SetEnabled(bool b_enabled);

      virtual std::string GetTypeDescription() const {
         return "point_cloud";
      }

      /**
       * Returns <tt>true</tt> if this point cloud is associated to a medium.
       * @return <tt>true</tt> if this point cloud is associated to a medium.
       * @see CPointCloudMedium
       */
      inline bool HasMedium() const {
         return m_pcPointCloudMedium != nullptr;
      }

      /**
       * Returns the medium associated to this point cloud.
       * @return The medium associated to this point cloud.
       * @see CPointCloudMedium
       */
      CPointCloudMedium& GetMedium() const;

      /**
       * Sets the medium associated to this entity.
       * @param c_medium The medium to associate to this entity.
       * @see CPointCloudMedium
       */
      void SetMedium(CPointCloudMedium& c_medium);

   private:

      CEmbodiedEntity*    m_pcEmbodiedEntity;
      CVector3            m_cSize;
      ECategory           m_eCategory;
      CColor              m_cColor;
      CPointCloudMedium*  m_pcPointCloudMedium;
      std::array<CVector3, 4> m_arrFrontFaceCorners;
      std::array<CVector3, 4> m_arrBackFaceCorners;
      size_t m_unNumCategories;

      void CalculateFaceCorners();

   };

   /****************************************/
   /****************************************/

   class CPointCloudEntitySpaceHashUpdater : public CSpaceHashUpdater<CPointCloudEntity> {

   public:

      virtual void operator()(CAbstractSpaceHash<CPointCloudEntity>& c_space_hash,
                              CPointCloudEntity& c_element);

   private:

      SInt32 m_nI, m_nJ, m_nK;

   };

   /****************************************/
   /****************************************/

   class CPointCloudEntityGridUpdater : public CGrid<CPointCloudEntity>::COperation {

   public:

      CPointCloudEntityGridUpdater(CGrid<CPointCloudEntity>& c_grid);
      virtual bool operator()(CPointCloudEntity& c_entity);

   private:

      CGrid<CPointCloudEntity>& m_cGrid;
      SInt32 m_nI, m_nJ, m_nK;

   };

   /****************************************/
   /****************************************/

}

#endif
