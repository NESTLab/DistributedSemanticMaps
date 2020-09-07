#ifndef POINT_CLOUD_MEDIUM_H
#define POINT_CLOUD_MEDIUM_H

namespace argos {
   class CPointCloudMedium;
   class CPointCloudEntity;
}

#include <argos3/core/simulator/medium/medium.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>
#include "../entities/point_cloud_entity.h"

namespace argos {

   class CPointCloudMedium : public CMedium {

   public:

      /**
       * Class constructor.
       */
      CPointCloudMedium():
         m_pcPointCloudEntityIndex(nullptr),
         m_pcPointCloudEntityGridUpdateOperation(nullptr) {}

      /**
       * Class destructor.
       */
      virtual ~CPointCloudMedium(){}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void PostSpaceInit();
      virtual void Reset();
      virtual void Destroy();
      virtual void Update();

     /**
      * Adds the specified entity to the list of managed entities.
      * @param c_entity The entity to add.
      */
      void AddEntity(CPointCloudEntity& c_entity);

     /**
      * Removes the specified entity from the list of managed entities.
      * @param c_entity The entity to remove.
      */
      void RemoveEntity(CPointCloudEntity& c_entity);

      /**
       * Returns the positional index.
       * @return The positional index.
       */
      CPositionalIndex<CPointCloudEntity>& GetIndex() {
         return *m_pcPointCloudEntityIndex;
      }

   private:

      /** A positional index for the point cloud entities */
      CPositionalIndex<CPointCloudEntity>* m_pcPointCloudEntityIndex;

      /** The update operation for the grid positional index */
      CPointCloudEntityGridUpdater* m_pcPointCloudEntityGridUpdateOperation;

   };

}

#endif
