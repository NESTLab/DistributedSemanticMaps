/**
 * @file <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_point_cloud.h>
 *
 * @author Nathalie Majcherczyk - <nathalie.majcherczyk@gmail.com>
 */

#ifndef QTOPENGL_POINT_CLOUD_H
#define QTOPENGL_POINT_CLOUD_H

namespace argos {
   class CQTOpenGLPointCloud;
   class CPointCloudEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLPointCloud {

   public:

      CQTOpenGLPointCloud();

      virtual ~CQTOpenGLPointCloud();

      virtual void Draw(const CPointCloudEntity& c_entity);

   private:

      void MakeBody();

   private:

      GLuint m_unBaseList;
      GLuint m_unBodyList;
      GLuint m_unVertices;

   };

}

#endif