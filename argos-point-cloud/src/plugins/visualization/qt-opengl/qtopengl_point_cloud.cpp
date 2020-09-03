/**
 * @file <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_point_cloud.cpp>
 *
 * @author Nathalie Majcherczyk - <nathalie.majcherczyk@gmail.com>
 */

#include "qtopengl_point_cloud.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/simulator/entities/point_cloud_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

namespace argos {

   /****************************************/
   /****************************************/

   const GLfloat MOVABLE_COLOR[]    = { 1.0f, 0.0f, 0.0f, 1.0f };
   const GLfloat NONMOVABLE_COLOR[] = { 0.7f, 0.7f, 0.7f, 1.0f };

   const GLfloat SPECULAR[]         = { 0.0f, 0.0f, 0.0f, 1.0f };
   const GLfloat SHININESS[]        = { 0.0f                   };
   const GLfloat EMISSION[]         = { 0.0f, 0.0f, 0.0f, 1.0f };

   /****************************************/
   /****************************************/

    CQTOpenGLPointCloud::CQTOpenGLPointCloud() :
       m_unVertices(20){

       /* Reserve the needed display lists */
       m_unBaseList = glGenLists(2); // 1?
       m_unBodyList = m_unBaseList;

       /* Make body list */
       glNewList(m_unBodyList, GL_COMPILE);
       MakeBody();
       glEndList();

    }

   /****************************************/
   /****************************************/

   CQTOpenGLPointCloud::~CQTOpenGLPointCloud() {
      glDeleteLists(m_unBaseList, 2); //1 ?
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLPointCloud::Draw(const CPointCloudEntity& c_entity) {
      /* Draw the body */

      // glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
      // glPushMatrix();
      // glScalef(c_entity.GetSize().GetX(), c_entity.GetSize().GetY(), c_entity.GetSize().GetZ());
      // glCallList(m_unBodyList);
      // glPopMatrix();

      // const GLfloat color[] = {c_entity.GetColor().GetRed(), 
      // c_entity.GetColor().GetGreen(), c_entity.GetColor().GetBlue(),
      // c_entity.GetColor().GetAlpha()};

      const SBoundingBox& sBBox = c_entity.GetEmbodiedEntity().GetBoundingBox();
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      glDisable(GL_LIGHTING);
      glLineWidth(3.0f);
      glColor3b(c_entity.GetColor().GetRed(), 
      c_entity.GetColor().GetGreen(), c_entity.GetColor().GetBlue());
      /* This part covers the top and bottom faces (parallel to XY) */
      glBegin(GL_QUADS);
      /* Bottom face */
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MinCorner.GetZ());
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MinCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MinCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MinCorner.GetZ());
      /* Top face */
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MaxCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MaxCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MaxCorner.GetZ());
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MaxCorner.GetZ());
      glEnd();
      /* This part covers the faces (South, East, North, West) */
      glBegin(GL_QUADS);
      /* South face */
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MinCorner.GetZ());
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MaxCorner.GetZ());
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MaxCorner.GetZ());
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MinCorner.GetZ());
      /* East face */
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MinCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MinCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MaxCorner.GetZ());
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MaxCorner.GetZ());
      /* West face */
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MinCorner.GetZ());
      glVertex3f(sBBox.MinCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MaxCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MaxCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MinCorner.GetZ());
      glEnd();

      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      /* North face */
      glBegin(GL_QUADS);
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MinCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MinCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MaxCorner.GetY(), sBBox.MaxCorner.GetZ());
      glVertex3f(sBBox.MaxCorner.GetX(), sBBox.MinCorner.GetY(), sBBox.MaxCorner.GetZ());

      glEnd();
      glEnable(GL_LIGHTING);
      glLineWidth(1.0f);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLPointCloud::MakeBody() {
	     /* Since this shape can be stretched,
	         make sure the normal vectors are unit-long */
	      glEnable(GL_NORMALIZE);

	      /* Set the material */
	      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR);
	      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, SHININESS);
	      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, EMISSION);

	      /* Let's start the actual shape */

	      /* This part covers the top and bottom faces (parallel to XY) */
	      glBegin(GL_QUADS);
	      /* Bottom face */
	      glNormal3f(0.0f, 0.0f, -1.0f);
	      glVertex3f( 0.5f,  0.5f, 0.0f);
	      glVertex3f( 0.5f, -0.5f, 0.0f);
	      glVertex3f(-0.5f, -0.5f, 0.0f);
	      glVertex3f(-0.5f,  0.5f, 0.0f);
	      /* Top face */
	      glNormal3f(0.0f, 0.0f, 1.0f);
	      glVertex3f(-0.5f, -0.5f, 1.0f);
	      glVertex3f( 0.5f, -0.5f, 1.0f);
	      glVertex3f( 0.5f,  0.5f, 1.0f);
	      glVertex3f(-0.5f,  0.5f, 1.0f);
	      glEnd();
	      /* This part covers the faces (South, East, North, West) */
	      glBegin(GL_QUADS);
	      /* South face */
        glNormal3f(0.0f, -1.0f, 0.0f);
        glVertex3f(-0.5f, -0.5f, 1.0f);
        glVertex3f(-0.5f, -0.5f, 0.0f);
        glVertex3f( 0.5f, -0.5f, 0.0f);
        glVertex3f( 0.5f, -0.5f, 1.0f);
        /* East face */
        glNormal3f(1.0f, 0.0f, 0.0f);
        glVertex3f( 0.5f, -0.5f, 1.0f);
        glVertex3f( 0.5f, -0.5f, 0.0f);
        glVertex3f( 0.5f,  0.5f, 0.0f);
        glVertex3f( 0.5f,  0.5f, 1.0f);
        /* North face */
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f( 0.5f,  0.5f, 1.0f);
        glVertex3f( 0.5f,  0.5f, 0.0f);
        glVertex3f(-0.5f,  0.5f, 0.0f);
        glVertex3f(-0.5f,  0.5f, 1.0f);
        /* West face */
        glNormal3f(-1.0f, 0.0f, 0.0f);
        glVertex3f(-0.5f,  0.5f, 1.0f);
        glVertex3f(-0.5f,  0.5f, 0.0f);
        glVertex3f(-0.5f, -0.5f, 0.0f);
        glVertex3f(-0.5f, -0.5f, 1.0f);
	      glEnd();
	      /* The shape definitions is finished */

	      /* We don't need it anymore */
	      glDisable(GL_NORMALIZE);
   }

   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawPointCloudNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CPointCloudEntity& c_entity) {
         static CQTOpenGLPointCloud m_cModel;
         c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawPointCloudSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CPointCloudEntity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawPointCloudNormal, CPointCloudEntity);

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawPointCloudSelected, CPointCloudEntity);

   /****************************************/
   /****************************************/

}