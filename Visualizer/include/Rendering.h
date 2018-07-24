#ifndef RENDERING_H
#define RENDERING_H

#include <string>
#include "OpenGLSupport.h"   // for OpenGL support
#include <Eigen/Core>

const double RAD_PER_DEG = 0.017453292519943295769236907684886;	///< PI/180.0

using namespace std;
using namespace Eigen;

struct GlColorType
{
   // Reverse for intel storage order
   unsigned char blue;
   unsigned char green;
   unsigned char red;
   unsigned char alpha;
};

struct GlRgbColorType
{
   unsigned char red;
   unsigned char green;
   unsigned char blue;
   unsigned char alpha;
};

void SetColor(GlColorType color, unsigned char red, unsigned char green, unsigned char blue);
void DrawSphere(GLdouble radius, GLint slices, GLint stacks, GLenum style,
                GLenum orientation = GLU_OUTSIDE, GLenum normals = GL_SMOOTH,
                GLenum textureCoords = GL_TRUE);
void DrawLine(GlColorType *color, const Vector3d &start, const Vector3d &end);
void DrawLine(float red, float green, float blue, const Vector3d &start, const Vector3d &end);
void DrawLine(double x1, double y1, double x2, double y2);
void DrawCube(float x, float y, float z);
void DrawSpacecraft(float radius, GlColorType *color1, GlColorType *color2, bool drawSphere = true);
void DrawEquatorialPlanes();
void DrawCircle(GLUquadricObj *qobj, double radius);
void DrawCircle(double x1, double y1, double radius, bool fill = true);
void DrawSquare(double x1, double y1, double radius, bool fill = true);
void DrawStringAt(const string &str, const Vector3d &point);
void DrawStringAt(const string &str, GLfloat x, GLfloat y, GLfloat z,
                  GLfloat k);

#endif
