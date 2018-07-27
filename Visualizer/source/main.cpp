#include <iostream>
#include <Eigen/Core>
#include <GL/freeglut.h>
#include "GLStars.h"
#include "OpenGLSupport.h"
#include "Camera.h"

using namespace std;
using namespace Eigen;
const double DEFAULT_DISTANCE = 30000.0;
int WIN_WIDTH = 800;
int WIN_HIGHT = 600;
// camera
Camera mCamera;
void myDisplay(void)
{
    GLStars *pStars = GLStars::Instance();
    mCamera.Reset();
    mCamera.Relocate(DEFAULT_DISTANCE, 0.0, 0.0, 0.0, 0.0, 0.0);

    InitGL();
    SetDrawingMode(WIN_WIDTH,WIN_HIGHT);
    // Set background color to black
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glDisable(GL_LIGHTING);
    // drawing the stars at infinity requires them to have their own projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLfloat aspect = WIN_WIDTH / WIN_HIGHT;
    glMatrixMode(GL_MODELVIEW);
    gluPerspective(mCamera.fovDeg, aspect, 0.1f, 50000000.0f);
    // The stars also need to be drawn in their own world view to be drawn at infinity
    Vector3d starPosition = mCamera.position;
    Vector3d starCenter = mCamera.view_center - starPosition;
    Vector3d starUp = mCamera.up;

    // if star position is not zero vector then normalize (bug 2367 fix)
    //if (!starPosition.IsZeroVector())
    //  starPosition.norm();
    starCenter += starPosition;

    gluLookAt(starPosition[0], starPosition[1], starPosition[2],
             starCenter[0], starCenter[1], starCenter[2],
             starUp[0], starUp[1], starUp[2]);


    pStars->SetDesiredStarCount(20000);
    pStars->DrawStarsVA(1.0f, 20000, false);

    glFlush();
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(WIN_WIDTH, WIN_HIGHT);
    glutCreateWindow("第一个OpenGL程序");
    glutDisplayFunc(&myDisplay);
    glutMainLoop();
    return 0;
}


