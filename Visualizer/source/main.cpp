#include <iostream>
#include <GL/freeglut.h>
#include "GLStars.h"

using namespace std;

void myDisplay(void)
{
    glClear(GL_COLOR_BUFFER_BIT);
    GLStars *pStars = GLStars::Instance();
    pStars->SetDesiredStarCount(5000);
    pStars->DrawStarsVA(1.0f, 5000, false);
    //glColor3f(1.0,0,0);
    //glRectf(-0.5f, -0.5f, 0.5f, 0.5f);
    glFlush();
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);
    glutCreateWindow("第一个OpenGL程序");
    glutDisplayFunc(&myDisplay);
    glutMainLoop();
    return 0;
}


