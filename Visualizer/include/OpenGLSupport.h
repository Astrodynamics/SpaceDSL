#ifndef OpenGLSupport_hpp
#define OpenGLSupport_hpp

#include <GL/freeglut.h>

// windows specific functions
void InitGL();
void SetDrawingMode(int width, int height );
bool SetPixelFormatDescriptor();
void SetDefaultGLFont();
void ScreenShotSave(char* ImagePath);
#endif
