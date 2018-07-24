#ifndef OpenGLSupport_hpp
#define OpenGLSupport_hpp

#include <GL/freeglut.h>

// windows specific functions
void InitGL();
bool SetPixelFormatDescriptor();
void SetDefaultGLFont();
void ScreenShotSave(char* ImagePath);
#endif
