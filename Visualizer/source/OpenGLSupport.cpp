#include "OpenGLSupport.h"   // for OpenGL support
#include "FallbackFont.h"


//------------------------------------------------------------------------------
//  void SetDefaultGLFont()
//------------------------------------------------------------------------------
/**
* Sets default GL font.
*/
//------------------------------------------------------------------------------
void SetDefaultGLFont()
{

   //RRC: Added fallback: cheap fixed-width font baked in (BET, 2016-10-19)
   unsigned int charidx;
   for(charidx = 0; charidx < 256; charidx++)
   {
      //Archaic GL, but equivalent to the above
      glNewList(charidx + 1000, GL_COMPILE);
      glBitmap(8, 8, 0, 0, 8, 0, &(FallbackFont[charidx*8]));
      glEndList();
   }

   glListBase(1000);
}

//------------------------------------------------------------------------------
// bool InitGL()
//------------------------------------------------------------------------------
/**
* Initializes GL and IL.
*/
//------------------------------------------------------------------------------
void InitGL()
{

   // remove back faces
   glEnable(GL_CULL_FACE);
   //glDisable(GL_CULL_FACE);
	
   // enable depth testing, so that objects further away from the
   // viewer aren't drawn over closer objects
   glEnable(GL_DEPTH_TEST);
   glDepthMask(GL_TRUE);

   glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
   glDepthFunc(GL_LEQUAL);
   glClearDepth(1.0f);
   //glDepthFunc(GL_LESS);
   //glDepthRange(0.0, 100.0); //loj: just tried - made no difference

   // speedups
   glEnable(GL_DITHER);

   // set polygons to be smoothly shaded (i.e. interpolate lighting equations
   // between points on polygons).
   glShadeModel(GL_SMOOTH);
   glFrontFace(GL_CCW);

	glEnable(GL_LINE_SMOOTH);
   glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
   glHint(GL_POLYGON_SMOOTH_HINT, GL_FASTEST);

   // font
   SetDefaultGLFont();


}


//------------------------------------------------------------------------------
// void ScreenShotSave(char* ImagePath)
//------------------------------------------------------------------------------
void ScreenShotSave(char* ImagePath)
{
   /*
   GLint vp[4];
   // Get the viewport dimensions
   glGetIntegerv(GL_VIEWPORT, vp);
   // Store the viewport width and height, width = vp[2], height = vp[3]
   wxImage image(vp[2], vp[3]);
   // glReadPixels can align the first pixel in each row at 1-, 2-, 4- and 
   // 8-byte boundaries. We have allocated the exact size needed for the image 
   // so we have to use 1-byte alignment (otherwise glReadPixels would write 
   // out of bounds)
   glPixelStorei(GL_PACK_ALIGNMENT, 1);
   glReadBuffer(GL_FRONT);
   glReadPixels(0, 0, vp[2], vp[3], GL_RGB, GL_UNSIGNED_BYTE, image.GetData());

   // glReadPixels reads the given rectangle from bottom-left to top-right, so we must
   // vertically flip the image to the proper orientation (option false is vertical flip)
   image = image.Mirror(false); 
   image.SaveFile(ImagePath, wxBITMAP_TYPE_PNG);
   */
}
