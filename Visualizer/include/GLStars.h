#ifndef GLSTARTS_h
#define GLSTARTS_h

#include <string>
#include <GL/freeglut.h>
using namespace std;

const int   MAXSTARS   = 42101;     // Max Number Stars in arrays
const int   MAXLINES   = 1600;      // Max Number of Constellation line vertices
const int   MAXCON     = 90;
const int   MAXBORDERS = 64000;     // Max Number of Constellation line vertices
const int   MAXBORDERGROUP = 90;    // Max Number of border groups
const float STARSTEP   = 0.5f;      // Scale into this size groups
const int   GROUPCOUNT = 18;        // Number of groups

//------------------------------------------------------------------------------
class GLStars
{
public:
    static GLStars* Instance();
    void DrawStarsVA(GLfloat ColorAlpha, int starCount, bool drawConstellations);    // Vertex Array Method, with alpha
    void SetDesiredStarCount(int count)
    {
        if (m_DesiredStarCount <= MAXSTARS)
            m_DesiredStarCount = count;
        else
            m_DesiredStarCount = MAXSTARS;
    }
    int  GetDesiredStarCount()
    {
        return m_DesiredStarCount;
    }
  
private:
   GLStars();
   ~GLStars();
   GLStars(const GLStars&);
   GLStars& operator= (const GLStars&);

private:
    void InitStars();                        // Initialize the Stars, including loading from file
    void ReadStars();
    void ReadConstellations();
    void ReadBorders();
    void SetVector (GLfloat v[4], double ra, double dec);
    void Correct1875 (GLfloat v[4]);

    GLfloat     m_Stars[MAXSTARS][4];
    GLfloat     m_CLines[MAXLINES][4];          // Constellation points
    GLfloat     m_Borders[MAXBORDERS][4];       // Constellation borders
    string      m_ConstellationNames[MAXCON];   // The names of the constellations
    int         m_ConstellationIndex[MAXCON][2]; // The starting and ending indices for each constellation
    int         m_GroupIndex[GROUPCOUNT];      // Indexes into the m_StarsVA array
    int         m_GroupCount[GROUPCOUNT];      // # of stars in this group
    //int       m_BorderGroup[90];
    int         m_BorderGroup[MAXBORDERGROUP];
    double      m_PointSize[GROUPCOUNT];       // Group GLpointsize
    int         m_LastGroupUsed;               // Not all groups may be used
    int         m_MaxDrawStars;                // Not Array Size
    int         m_DesiredStarCount;            // DesiredStars, if we want it to be adjustable
    int         m_NumLines;                    // Number of constellation lines loaded
    int         m_NumConstellations;           // Number of constellations loaded
    int         m_BorderGroupCount;
    static      GLStars* theInstance;        // The singleton instance of the Stars
};
//------------------------------------------------------------------------------
#endif //GLSTARTS_h
