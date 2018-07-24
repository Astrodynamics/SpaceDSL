#include "GLStars.h"
#include "Rendering.h"

#include <fstream>

//------------------------------------------------------------------------------
const GLfloat SOURCELIGHT99[] = {0.99f, 0.99f, 0.99f, 1.0f};
GLStars* GLStars::theInstance = NULL;
//------------------------------------------------------------------------------
// GLStars()
//------------------------------------------------------------------------------
/**
 * Initialize certain values
 */
//------------------------------------------------------------------------------
GLStars::GLStars()
    : m_DesiredStarCount(42000)   // Fixed uninitialzed value error
{
    m_LastGroupUsed = 0;
    m_MaxDrawStars = 0;
    m_DesiredStarCount = 0;
    m_NumLines = 0;
    m_NumConstellations = 0;
    m_BorderGroupCount = 0;
    InitStars();
}
//------------------------------------------------------------------------------
// ~GLStars()
//------------------------------------------------------------------------------
/**
 * Destroy the instance of the class on the way out
 */
//------------------------------------------------------------------------------
GLStars::~GLStars()
{
    if (theInstance)
        delete theInstance;
    theInstance = NULL;
}
//------------------------------------------------------------------------------
// GLStars* Instance()
//------------------------------------------------------------------------------
/**
 * Return the created instance of the Stars
 */
//------------------------------------------------------------------------------
GLStars* GLStars::Instance()
{
    if (theInstance == NULL)
        theInstance = new GLStars;
    return theInstance;
}
//------------------------------------------------------------------------------
// void ReadStars()
//------------------------------------------------------------------------------
/**
 * Read the star information from the file
 */
//------------------------------------------------------------------------------
void GLStars::ReadStars()
{
    int index = 0;
    GLfloat VisMagStep = -3.0f;
    const GLfloat VSScale = 0.30f;
    const GLfloat StarDimmest = 0.02f;
    const GLfloat BaseScale = 9.0f;

    string starFileName = "astrodata/Graphics/stars/StarCatalog.dat";

    // Try opening the file
    ifstream StarsFile;
    StarsFile.open(starFileName);
    if (!StarsFile.is_open())
    {
        return;
    }

    // Parse through the whole file and repeat this process, storing the star in the appropriate group
    int line = 0;
    double ra, dec, mag;
    while (!StarsFile.eof())
    {
        char buffer[30];
        StarsFile.getline(buffer, 30);
        ++line;
        string bufferstr = buffer;
        string raStr = bufferstr.substr(0,8);
        string decStr = bufferstr.substr(8,8);
        string magStr = bufferstr.substr(16,8);
        ra = stod(raStr);
        dec = stod(decStr);
        mag = stod(magStr);
        if (line == 0)
            mag = -2;
        SetVector (m_Stars[line],ra,dec);

        if (mag > VisMagStep)
        {
            m_GroupCount[index] = line - m_GroupIndex[index];
            m_PointSize[index] = StarDimmest + ((BaseScale - VisMagStep) * VSScale);
            index++;
            if (index >= GROUPCOUNT)
                break;
            m_GroupIndex[index] = line;
            VisMagStep += STARSTEP;
        }
    }

    // Finalize the values and close the file
    m_GroupCount[index] = line - m_GroupIndex[index] - 1;
    m_PointSize[index] = StarDimmest + ((BaseScale - VisMagStep) * VSScale);
    m_LastGroupUsed = index;
    StarsFile.close();

    // Store how many star positions we loaded from the file
    m_MaxDrawStars = line - 1;
}
//------------------------------------------------------------------------------
// void ReadConstellations()
//------------------------------------------------------------------------------
/**
 * Read in the star information for the constellations.
 */
//------------------------------------------------------------------------------
void GLStars::ReadConstellations()
   {
   /*
   FileManager *fm = FileManager::Instance();
   wxString constFileName = (fm->FindPath("", "CONSTELLATION_FILE", true, false, true)).c_str();
   
   // Attempt to open the constellation file
   wxTextFile ConstellationFile;
   if (!ConstellationFile.Open(constFileName))
      {
      MessageInterface::LogMessage
          ("Could not open Constellation File at " + constFileName + "\n");
      return;
      }
   
   NumLines = 0;
   NumConstellations = 0;
   
   // Go through the file and store all of the star pairs that make up the constellation lines
   int i;
   wxString buffer;
   double ra1, ra2, dec1, dec2;
   for (i = 0, buffer = ConstellationFile.GetFirstLine(); i < MAXLINES && !ConstellationFile.Eof();
        buffer = ConstellationFile.GetNextLine())
      {
      // Skip empty lines
      if(buffer.IsEmpty())
         continue;
      // Skip the comment lines
      if (buffer[0] == '#')
         continue;
      if (buffer[0] == 'N')
         {
         ConstellationIndex[NumConstellations][1] = i-1;
         NumConstellations++;
         ConstellationNames[NumConstellations] = buffer.AfterFirst(' ');
         ConstellationIndex[NumConstellations][0] = i;
         continue;
         }
      
      // Retrieve the pair of Right Ascensions and Declinations from the line
      std::string line (buffer);
      TextParser tp;
      StringArray split = tp.SeparateSpaces (line);
      GmatStringUtil::ToReal (split[0],dec1);
      GmatStringUtil::ToReal (split[1],ra1);
      GmatStringUtil::ToReal (split[2],dec2);
      GmatStringUtil::ToReal (split[3],ra2);
      // Convert the Right Ascensions from hours to degrees
      ra1 *= 15;
      ra2 *= 15;
      // Shorten each end of line by 0.5 degConvert the Right Ascensions from hours to degrees
      double ddec = dec2-dec1;
      double dra = ra2-ra1;
      if (dra > 180) dra -= 360;
      if (dra < -180) dra += 360;
      double dist = sqrt (ddec*ddec+dra*dra);
      double f = 0.5/dist;
      double fdec = f*ddec;
      double fra = f*dra;
      ra1 = ra1+fra;
      ra2 = ra2-fra;
      dec1 = dec1+fdec;
      dec2 = dec2-fdec;
      // Calculate the star positions and store them in the array
      SetVector (CLines[i],ra1,dec1);
      SetVector (CLines[i+1],ra2,dec2);
      i+=2;
      }
   // Store how many constellation line vertices we loaded in
   NumLines = i - 1;
   ConstellationIndex[NumConstellations][1] = i - 1;

   ConstellationFile.Close();
   */
   }
//------------------------------------------------------------------------------
// void ReadBorders()
//------------------------------------------------------------------------------
/**
 * Read in the border information for the constellations.
 */
//------------------------------------------------------------------------------
void GLStars::ReadBorders()
{
    /*
   FileManager *fm = FileManager::Instance();
   wxString borderFileName = (fm->FindPath("", "BORDER_FILE", true, false, true)).c_str();
   
   // Attempt to open the constellation file
   wxTextFile BorderFile;
   if (!BorderFile.Open(borderFileName))
      {
      MessageInterface::LogMessage
          ("Could not open Constellation Border File at " + borderFileName + "\n");
      return;
      }
      
   int count = 0;  
   int ix = 0; 
   double oldra = 0;
   wxString buffer;
   double ra, dec;
   for (buffer = BorderFile.GetFirstLine(); ix < MAXBORDERS && !BorderFile.Eof();
        buffer = BorderFile.GetNextLine())
   {
      // Skip empty lines
      if(buffer.IsEmpty())
         continue;
      
      std::string line (buffer);
      TextParser tp;
      StringArray split = tp.SeparateSpaces (line);
      GmatStringUtil::ToReal (split[0],ra);
      GmatStringUtil::ToReal (split[1],dec);
      ra *= 15;
      bool start = split.size() == 3;
      if (start)
      {
         count = 0;
         if (ix!=0)
         {
            if (BorderGroupCount < MAXBORDERGROUP)
            {
               BorderGroup[BorderGroupCount] = ix;
               ++BorderGroupCount;
            }
         }
      }
      ++count;
      if (count==1)
      {
         SetVector (Borders[ix],ra,dec);
         Correct1875(Borders[ix]);
         ++ix;
      }
      else if (ra == oldra)
      {
         SetVector (Borders[ix],ra,dec);
         Correct1875(Borders[ix]);
         ++ix;
      }
      else
      {
         double dra = ra - oldra;
         if (dra > 180) dra -=360;
         if (dra < -180) dra +=360;
         double rastep = dra > 0 ? 1 : -1;
         int steps = floor(std::abs(dra));
         for (int i=1;  i<=steps;  ++i)
         {
            SetVector (Borders[ix],oldra+rastep*i,dec);
            Correct1875(Borders[ix]);
            ++ix;
         }
         SetVector (Borders[ix],ra,dec);
         Correct1875(Borders[ix]);
         ++ix;
      }
      oldra = ra;
   }
   if (BorderGroupCount < MAXBORDERGROUP)
   {
      BorderGroup[BorderGroupCount] = ix;
      ++BorderGroupCount;
   }
   BorderFile.Close();
   */
}
//-------------------------------------------------------------------------------
// void SetVector (GLfloat v[4], Real ra, Real dec)
//-------------------------------------------------------------------------------
// Sets a vector the input ra and dec
//
// v: Output vector
// ra: Right Ascension of vector
// dec: Declination of vector
//------------------------------------------------------------------------------
void GLStars::SetVector (GLfloat v[4], double ra, double dec)
{
    const GLfloat StarRange = 2.0;
    const GLfloat RangeFactor = 0.0;
    double ra_rad = ra * RAD_PER_DEG;
    double dec_rad = dec * RAD_PER_DEG;
    v[0] = StarRange * cos(ra_rad) * cos(dec_rad);
    v[1] = StarRange * sin(ra_rad) * cos(dec_rad);
    v[2] = StarRange * sin(dec_rad);
    v[3] = RangeFactor;
}
//-------------------------------------------------------------------------------
//void Correct1875 (GLfloat v[4])
//-------------------------------------------------------------------------------
// Correct a vector from B1875.0 frame to J2000 frame.  
// Used for constellation borders
//------------------------------------------------------------------------------
void GLStars::Correct1875 (GLfloat v[4])
   {
   GLfloat copy[3];
   GLfloat a1875[3][3];
   a1875[0][0] =   0.99953588F;
   a1875[0][1] =   0.02793679F;
   a1875[0][2] =   0.01214762F;
   a1875[1][0] =   -0.02793679F;
   a1875[1][1] =   0.99960968F;
   a1875[1][2] =   -0.00016969F;
   a1875[2][0] =   -0.01214762F;
   a1875[2][1] =   -0.00016976F;
   a1875[2][2] =   0.99992620F;
   for (int i=0;  i<=3-1;  ++i)
      {
      copy[i] = v[i];
      v[i] = 0;
      }
   for (int i=0;  i<=3-1;  ++i)
      for (int j=0;  j<=3-1;  ++j)
         v[i] += a1875[j][i] * copy[j];
   }
//------------------------------------------------------------------------------
// void InitStars()
//------------------------------------------------------------------------------
/**
 * Initialize the stars and constellations, reading them in from the files.
 */
//------------------------------------------------------------------------------
void GLStars::InitStars()
{
   SetDesiredStarCount(42000);
   
   ReadStars();
   ReadConstellations();
   ReadBorders();
   if (m_MaxDrawStars > m_DesiredStarCount)
      m_MaxDrawStars = m_DesiredStarCount;
}

//------------------------------------------------------------------------------
// void DrawStarsVA(GLfloat ColorAlpha, int starCount, bool drawConstellations)
//------------------------------------------------------------------------------
/**
 * Draw the stars and the constellation lines, if you want them.
 */
//------------------------------------------------------------------------------
void GLStars::DrawStarsVA(GLfloat ColorAlpha, int starCount, bool drawConstellations)
{
   // The colors for the stars, north star, and constellation lines
   SetDesiredStarCount(starCount);
   GLfloat StarWhite[4] = {1.0f, 1.0f, 1.0f, 1.0f};
   GLfloat StarBlue[4] = {0.2f, 0.2f, 1.0f, 1.0f};
   GLfloat LineColor[4] = {0.2f, 0.6f, 0.6f, 1.0f};
   GLfloat NameColor[4] = {0.5f, 0.5f, 0.2f, 1.0f};
   GLfloat BorderColor[4] = {0.15f, 0.15f, 0.10f, 1.0f};
   
   // Set the alpha value
   StarWhite[3] = ColorAlpha;
   StarBlue[3] = ColorAlpha;
   
   // Store the lighting bit. We need a special lighting so the stars and lines will show.
   glPushAttrib(GL_LIGHTING_BIT);
   glLightfv(GL_LIGHT0, GL_AMBIENT, SOURCELIGHT99);
   
   // Anti-aliasing. 
   glEnable(GL_POINT_SMOOTH);
   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
   
   // We want to use vertex arrays for efficiency
   glEnableClientState(GL_VERTEX_ARRAY);
   glDisable(GL_DEPTH_TEST);
   // Point to our array of star vertices
   glVertexPointer(4, GL_FLOAT, 0, m_Stars);

   // First the north star
   glColor4fv(StarBlue);
   
   // Make it big so it stands out
   glPointSize(8.0f);
   // Draw the north star only
   glDrawArrays(GL_POINTS, 0, 1);
   int TotalDrawnStars = 1;
   
   // The rest of the stars are white
   glColor4fv(StarWhite);
   // Point again to the stars array
   glVertexPointer(4, GL_FLOAT, 0, m_Stars);
   
   // Iterate through the groups and draw them all
   for (int i = 1; i <= m_LastGroupUsed; i++)
   {
      // Groups are organized by visual magnitude
      glPointSize(m_PointSize[i]);
      // Draw them all
      glDrawArrays(GL_POINTS, m_GroupIndex[i], m_GroupCount[i]);
      // Count the stars we've drawn so far
      TotalDrawnStars += m_GroupCount[i];
      // If we start getting too many, stop
      if (TotalDrawnStars >= m_DesiredStarCount || TotalDrawnStars >= m_MaxDrawStars)
         break;
   }
   
   // Draw the constellation lines if the user wants them
   if (drawConstellations)
      {
      glColor4fv(LineColor);
      // Point to the vertex array
      glVertexPointer(4, GL_FLOAT, 0, m_CLines);
      // Enable smoothing and make these lines thin
      glEnable(GL_LINE_SMOOTH);
      glLineWidth(0.3f);
      // Draw the lines
      glDrawArrays(GL_LINES, 0, m_NumLines);
      
      glColor4fv(NameColor);
      for (int i = 1; i < m_NumConstellations; i++)
         {
         int count = 0;
         double v[3] = {0,0,0};
         for (int j = m_ConstellationIndex[i][0];  j <= m_ConstellationIndex[i][1];  ++j)
             {
             ++count;
             for (int k=0;  k<=3-1;  ++k)
                v[k] += m_CLines[j][k];
             }
         for (int k=0;  k<=3-1;  ++k)
            v[k] /= count;
         GLfloat w = m_CLines[m_ConstellationIndex[i][0]][3];
         if (w == 0.0) w = 0.00001f;
         DrawStringAt(m_ConstellationNames[i],v[0],v[1],v[2],w);
         }

      glColor4fv(BorderColor);
      // Point to the vertex array
      glVertexPointer(4, GL_FLOAT, 0, m_Borders);
      // Enable smoothing and make these lines thin
      glEnable(GL_LINE_SMOOTH);
      glLineWidth(0.3f);
      // Draw the lines
      for (int i = 0;  i <= m_BorderGroupCount-1;  ++i)
         glDrawArrays(GL_LINE_STRIP, m_BorderGroup[i], m_BorderGroup[i+1]-m_BorderGroup[i]-1);
      }
   
   glEnable(GL_DEPTH_TEST);
   // Disable vertex arrays
   glDisableClientState(GL_VERTEX_ARRAY);
   glDisable(GL_BLEND);
   glPopAttrib();
   }
//------------------------------------------------------------------------------
