/****************************************************************************
**
** Copyright (C) 2005-2006 Trolltech ASA. All rights reserved.
**
** This file is part of the documentation of the Qt Toolkit.
**
** This file may be used under the terms of the GNU General Public
** License version 2.0 as published by the Free Software Foundation
** and appearing in the file LICENSE.GPL included in the packaging of
** this file.  Please review the following information to ensure GNU
** General Public Licensing requirements will be met:
** http://www.trolltech.com/products/qt/opensource.html
**
** If you are unsure which license is appropriate for your use, please
** review the following information:
** http://www.trolltech.com/products/qt/licensing.html or contact the
** sales department at sales@trolltech.com.
**
** This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
** WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
**
****************************************************************************/

#include <QtGui>

#include <math.h>

#include "GLWidget.h"
#include <sys/stat.h>
#include <GL/glu.h>


//***************************************************************************
// Stuff copied from drawstuff.cpp



#define DEFAULT_PATH_TO_TEXTURES "textures/"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// constants to convert degrees to radians and the reverse
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)

// light vector. LIGHTZ is implicitly 1
//#define LIGHTX (1.0f)
//#define LIGHTY (0.4f)

// ground and sky
#define SHADOW_INTENSITY (0.65f)
#define GROUND_R (0.5f) 	// ground color for when there's no texture
#define GROUND_G (0.5f)
#define GROUND_B (0.3f)

const float ground_scale = 1.0f/1.0f;	// ground texture scale (1/size)
const float ground_ofsx = 0.5;		// offset of ground texture
const float ground_ofsy = 0.5;
const float sky_scale = 1.0f/4.0f;	// sky texture scale (1/size)
const float sky_height = 1.0f;		// sky height above viewpoint

//***************************************************************************
// misc mathematics stuff

#ifndef dCROSS
#define dCROSS(a,op,b,c) \
  (a)[0] op ((b)[1]*(c)[2] - (b)[2]*(c)[1]); \
  (a)[1] op ((b)[2]*(c)[0] - (b)[0]*(c)[2]); \
  (a)[2] op ((b)[0]*(c)[1] - (b)[1]*(c)[0]);
#endif


#ifndef dDOT
inline float dDOT (const float *a, const float *b)
  { return ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2]); }
#endif

/*
static void normalizeVector3 (float v[3])
{
  float len = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
  if (len <= 0.0f) {
    v[0] = 1;
    v[1] = 0;
    v[2] = 0;
  }
  else {
    len = 1.0f / (float)sqrt(len);
    v[0] *= len;
    v[1] *= len;
    v[2] *= len;
  }
}
*/

// Basic Drawstuff error functions

void dsPrintMessage(const char *msg1, const char *msg2, va_list ap)
{
  fflush (stderr);
  fflush (stdout);
  fprintf (stderr,"\n%s: ",msg1);
  vfprintf (stderr,msg2,ap);
  fprintf (stderr,"\n");
  fflush (stderr);
}

void dsError(const char *msg, ...)
{
	va_list ap;
	va_start (ap,msg);
	dsPrintMessage ("Error",msg,ap);
	// I HATE exit().
	//exit (1);
}

#define dsErrorReturn(msg, ...) do {dsError(msg, __VA_ARGS__); return; } while (0)

//***************************************************************************
// PPM image object

// skip over whitespace and comments in a stream.

void dsSkipWhiteSpace (char *filename, FILE *f)
{
  int c,d;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) dsError ("unexpected end of file in \"%s\"",filename);

    // skip comments
    if (c == '#') {
      do {
	d = fgetc(f);
	if (d==EOF) dsError ("unexpected end of file in \"%s\"",filename);
      } while (d != '\n');
      continue;
    }

    if (c > ' ') {
      ungetc (c,f);
      return;
    }
  }
}


// read a number from a stream, this return 0 if there is none (that's okay
// because 0 is a bad value for all PPM numbers anyway).

int dsReadNumber (char *filename, FILE *f)
{
  int c,n=0;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) dsError ("unexpected end of file in \"%s\"",filename);
    if (c >= '0' && c <= '9') n = n*10 + (c - '0');
    else {
      ungetc (c,f);
      return n;
    }
  }
}


Image::Image (char *filename) : image_width(0), image_height(0), image_data(NULL)
{
  FILE *f = fopen (filename,"rb");
  if (!f) dsErrorReturn ("Can't open image file `%s'",filename);

  // read in header
  if (fgetc(f) != 'P' || fgetc(f) != '6')
  {
	fclose(f);
    dsErrorReturn ("image file \"%s\" is not a binary PPM (no P6 header)",filename);
  }
  dsSkipWhiteSpace (filename,f);

  // read in image parameters
  image_width = dsReadNumber (filename,f);
  dsSkipWhiteSpace (filename,f);
  image_height = dsReadNumber (filename,f);
  dsSkipWhiteSpace (filename,f);
  int max_value = dsReadNumber (filename,f);

  // check values
  if (image_width < 1 || image_height < 1)
  {
	fclose(f);
    dsErrorReturn ("bad image file \"%s\"",filename);
  }
  if (max_value != 255)
  {
	fclose(f);
    dsErrorReturn ("image file \"%s\" must have color range of 255",filename);
  }

  // read either nothing, LF (10), or CR,LF (13,10)
  int c = fgetc(f);
  if (c == 10) {
    // LF
  }
  else if (c == 13) {
    // CR
    c = fgetc(f);
    if (c != 10) ungetc (c,f);
  }
  else ungetc (c,f);

  // read in rest of data
  image_data = new byte [image_width*image_height*3];
  if (fread (image_data,image_width*image_height*3,1,f) != 1)
  {
    fclose(f);
    dsError ("Can not read data from image file `%s'",filename);
  }
  fclose (f);
}


Image::~Image()
{
  if (image_data)
    delete[] image_data;
}


//***************************************************************************
// Texture object.

Texture::Texture (char *filename)
{
	image = new Image (filename);
	glGenTextures (1,&name);
	glBindTexture (GL_TEXTURE_2D,name);

	// set pixel unpacking mode
	glPixelStorei (GL_UNPACK_SWAP_BYTES, 0);
	glPixelStorei (GL_UNPACK_ROW_LENGTH, 0);
	glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei (GL_UNPACK_SKIP_ROWS, 0);
	glPixelStorei (GL_UNPACK_SKIP_PIXELS, 0);

	//glTexImage2D (GL_TEXTURE_2D, 0, 3, image->width(), image->height(), 0, GL_RGB, GL_UNSIGNED_BYTE, image->data());
	if (image->data())
		gluBuild2DMipmaps (GL_TEXTURE_2D, 3, image->width(), image->height(), GL_RGB, GL_UNSIGNED_BYTE, image->data());
	else
	{
		unsigned char data[] = {128, 128, 128};
		gluBuild2DMipmaps (GL_TEXTURE_2D, 3, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, data);
	}

	// set texture parameters - will these also be bound to the texture???
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
}


Texture::~Texture()
{
  delete image;
  glDeleteTextures (1,&name);
}


void Texture::bind (int modulate)
{
  glBindTexture (GL_TEXTURE_2D,name);
  glTexEnvi (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,
	     modulate ? GL_MODULATE : GL_DECAL);
}
//***************************************************************************
// OpenGL utility stuff

static void setCamera (float x, float y, float z, float h, float p, float r)
{
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();
  glRotatef (90, 0,0,1);
  glRotatef (90, 0,1,0);
  glRotatef (r, 1,0,0);
  glRotatef (p, 0,1,0);
  glRotatef (-h, 0,0,1);
  glTranslatef (-x,-y,-z);
}

// sets the material color, not the light color

static void setColor (float r, float g, float b, float alpha)
{
  GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
  light_ambient[0] = r*0.3f;
  light_ambient[1] = g*0.3f;
  light_ambient[2] = b*0.3f;
  light_ambient[3] = alpha;
  light_diffuse[0] = r*0.7f;
  light_diffuse[1] = g*0.7f;
  light_diffuse[2] = b*0.7f;
  light_diffuse[3] = alpha;
  light_specular[0] = r*0.2f;
  light_specular[1] = g*0.2f;
  light_specular[2] = b*0.2f;
  light_specular[3] = alpha;
  glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
  glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, light_diffuse);
  glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, light_specular);
  glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 5.0f);
}


static void setTransform (const float pos[3], const float R[12])
{
  GLfloat matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}
/*
static void setTransformD (const double pos[3], const double R[12])
{
  GLdouble matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixd (matrix);
}
*/

static void drawBox (const float sides[3])
{
  float lx = sides[0]*0.5f;
  float ly = sides[1]*0.5f;
  float lz = sides[2]*0.5f;

  // sides
  glBegin (GL_TRIANGLE_STRIP);
  glNormal3f (-1,0,0);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,-ly,lz);
  glVertex3f (-lx,ly,-lz);
  glVertex3f (-lx,ly,lz);
  glNormal3f (0,1,0);
  glVertex3f (lx,ly,-lz);
  glVertex3f (lx,ly,lz);
  glNormal3f (1,0,0);
  glVertex3f (lx,-ly,-lz);
  glVertex3f (lx,-ly,lz);
  glNormal3f (0,-1,0);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,-ly,lz);
  glEnd();

  // top face
  glBegin (GL_TRIANGLE_FAN);
  glNormal3f (0,0,1);
  glVertex3f (-lx,-ly,lz);
  glVertex3f (lx,-ly,lz);
  glVertex3f (lx,ly,lz);
  glVertex3f (-lx,ly,lz);
  glEnd();

  // bottom face
  glBegin (GL_TRIANGLE_FAN);
  glNormal3f (0,0,-1);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,ly,-lz);
  glVertex3f (lx,ly,-lz);
  glVertex3f (lx,-ly,-lz);
  glEnd();
}

// This is recursively subdivides a triangular area (vertices p1,p2,p3) into
// smaller triangles, and then draws the triangles. All triangle vertices are
// normalized to a distance of 1.0 from the origin (p1,p2,p3 are assumed
// to be already normalized). Note this is not super-fast because it draws
// triangles rather than triangle strips.

static void drawPatch (float p1[3], float p2[3], float p3[3], int level)
{
  int i;
  if (level > 0) {
    float q1[3],q2[3],q3[3];		 // sub-vertices
    for (i=0; i<3; i++) {
      q1[i] = 0.5f*(p1[i]+p2[i]);
      q2[i] = 0.5f*(p2[i]+p3[i]);
      q3[i] = 0.5f*(p3[i]+p1[i]);
    }
    float length1 = (float)(1.0/sqrt(q1[0]*q1[0]+q1[1]*q1[1]+q1[2]*q1[2]));
    float length2 = (float)(1.0/sqrt(q2[0]*q2[0]+q2[1]*q2[1]+q2[2]*q2[2]));
    float length3 = (float)(1.0/sqrt(q3[0]*q3[0]+q3[1]*q3[1]+q3[2]*q3[2]));
    for (i=0; i<3; i++) {
      q1[i] *= length1;
      q2[i] *= length2;
      q3[i] *= length3;
    }
    drawPatch (p1,q1,q3,level-1);
    drawPatch (q1,p2,q2,level-1);
    drawPatch (q1,q2,q3,level-1);
    drawPatch (q3,q2,p3,level-1);
  }
  else {
    glNormal3f (p1[0],p1[1],p1[2]);
    glVertex3f (p1[0],p1[1],p1[2]);
    glNormal3f (p2[0],p2[1],p2[2]);
    glVertex3f (p2[0],p2[1],p2[2]);
    glNormal3f (p3[0],p3[1],p3[2]);
    glVertex3f (p3[0],p3[1],p3[2]);
  }
}

// draw a sphere of radius 1

static int sphere_quality = 1;

void drawSphere()
{
  // icosahedron data for an icosahedron of radius 1.0
# define ICX 0.525731112119133606f
# define ICZ 0.850650808352039932f
  static GLfloat idata[12][3] = {
    {-ICX, 0, ICZ},
    {ICX, 0, ICZ},
    {-ICX, 0, -ICZ},
    {ICX, 0, -ICZ},
    {0, ICZ, ICX},
    {0, ICZ, -ICX},
    {0, -ICZ, ICX},
    {0, -ICZ, -ICX},
    {ICZ, ICX, 0},
    {-ICZ, ICX, 0},
    {ICZ, -ICX, 0},
    {-ICZ, -ICX, 0}
  };

  static int index[20][3] = {
    {0, 4, 1},	  {0, 9, 4},
    {9, 5, 4},	  {4, 5, 8},
    {4, 8, 1},	  {8, 10, 1},
    {8, 3, 10},   {5, 3, 8},
    {5, 2, 3},	  {2, 7, 3},
    {7, 10, 3},   {7, 6, 10},
    {7, 11, 6},   {11, 0, 6},
    {0, 1, 6},	  {6, 1, 10},
    {9, 0, 11},   {9, 11, 2},
    {9, 2, 5},	  {7, 2, 11},
  };

  static GLuint listnum = 0;
  if (listnum==0) {
    listnum = glGenLists (1);
    glNewList (listnum,GL_COMPILE);
    glBegin (GL_TRIANGLES);
    for (int i=0; i<20; i++) {
      drawPatch (&idata[index[i][2]][0],&idata[index[i][1]][0],
		 &idata[index[i][0]][0],sphere_quality);
    }
    glEnd();
    glEndList();
  }
  glCallList (listnum);
}

// normalizeVector() and calcNormal() from: http://nehe.gamedev.net/data/lessons/lesson.asp?lesson=36

// Normalizes a vector
bool normalizeVector(float vector[3])
{
	float length;							// Holds Unit Length
	float invLength;
	// Calculates The Length Of The Vector
	length = (float)sqrt((vector[0]*vector[0]) + (vector[1]*vector[1]) + (vector[2]*vector[2]));

	if(length != 0.0f)
	{
		invLength = 1.0f/length;
		vector[0] *= invLength;
		vector[1] *= invLength;
		vector[2] *= invLength;
		return true;
	}
	else
		return false;
}

void negateVector(float vector[3])
{
	vector[0] = -vector[0];
	vector[1] = -vector[1];
	vector[2] = -vector[2];
}

// v[9] = [x0 y0 z0 x1 y1 z1 x2 y2 z2]
bool calcNormal(const float v[9], float out[3])
{
	float v1[3],v2[3];						// Vector 1 (x,y,z) & Vector 2 (x,y,z)
	static const int x = 0;						// Define X Coord
	static const int y = 1;						// Define Y Coord
	static const int z = 2;						// Define Z Coord

	// Finds The Vector Between 2 Points By Subtracting
	// The x,y,z Coordinates From One Point To Another.

	// Calculate The Vector From Point 1 To Point 0
	v1[x] = v[0*3+x] - v[1*3+x];					// Vector 1.x=Vertex[0].x-Vertex[1].x
	v1[y] = v[0*3+y] - v[1*3+y];					// Vector 1.y=Vertex[0].y-Vertex[1].y
	v1[z] = v[0*3+z] - v[1*3+z];					// Vector 1.z=Vertex[0].y-Vertex[1].z
	// Calculate The Vector From Point 2 To Point 1
	v2[x] = v[1*3+x] - v[2*3+x];					// Vector 2.x=Vertex[0].x-Vertex[1].x
	v2[y] = v[1*3+y] - v[2*3+y];					// Vector 2.y=Vertex[0].y-Vertex[1].y
	v2[z] = v[1*3+z] - v[2*3+z];					// Vector 2.z=Vertex[0].z-Vertex[1].z
	// Compute The Cross Product To Give Us A Surface Normal
	out[x] = v1[y]*v2[z] - v1[z]*v2[y];				// Cross Product For Y - Z
	out[y] = v1[z]*v2[x] - v1[x]*v2[z];				// Cross Product For X - Z
	out[z] = v1[x]*v2[y] - v1[y]*v2[x];				// Cross Product For X - Y

	return normalizeVector(out);						// Normalize The Vectors
}

// vertexData of the form [x0 y0 z0 x1 y1 z1 ...]
void drawStrip(const float* vertexData, const int numVertices)
{
	float normal[3];
	glBegin (GL_TRIANGLE_STRIP);
	// Add first triangle manually
	calcNormal(vertexData, normal);
	glNormal3f (normal[0], normal[1], normal[2]);
	glVertex3f (vertexData[0], vertexData[1], vertexData[2]);
	glVertex3f (vertexData[3], vertexData[4], vertexData[5]);
	glVertex3f (vertexData[6], vertexData[7], vertexData[8]);

	float neg = -1.0f;
	for (int iV=3; iV<numVertices; iV++)
	{
		if (calcNormal(vertexData + iV*3, normal))
			glNormal3f (neg*normal[0], neg*normal[1], neg*normal[2]);
		glVertex3f (vertexData[iV*3], vertexData[iV*3+1], vertexData[iV*3+2]);
		neg *= -1.0f;
	}
	glEnd();
}

void GLWidget::dsDrawSphereShadow (float px, float py, float pz, float radius)
{
  // calculate shadow constants based on light vector
  static int init=0;
  static float len2,len1,scale;
  if (!init) {
    len2 = lightX*lightX + lightY*lightY;
    len1 = 1.0f/(float)sqrt(len2);
    scale = (float) sqrt(len2 + 1);
    init = 1;
  }

  // map sphere center to ground plane based on light vector
  px -= lightX*pz;
  py -= lightY*pz;

  const float kx = 0.96592582628907f;
  const float ky = 0.25881904510252f;
  float x=radius, y=0;

  glBegin (GL_TRIANGLE_FAN);
  for (int i=0; i<24; i++) {
    // for all points on circle, scale to elongated rotated shadow and draw
    float x2 = (lightX*x*scale - lightY*y)*len1 + px;
    float y2 = (lightY*x*scale + lightX*y)*len1 + py;
    glTexCoord2f (x2*ground_scale+ground_ofsx,y2*ground_scale+ground_ofsy);
    glVertex3f (x2,y2,0);

    // rotate [x,y] vector
    float xtmp = kx*x - ky*y;
    y = ky*x + kx*y;
    x = xtmp;
  }
  glEnd();
}

// draw a capped cylinder of length l and radius r, aligned along the x axis

static int capped_cylinder_quality = 3;

static void drawCapsule (float l, float r)
{
  int i,j;
  float tmp,nx,ny,nz,start_nx,start_ny,a,ca,sa;
  // number of sides to the cylinder (divisible by 4):
  const int n = capped_cylinder_quality*4;

  l *= 0.5;
  a = float(M_PI*2.0)/float(n);
  sa = (float) sin(a);
  ca = (float) cos(a);

  // draw cylinder body
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,l);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,-l);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw first cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    float start_nx2 =  ca*start_nx + sa*start_ny;
    float start_ny2 = -sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,l+nx2*r);
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,l+nx*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  // draw second cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    float start_nx2 = ca*start_nx - sa*start_ny;
    float start_ny2 = sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,-l+nx*r);
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,-l+nx2*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

//  glPopMatrix();
}


// draw a cylinder of length l and radius r, aligned along the z axis

void GLWidget::drawCylinder (float l, float r, float zoffset)
{
  int i;
  float tmp,ny,nz,a,ca,sa;
  const int n = 24;	// number of sides to the cylinder (divisible by 4)

  l *= 0.5;
  a = float(M_PI*2.0)/float(n);
  sa = (float) sin(a);
  ca = (float) cos(a);

  // draw cylinder body
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,l+zoffset);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,-l+zoffset);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw top cap
  glShadeModel (GL_FLAT);
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,1);
  glVertex3d (0,0,l+zoffset);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,1);
    glVertex3d (ny*r,nz*r,l+zoffset);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw bottom cap
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,-1);
  glVertex3d (0,0,-l+zoffset);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,-1);
    glVertex3d (ny*r,nz*r,-l+zoffset);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny + sa*nz;
    nz = -sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();
}

void GLWidget::drawCone(float length, float radius)
{
	  const int n = 24;	// number of sides to the cylinder (divisible by 4)
	  float a = float(M_PI*2.0)/float(n);
	  float sa = (float) sin(a);
	  float ca = (float) cos(a);
	  float l = length/2;

	  // draw top cap
	  glShadeModel (GL_FLAT);
	  float nx=1, ny=0;		  // normal vector = (nx,ny,nz)
	  float nz = radius / sqrt(radius*radius + length*length);
	  float nf = 1.0f - nz*nz;

	  glBegin (GL_TRIANGLE_FAN);
	  glNormal3d (0,0,1);
	  glVertex3d (0, 0, l);	// The cone top
	  for (int i=0; i<=n; i++)
	  {
	    glNormal3d (nf*nx, nf*ny, nz);
	    glVertex3d (nx*radius, ny*radius, -l);
	    // rotate nx,ny
	    float tmp = ca*nx - sa*ny;
	    ny = sa*nx + ca*ny;
	    nx = tmp;
	  }
	  glEnd();

	  // draw bottom cap
	  nx=1; ny=0;		  // normal vector = (0,ny,nz)
	  glBegin (GL_TRIANGLE_FAN);
	  glNormal3d (0,0,-1);
	  glVertex3d (0,0,-l);
	  for (int i=0; i<=n; i++) {
	    if (i==1 || i==n/2+1)
	      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
	    glNormal3d (0,0,-1);
	    glVertex3d (nx*radius,ny*radius,-l);
	    if (i==1 || i==n/2+1)
	      setColor (color[0],color[1],color[2],color[3]);

	    // rotate nx,ny
	    float tmp = ca*nx + sa*ny;
	    ny = -sa*nx + ca*ny;
	    nx = tmp;
	  }
	  glEnd();
}

// set shadow projection transform

void GLWidget::dsSetShadowTransform()
{
  GLfloat matrix[16];
  for (int i=0; i<16; i++) matrix[i] = 0;
  matrix[0]=1;
  matrix[5]=1;
  matrix[8]=-lightX;
  matrix[9]=-lightY;
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}

int GLWidget::dsGetShadows()
{
  return use_shadows;
}

void GLWidget::dsSetShadows (int a)
{
  use_shadows = (a != 0);
}

int GLWidget::dsGetTextures()
{
  return use_textures;
}

void GLWidget::dsSetTextures(int a)
{
  use_textures = (a != 0);
}

void GLWidget::dsSetTexture(int texture_number)
{
  tnum = texture_number;
}

void GLWidget::dsSetColor(float red, float green, float blue)
{
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  color[3] = 1;
}

void GLWidget::dsSetColorAlpha (float red, float green, float blue, float alpha)
{
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  color[3] = alpha;
}

void GLWidget::dsGetViewpoint(float xyz[3], float hpr[3])
{
	if (xyz)
	{
		xyz[0] = view_xyz[0];
		xyz[1] = view_xyz[1];
		xyz[2] = view_xyz[2];
	}
	if (hpr)
	{
		hpr[0] = view_hpr[0];
		hpr[1] = view_hpr[1];
		hpr[2] = view_hpr[2];
	}
}

void GLWidget::dsSetViewpoint(float xyz[3], float hpr[3])
{
	if (xyz)
	{
		view_xyz[0] = xyz[0];
		view_xyz[1] = xyz[1];
		view_xyz[2] = xyz[2];
	}
	if (hpr)
	{
		view_hpr[0] = hpr[0];
		view_hpr[1] = hpr[1];
		view_hpr[2] = hpr[2];
		dsWrapCameraAngles();
	}
	dsAdjustLightWithViewpoint();
}

void GLWidget::dsAdjustLightWithViewpoint()
{
	// Set light to follow ('assist') the viewpoint
	float yaw = atan2(view_xyz[1], view_xyz[0]);//180.0f + view_hpr[0]*M_PI/180.0f;
	lightX = cos(yaw - 0.3);
	lightY = sin(yaw - 0.3);
}

void GLWidget::dsStartGraphics(const char *texturePath)
{
	printf("[DrawStuff] Started loading textures..\n");

	struct stat dummystat;
	char prefix[1024];
	if (texturePath != NULL)
		strcpy(prefix,texturePath);
	else
		strcpy(prefix, DEFAULT_PATH_TO_TEXTURES);
	for (int i=0; i<2; i++)
	{
		if (stat(prefix, &dummystat) != 0)
		{
			// Put "../" in front
			char tempbuf[1024];
			strcpy(tempbuf, prefix);
			strcpy(prefix, "../");
			strcat(prefix, tempbuf);
		}
		else
			break;
	}

	char *s = (char*) alloca (strlen(prefix) + 20);

	strcpy (s,prefix);
	strcat (s,"/sky.ppm");
	sky_texture = new Texture (s);

	strcpy (s,prefix);
	strcat (s,"/ground.ppm");
	ground_texture = new Texture (s);

	strcpy (s,prefix);
	strcat (s,"/wood.ppm");
	wood_texture = new Texture (s);
	printf("[DrawStuff] Textures loaded\n");
}

void GLWidget::dsStopGraphics()
{
  delete sky_texture;
  delete ground_texture;
  delete wood_texture;
  sky_texture = 0;
  ground_texture = 0;
  wood_texture = 0;
}

void GLWidget::drawSky (float view_xyz[3])
{
  glDisable (GL_LIGHTING);
  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    sky_texture->bind (0);
  }
  else {
    glDisable (GL_TEXTURE_2D);
    glColor3f (0,0.5,1.0);
  }

  // make sure sky depth is as far back as possible
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LEQUAL);
  glDepthRange (1,1);

  const float ssize = 1000.0f;
  static float offset = 0.0f;

  float x = ssize*sky_scale;
  float z = view_xyz[2] + sky_height;

  glBegin (GL_QUADS);
  glNormal3f (0,0,-1);
  glTexCoord2f (-x+offset,-x+offset);
  glVertex3f (-ssize+view_xyz[0],-ssize+view_xyz[1],z);
  glTexCoord2f (-x+offset,x+offset);
  glVertex3f (-ssize+view_xyz[0],ssize+view_xyz[1],z);
  glTexCoord2f (x+offset,x+offset);
  glVertex3f (ssize+view_xyz[0],ssize+view_xyz[1],z);
  glTexCoord2f (x+offset,-x+offset);
  glVertex3f (ssize+view_xyz[0],-ssize+view_xyz[1],z);
  glEnd();

  offset = offset + 0.002f;
  if (offset > 1) offset -= 1;

  glDepthFunc (GL_LESS);
  glDepthRange (0,1);
}


void GLWidget::drawGround()
{
  glDisable (GL_LIGHTING);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  // glDepthRange (1,1);

  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    ground_texture->bind (0);
  }
  else {
    glDisable (GL_TEXTURE_2D);
    glColor3f (GROUND_R,GROUND_G,GROUND_B);
  }

  // ground fog seems to cause problems with TNT2 under windows
  /*
  GLfloat fogColor[4] = {0.5, 0.5, 0.5, 1};
  glEnable (GL_FOG);
  glFogi (GL_FOG_MODE, GL_EXP2);
  glFogfv (GL_FOG_COLOR, fogColor);
  glFogf (GL_FOG_DENSITY, 0.05f);
  glHint (GL_FOG_HINT, GL_NICEST); // GL_DONT_CARE);
  glFogf (GL_FOG_START, 1.0);
  glFogf (GL_FOG_END, 5.0);
  */

  const float gsize = 100.0f;
  const float offset = 0; // -0.001f; ... polygon offsetting doesn't work well

  glBegin (GL_QUADS);
  glNormal3f (0,0,1);
  glTexCoord2f (-gsize*ground_scale + ground_ofsx,
		-gsize*ground_scale + ground_ofsy);
  glVertex3f (-gsize,-gsize,offset);
  glTexCoord2f (gsize*ground_scale + ground_ofsx,
		-gsize*ground_scale + ground_ofsy);
  glVertex3f (gsize,-gsize,offset);
  glTexCoord2f (gsize*ground_scale + ground_ofsx,
		gsize*ground_scale + ground_ofsy);
  glVertex3f (gsize,gsize,offset);
  glTexCoord2f (-gsize*ground_scale + ground_ofsx,
		gsize*ground_scale + ground_ofsy);
  glVertex3f (-gsize,gsize,offset);
  glEnd();

  glDisable (GL_FOG);
}


void GLWidget::drawPyramidGrid()
{
  // setup stuff
  glEnable (GL_LIGHTING);
  glDisable (GL_TEXTURE_2D);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);

  // draw the pyramid grid
  for (int i=-1; i<=1; i++) {
    for (int j=-1; j<=1; j++) {
      glPushMatrix();
      glTranslatef ((float)i,(float)j,(float)0);
      if (i==1 && j==0) setColor (1,0,0,1);
      else if (i==0 && j==1) setColor (0,0,1,1);
      else setColor (1,1,0,1);
      const float k = 0.01f;
      glBegin (GL_TRIANGLE_FAN);
      glNormal3f (0,-1,1);
      glVertex3f (0,0,k);
      glVertex3f (-k,-k,0);
      glVertex3f ( k,-k,0);
      glNormal3f (1,0,1);
      glVertex3f ( k, k,0);
      glNormal3f (0,1,1);
      glVertex3f (-k, k,0);
      glNormal3f (-1,0,1);
      glVertex3f (-k,-k,0);
      glEnd();
      glPopMatrix();
    }
  }
}

GLWidget::GLWidget(QWidget *parent):
	QGLWidget(parent),
	mVidPixAlignment(16)
{
	object = 0;
	xRot			= 0;
	yRot			= 0;
	zRot			= 0;
	camDistance		= -1.5;

	mWindowWidth	= 0;
	mWindowHeight	= 0;

	lightX = 1.0f;
	lightY = 0.4f;

	trolltechGreen = QColor::fromCmykF(0.40, 0.0, 1.0, 0.0);
	trolltechPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);

	// Request keyboard focus
	setFocusPolicy(Qt::StrongFocus);

	// Set standard view mode
	mCamViewMode		= cmStandardView;
	mGraphicsEnabled	= true;
}

GLWidget::~GLWidget()
{
	makeCurrent();
	glDeleteLists(object, 1);

	dsStopGraphics();
}

void GLWidget::init()
{
	// Init drawstuff 'motion model' (who came up with this term?)
	view_xyz[0] = 2;
	view_xyz[1] = 0;
	view_xyz[2] = 1;
	view_hpr[0] = 180;
	view_hpr[1] = 0;
	view_hpr[2] = 0;

	mouseMode		= 0;
	use_textures	= 1;
	use_shadows		= 1;
	sky_texture		= NULL;
	ground_texture	= NULL;
	wood_texture	= NULL;

	memset(color, 0, 4*sizeof(color[0]));
	tnum = 0;			// current texture number

	makeCurrent();

	// Start the graphics
	dsStartGraphics(ODESIM_TEXTURE_DIR);

	// Print user directions
    fprintf
    (
      stderr,
      "\n"
      "Graphics rendering environment:\n"
      "   Ctrl-T : toggle textures.\n"
      "   Ctrl-S : toggle shadows.\n"
      "   Ctrl-V : print current viewpoint coordinates (x,y,z,h,p,r).\n"
      "   Ctrl+M : Switch between standard-view mode and center-view mode.\n"
      //"   Ctrl-X : exit.\n"
      "\n"
      "Change the camera position by clicking + dragging in the window.\n"
      "Standard-view mode:\n"
      "   Left button - pan and tilt.\n"
      "   Right button - forward and sideways.\n"
      "   Left + Right button (or middle button) - sideways and up.\n"
      "Center-view mode:\n"

      "\n"
    );

    // Side view of Leo by default
    float xyz[3] = {0.0f, 0.8f, 0.25f}, hpr[3] = {-90.0f, 0.0f, 0.0f};
    dsSetViewpoint(xyz, hpr);
    updateGL();

    // Perform user initialization
    onInit();
}

QSize GLWidget::minimumSizeHint() const
{
	return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
	return QSize(400, 400);
}

void GLWidget::setXRotation(int angle)
{
	normalizeAngle(&angle);
	if (angle != xRot)
	{
		xRot = angle;
		emit xRotationChanged(angle);
		updateGL();
	}
}

void GLWidget::setYRotation(int angle)
{
	normalizeAngle(&angle);
	if (angle != yRot)
	{
		yRot = angle;
		emit yRotationChanged(angle);
		updateGL();
	}
}

void GLWidget::setZRotation(int angle)
{
	normalizeAngle(&angle);
	if (angle != zRot)
	{
		zRot = angle;
		emit zRotationChanged(angle);
		updateGL();
	}
}

void GLWidget::initializeGL()
{
	qglClearColor(trolltechGreen.dark());
	object = makeObject();
	glShadeModel(GL_FLAT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
}

bool GLWidget::isGraphicsEnabled()
{
	return mGraphicsEnabled;
}

void GLWidget::enableGraphics(bool enabled)
{
	mGraphicsEnabled = enabled;
}

void GLWidget::paintGL()
{
	if (mGraphicsEnabled)
	{
		// setup stuff
		glEnable (GL_LIGHTING);
		glEnable (GL_LIGHT0);
		glDisable (GL_TEXTURE_2D);
		glDisable (GL_TEXTURE_GEN_S);
		glDisable (GL_TEXTURE_GEN_T);
		glShadeModel (GL_FLAT);
		glEnable (GL_DEPTH_TEST);
		glDepthFunc (GL_LESS);
		glEnable (GL_CULL_FACE);
		glCullFace (GL_BACK);
		glFrontFace (GL_CCW);

		// setup viewport
		glViewport (0, 0, mWindowWidth, mWindowHeight);
		glMatrixMode (GL_PROJECTION);
		glLoadIdentity();
		const float vnear = 0.1f;
		const float vfar = 100.0f;
		const float k = 0.8f;     // view scale, 1 = +/- 45 degrees
		if (mWindowWidth >= mWindowHeight)
		{
			float k2 = float(mWindowHeight)/float(mWindowWidth);
			glFrustum (-vnear*k,vnear*k,-vnear*k*k2,vnear*k*k2,vnear,vfar);
		}
		else
		{
			float k2 = float(mWindowWidth)/float(mWindowHeight);
			glFrustum (-vnear*k*k2,vnear*k*k2,-vnear*k,vnear*k,vnear,vfar);
		}

		// setup lights. it makes a difference whether this is done in the
		// GL_PROJECTION matrix mode (lights are scene relative) or the
		// GL_MODELVIEW matrix mode (lights are camera relative, bad!).
		static GLfloat light_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
		static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
		static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
		glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
		glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
		glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
		glColor3f (1.0, 1.0, 1.0);

		// clear the window
		glClearColor (0.5,0.5,0.5,0);
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// snapshot camera position (in MS Windows it is changed by the GUI thread)
		float view2_xyz[3];
		float view2_hpr[3];
		memcpy (view2_xyz,view_xyz,sizeof(float)*3);
		memcpy (view2_hpr,view_hpr,sizeof(float)*3);

		// Set the camera
		if (mCamViewMode == cmStandardView)
		{
			setCamera (view2_xyz[0],view2_xyz[1],view2_xyz[2],
					view2_hpr[0],view2_hpr[1],view2_hpr[2]);
			// set the light position (for some reason we have to do this in model view.
			GLfloat light_position[] = { lightX, lightY, 1.0, 0.0 };
			glLightfv (GL_LIGHT0, GL_POSITION, light_position);
		}
		else	// cmCenterView
		{
			glMatrixMode (GL_MODELVIEW);
			glLoadIdentity();
			glTranslated(0.0, 0.0, camDistance);
			glRotated(xRot / 16.0, 1.0, 0.0, 0.0);
			glRotated(yRot / 16.0, 0.0, 1.0, 0.0);
			glRotated(zRot / 16.0, 0.0, 0.0, 1.0);
		}


		  // draw the background (ground, sky etc)
		drawSky (view2_xyz);
		drawGround();

		// draw the little markers on the ground
		drawPyramidGrid();

		// leave openGL in a known state - flat shaded white, no textures
		glEnable (GL_LIGHTING);
		glDisable (GL_TEXTURE_2D);
		glShadeModel (GL_FLAT);
		glEnable (GL_DEPTH_TEST);
		glDepthFunc (GL_LESS);
		glColor3f (1,1,1);
		setColor (1,1,1,1);

		// draw the rest of the objects. set drawing state first.
		color[0] = 1;
		color[1] = 1;
		color[2] = 1;
		color[3] = 1;
		tnum = 0;

		// User-drawing
		onPaint();

#ifdef USE_FFMPEG
		if(mVideoEncoder.isOk())
		{
			// Capture frame
			QImage frame = grabFrameBuffer(true);
			int width = frame.width() - (frame.width()%mVidPixAlignment);
			int height = frame.height() - (frame.height()%mVidPixAlignment);
			QImage croppedFrame = frame.copy(0, 0, width, height);
			//croppedFrame.save("croppedframe.png", "PNG");
			if (mVideoEncoder.encodeImage(croppedFrame) < 0)
				printf("Failed to capture image! Did you resize the window?\n");
			else
				printf("Captured image!\n");
		}
#endif
	}
	else
	{
		// clear the window
		glClearColor (0.5,0.5,0.5,0);
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		// Render text
		int halfTextWidth = 30;	// Estimate for now
		renderText(width()/2 - halfTextWidth, height()/2, "Graphics disabled");
	}
}

void GLWidget::resizeGL(int width, int height)
{
	mWindowWidth	= width;
	mWindowHeight	= height;
	/*
	int side = qMin(width, height);
	glViewport((width - side) / 2, (height - side) / 2, side, side);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-0.5, +0.5, +0.5, -0.5, 4.0, 15.0);
	glMatrixMode(GL_MODELVIEW);
	*/

}

void GLWidget::dsWrapCameraAngles()
{
  for (int i=0; i<3; i++) {
    while (view_hpr[i] > 180) view_hpr[i] -= 360;
    while (view_hpr[i] < -180) view_hpr[i] += 360;
  }
}

void GLWidget::dsCamMotion (int mode, int deltax, int deltay)
{
  float side = 0.01f * float(deltax);
  float fwd = (mode==4) ? (0.01f * float(deltay)) : 0.0f;
  float s = (float) sin (view_hpr[0]*DEG_TO_RAD);
  float c = (float) cos (view_hpr[0]*DEG_TO_RAD);

  if (mode==1) {
    view_hpr[0] += float (deltax) * 0.5f;
    view_hpr[1] += float (deltay) * 0.5f;
  }
  else {
    view_xyz[0] += -s*side + c*fwd;
    view_xyz[1] += c*side + s*fwd;
    if (mode==2 || mode==5) view_xyz[2] += 0.01f * float(deltay);
  }
  dsWrapCameraAngles();
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	// No mouse movements if graphics are disabled
	if (!mGraphicsEnabled)
		return;

    if (event->button() == Qt::LeftButton)
    	mouseMode |= 1;
    if (event->button() == Qt::MidButton)
    	mouseMode |= 2;
    if (event->button() == Qt::RightButton)
    	mouseMode |= 4;
    lastPos = event->pos();
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
	// No mouse movements if graphics are disabled
	if (!mGraphicsEnabled)
		return;

    if (event->button() == Qt::LeftButton)
    	mouseMode &= (~1);
    if (event->button() == Qt::MidButton)
    	mouseMode &= (~2);
    if (event->button() == Qt::RightButton)
    	mouseMode &= (~4);
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	// No mouse movements if graphics are disabled
	if (!mGraphicsEnabled)
		return;

	int dx = event->x() - lastPos.x();
	int dy = event->y() - lastPos.y();
	if (mCamViewMode == cmStandardView)
	{
		dsCamMotion (mouseMode, dx, dy);

		// Set light to follow ('assist') the viewpoint
		dsAdjustLightWithViewpoint();
	}
	else
	{
		// cmCenterView
		if (event->buttons() & Qt::RightButton)
		{
			// Right button
			camDistance += 0.03 * dy;
		}
	/*			else
				{
					// Left button only
					setXRotation(xRot + 8 * dy);
					setYRotation(yRot + 8 * dx);
				}*/
		else if (event->buttons() & Qt::LeftButton)
		{
			// Right button only
			setXRotation(xRot + 8 * dy);
			setZRotation(zRot + 8 * dx);
		}
	}

	lastPos = event->pos();
	updateGL();
}

void GLWidget::keyPressEvent (QKeyEvent *event)
{
	// No keypress events if graphics are disabled
	if (!mGraphicsEnabled)
		return;

	// Process keypresses of the form Alt + ..
	if (event->modifiers() & Qt::AltModifier)
	{
		switch(event->key())
		{
			case 't': case 'T':
				{
					float xyz[3] = {0.0f, 0.0f, 3.0f}, hpr[3] = {90.0f, -90.0f, 0.0f};
					dsSetViewpoint(xyz, hpr);
					updateGL();
				}
				break;
			case 's': case 'S':
				{
					float xyz[3] = {0.0f, 0.8f, 0.25f}, hpr[3] = {-90.0f, 0.0f, 0.0f};
					dsSetViewpoint(xyz, hpr);
					updateGL();
				}
				break;
		}
	}
	else
	// Process keypresses of the form Ctrl + ..
	if (event->modifiers() & Qt::ControlModifier)
	{
		switch (event->key())
		{
			case 't': case 'T':
				dsSetTextures (dsGetTextures() ^ 1);
				updateGL();
				break;
			case 's': case 'S':
				dsSetShadows (dsGetShadows() ^ 1);
				updateGL();
				break;
			case 'v': case 'V':
				{
					float xyz[3],hpr[3];
					dsGetViewpoint (xyz,hpr);
					printf ("Viewpoint = (%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)\n",
						xyz[0],xyz[1],xyz[2],hpr[0],hpr[1],hpr[2]);
				}
				break;
			case 'm': case 'M':
				{
					if (mCamViewMode == cmStandardView)
					{
						mCamViewMode = cmCenterView;
						printf("Switching to center-view mode.\n");
					}
					else
					{
						mCamViewMode = cmStandardView;
						printf("Switching to standard-view mode.\n");
					}
					updateGL();
				}
				break;
			/*
			case 'w': case 'W':
				writeframes ^= 1;
				if (writeframes)
				printf ("Now writing frames to PPM files\n");
				break;
			*/
		}
	}
	onKeyPress(event);
}

GLuint GLWidget::makeObject()
{
	GLuint list = glGenLists(1);
	glNewList(list, GL_COMPILE);

	glBegin(GL_QUADS);

	GLdouble x1 = +0.06;
	GLdouble y1 = -0.14;
	GLdouble x2 = +0.14;
	GLdouble y2 = -0.06;
	GLdouble x3 = +0.08;
	GLdouble y3 = +0.00;
	GLdouble x4 = +0.30;
	GLdouble y4 = +0.22;

	quad(x1, y1, x2, y2, y2, x2, y1, x1);
	quad(x3, y3, x4, y4, y4, x4, y3, x3);

	extrude(x1, y1, x2, y2);
	extrude(x2, y2, y2, x2);
	extrude(y2, x2, y1, x1);
	extrude(y1, x1, x1, y1);
	extrude(x3, y3, x4, y4);
	extrude(x4, y4, y4, x4);
	extrude(y4, x4, y3, x3);

	const double Pi = 3.14159265358979323846;
	const int NumSectors = 200;

	for (int i = 0; i < NumSectors; ++i) {
		double angle1 = (i * 2 * Pi) / NumSectors;
		GLdouble x5 = 0.30 * sin(angle1);
		GLdouble y5 = 0.30 * cos(angle1);
		GLdouble x6 = 0.20 * sin(angle1);
		GLdouble y6 = 0.20 * cos(angle1);

		double angle2 = ((i + 1) * 2 * Pi) / NumSectors;
		GLdouble x7 = 0.20 * sin(angle2);
		GLdouble y7 = 0.20 * cos(angle2);
		GLdouble x8 = 0.30 * sin(angle2);
		GLdouble y8 = 0.30 * cos(angle2);

		quad(x5, y5, x6, y6, x7, y7, x8, y8);

		extrude(x6, y6, x7, y7);
		extrude(x8, y8, x5, y5);
	}

	glEnd();

	glEndList();
	return list;
}

void GLWidget::quad(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2,
					GLdouble x3, GLdouble y3, GLdouble x4, GLdouble y4)
{
	qglColor(trolltechPurple);

	glVertex3d(x1, y1, -0.05);
	glVertex3d(x2, y2, -0.05);
	glVertex3d(x3, y3, -0.05);
	glVertex3d(x4, y4, -0.05);

	glVertex3d(x4, y4, +0.05);
	glVertex3d(x3, y3, +0.05);
	glVertex3d(x2, y2, +0.05);
	glVertex3d(x1, y1, +0.05);
}

void GLWidget::extrude(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2)
{
	qglColor(trolltechPurple.dark(250 + int(100 * x1)));

	glVertex3d(x1, y1, +0.05);
	glVertex3d(x2, y2, +0.05);
	glVertex3d(x2, y2, -0.05);
	glVertex3d(x1, y1, -0.05);
}

void GLWidget::normalizeAngle(int *angle)
{
	while (*angle < 0)
		*angle += 360 * 16;
	while (*angle > 360 * 16)
		*angle -= 360 * 16;
}

// Drawstuff object drawing functions
// sets lighting and texture modes, sets current color
void GLWidget::dsSetupDrawingMode()
{
  glEnable (GL_LIGHTING);
  if (tnum) {
    if (use_textures) {
      glEnable (GL_TEXTURE_2D);
      wood_texture->bind (1);
      glEnable (GL_TEXTURE_GEN_S);
      glEnable (GL_TEXTURE_GEN_T);
      glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
      glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
      static GLfloat s_params[4] = {1.0f,1.0f,0.0f,1};
      static GLfloat t_params[4] = {0.817f,-0.817f,0.817f,1};
      glTexGenfv (GL_S,GL_OBJECT_PLANE,s_params);
      glTexGenfv (GL_T,GL_OBJECT_PLANE,t_params);
    }
    else {
      glDisable (GL_TEXTURE_2D);
    }
  }
  else {
    glDisable (GL_TEXTURE_2D);
  }
  setColor (color[0],color[1],color[2],color[3]);

  if (color[3] < 1) {
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  }
  else {
    glDisable (GL_BLEND);
  }
}

void GLWidget::dsSetShadowDrawingMode()
{
  glDisable (GL_LIGHTING);
  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    ground_texture->bind (1);
    glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
    glEnable (GL_TEXTURE_2D);
    glEnable (GL_TEXTURE_GEN_S);
    glEnable (GL_TEXTURE_GEN_T);
    glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_EYE_LINEAR);
    glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_EYE_LINEAR);
    static GLfloat s_params[4] = {ground_scale,0,0,ground_ofsx};
    static GLfloat t_params[4] = {0,ground_scale,0,ground_ofsy};
    glTexGenfv (GL_S,GL_EYE_PLANE,s_params);
    glTexGenfv (GL_T,GL_EYE_PLANE,t_params);
  }
  else {
    glDisable (GL_TEXTURE_2D);
    glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
	       GROUND_B*SHADOW_INTENSITY);
  }
  glDepthRange (0,0.9999);
}


void GLWidget::dsDrawBox (const float pos[3], const float R[12], const float sides[3])
{
	dsSetupDrawingMode();
	glShadeModel (GL_FLAT);
	setTransform (pos,R);
	drawBox (sides);
	glPopMatrix();

	if (use_shadows)
	{
		dsSetShadowDrawingMode();
		dsSetShadowTransform();
		setTransform (pos,R);
		drawBox (sides);
		glPopMatrix();
		glPopMatrix();
		glDepthRange (0,1);
	}
}

void GLWidget::dsDrawSphere (const float pos[3], const float R[12], float radius)
{
	dsSetupDrawingMode();
	glEnable (GL_NORMALIZE);
	glShadeModel (GL_SMOOTH);
	setTransform (pos,R);
	glScaled (radius,radius,radius);
	drawSphere();
	glPopMatrix();
	glDisable (GL_NORMALIZE);

	// draw shadows
	if (use_shadows)
	{
		glDisable (GL_LIGHTING);
		if (use_textures)
		{
			ground_texture->bind (1);
			glEnable (GL_TEXTURE_2D);
			glDisable (GL_TEXTURE_GEN_S);
			glDisable (GL_TEXTURE_GEN_T);
			glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
		}
		else
		{
			glDisable (GL_TEXTURE_2D);
			glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY, GROUND_B*SHADOW_INTENSITY);
		}
		glShadeModel (GL_FLAT);
		glDepthRange (0,0.9999);
		dsDrawSphereShadow (pos[0],pos[1],pos[2],radius);
		glDepthRange (0,1);
	}
}

void GLWidget::dsDrawCylinder (const float pos[3], const float R[12], float length, float radius)
{
	dsSetupDrawingMode();
	glShadeModel (GL_SMOOTH);
	setTransform (pos,R);
	drawCylinder (length,radius,0);
	glPopMatrix();

	if (use_shadows)
	{
		dsSetShadowDrawingMode();
		dsSetShadowTransform();
		setTransform (pos,R);
		drawCylinder (length,radius,0);
		glPopMatrix();
		glPopMatrix();
		glDepthRange (0,1);
	}
}

void GLWidget::dsDrawCapsule(const float pos[3], const float R[12], float length, float radius)
{
	dsSetupDrawingMode();
	glShadeModel (GL_SMOOTH);
	setTransform (pos,R);
	drawCapsule (length,radius);
	glPopMatrix();

	if (use_shadows)
	{
		  dsSetShadowDrawingMode();
		  dsSetShadowTransform();
		  setTransform (pos,R);
		  drawCapsule (length,radius);
		  glPopMatrix();
		  glPopMatrix();
		  glDepthRange (0,1);
	}
}

void GLWidget::dsDrawCone(const float pos[3], const float R[12], float length, float radius)
{
	dsSetupDrawingMode();
	glShadeModel(GL_SMOOTH);
	setTransform(pos,R);
	drawCone(length, radius);
	glPopMatrix();

	if (use_shadows)
	{
		 // no shadow yet
	}
}

void GLWidget::dsDrawTriangleStrip(const float pos[3], const float R[12], const float* vertexData, const int numVertices)
{
	dsSetupDrawingMode();
	glShadeModel (GL_SMOOTH);
	setTransform (pos,R);
	drawStrip(vertexData, numVertices);
	glPopMatrix();

	// No shadow
}

bool GLWidget::startVideoEncoding(const QString& filename, int bitrate, int keyframeInterval, int fps, bool multithreaded)
{
#ifdef USE_FFMPEG
	// Determine image width and height - cropped to multiples of 8 pixels
	int alWidth		= width() - (width()%mVidPixAlignment);
	int alHeight	= height() - (height()%mVidPixAlignment);

	// Create the video file
	return mVideoEncoder.createFile(filename, alWidth, alHeight, bitrate, keyframeInterval, fps, multithreaded);
#else
  return false;
#endif
}

bool GLWidget::stopVideoEncoding()
{
#ifdef USE_FFMPEG
	// Close the video file
	return mVideoEncoder.close();
#else
  return false;
#endif
}

QString GLWidget::selectVideoFileAndFormat()
{
#ifdef USE_FFMPEG
	return mVideoEncoder.selectFileAndAVFormat();
#else
  return "";
#endif
}

