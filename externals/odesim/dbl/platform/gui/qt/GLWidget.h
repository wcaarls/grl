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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtOpenGL/QGLWidget>

#ifdef USE_FFMPEG
#include <QVideoEncoder.h>
#endif

typedef unsigned char byte;

// texture numbers
#define DS_NONE   0	/* uses the current color instead of a texture */
#define DS_WOOD   1

enum ECamViewMode
{
	cmStandardView,
	cmCenterView
};


// Drawstuff classes Image and Texture
class Image
{
		int image_width,image_height;
		byte *image_data;
	public:
		Image (char *filename);
		// load from PPM file
		~Image();
		int width() { return image_width; }
		int height() { return image_height; }
		byte *data() { return image_data; }
};

class Texture
{
		Image *image;
		GLuint name;
	public:
		Texture (char *filename);
		~Texture();
		void bind (int modulate);
};

// Our GLWidget class
class GLWidget : public QGLWidget
{
	Q_OBJECT

	private:
		GLuint	makeObject();
		void	quad(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2, GLdouble x3, GLdouble y3, GLdouble x4, GLdouble y4);
		void	extrude(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2);
		void	normalizeAngle(int *angle);

		// Drawstuff ugliness
		void	drawCylinder (float l, float r, float zoffset);
		void	drawCone(float length, float radius);

		GLuint	object;
		int		xRot;
		int		yRot;
		int		zRot;
		double	camDistance;
		QPoint	lastPos;
		int		mouseMode;
		QColor	trolltechGreen;
		QColor	trolltechPurple;

		// Drawstuff stuff. Dirty dirty..!
		float	view_xyz[3];	// position x,y,z
		float	view_hpr[3];	// heading, pitch, roll (degrees)
		int		use_textures;
		int		use_shadows;
		float	lightX, lightY;
		Texture	*sky_texture;
		Texture	*ground_texture;
		Texture	*wood_texture;
		float	color[4];	// current r,g,b,alpha color
		int		tnum;			// current texture number

	protected:
#ifdef USE_FFMPEG
		QVideoEncoder	mVideoEncoder;
#endif
		int				mVidPixAlignment;
		ECamViewMode	mCamViewMode;
		unsigned int	mWindowWidth, mWindowHeight;
		bool			mGraphicsEnabled;

		void			initializeGL();
		virtual void	paintGL();
		virtual void	resizeGL(int width, int height);
		virtual void	mousePressEvent(QMouseEvent *event);
		virtual void	mouseReleaseEvent(QMouseEvent *event);
		virtual void	mouseMoveEvent(QMouseEvent *event);
		virtual void	keyPressEvent (QKeyEvent *event);

		// Drawstuff stuff..
		void	dsGetViewpoint (float xyz[3], float hpr[3]);
		void	dsSetViewpoint(float xyz[3], float hpr[3]);
		void	dsSetTexture(int texture_number);
		void	dsSetColor(float red, float green, float blue);
		void	dsSetColorAlpha (float red, float green, float blue, float alpha);
		void	dsDrawSphereShadow (float px, float py, float pz, float radius);
		void	dsSetShadowTransform();
		void	dsAdjustLightWithViewpoint();

		void	dsCamMotion (int mode, int deltax, int deltay);
		void	dsWrapCameraAngles();
		void	dsStartGraphics(const char *texturePath);
		void	dsStopGraphics();
		// Standard drawing of the cloudy sky, ground and axis pyramids
		void	drawSky (float view_xyz[3]);
		void	drawGround();
		void	drawPyramidGrid();

		// Drawing functions of standard objects
		void	dsDrawBox (const float pos[3], const float R[12], const float sides[3]);
		void	dsDrawSphere (const float pos[3], const float R[12], float radius);
		void	dsDrawCylinder (const float pos[3], const float R[12], float length, float radius);
		void	dsDrawCapsule(const float pos[3], const float R[12], float length, float radius);
		void	dsDrawCone(const float pos[3], const float R[12], float length, float radius);
		void	dsDrawTriangleStrip(const float pos[3], const float R[12], const float* vertexData, const int numVertices);		void	dsSetupDrawingMode();
		void	dsSetShadowDrawingMode();

		// Override these functions to add drawing and keypressing functionality
		virtual void	onKeyPress(QKeyEvent *event)	{}
		virtual void	onPaint()						{}
		virtual void	onInit()						{}

	public:
		GLWidget(QWidget *parent = 0);
		~GLWidget();
		void	init();	// ALWAYS CALL INIT AFTER CONSTRUCTING THE OBJECT!

		QSize	minimumSizeHint() const;
		QSize	sizeHint() const;

		// Update the screen using updateGL();

		bool	isGraphicsEnabled();	// Lekker engels...
		void	enableGraphics(bool enabled);
		int		dsGetTextures();
		void	dsSetTextures(int a);
		int		dsGetShadows();
		void	dsSetShadows (int a);

		QString	selectVideoFileAndFormat(); // Returns filename
		bool	startVideoEncoding(const QString& filename, int bitrate, int keyframeInterval, int fps, bool multithreaded);
		bool	stopVideoEncoding();

	public slots:
		void	setXRotation(int angle);
		void	setYRotation(int angle);
		void	setZRotation(int angle);

	signals:
		void	xRotationChanged(int angle);
		void	yRotationChanged(int angle);
		void	zRotationChanged(int angle);

};

#endif /* GLWIDGET_H_ */
