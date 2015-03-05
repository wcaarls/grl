/*
 *    Copyright (C) 2012 Erik Schuitema (DBL)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "SimVisWidget.h"

CSimVisWidget::CSimVisWidget(QWidget *parent):
	GLWidget(parent)
{
	mSim = NULL;
	mTimeDisplayEnabled = true;
}

void CSimVisWidget::setSim(CVisualSim* sim)
{
	mSim = sim;
}

void CSimVisWidget::enableTimeDisplay(bool enabled)
{
	mTimeDisplayEnabled = enabled;
}

void CSimVisWidget::drawObject(CSimVisObject* object)
{
	float pos[3];
	float R[12];
	object->getPositionF(pos);
	object->getRotationF(R);
	dsSetColorAlpha(object->mColor.mR, object->mColor.mG, object->mColor.mB, object->mColor.mAlpha);
	dsSetTexture(DS_WOOD);
	switch (object->getType())
	{
		case dtBox:
			{
				float sides[3];
				sides[0] = object->getParam1();
				sides[1] = object->getParam2();
				sides[2] = object->getParam3();
				dsDrawBox(pos, R, sides);
			}
			break;
		case dtSphere:
			{
				dsDrawSphere(pos, R, object->getParam1());
			}
			break;
		case dtCylinder:
			{
				dsDrawCylinder(pos, R, object->getParam1(), object->getParam2());
			}
			break;
		case dtCapsule:
			{
				dsDrawCapsule(pos, R, object->getParam1(), object->getParam2());
			}
			break;
		case dtCone:
			{
				dsDrawCone(pos, R, object->getParam1(), object->getParam2());
			}
			break;
		case dtTriangleStrip:
			{
				dsDrawTriangleStrip(pos, R, object->getVertexData(), object->getNumVertices());
			}
			break;
		default:
			break;
	}
}

void CSimVisWidget::onPaint()
{
	if (mSim == NULL)
	{
		mLogErrorLn("CSimVisWidget::onPaint() cannot be processed because mSim is NULL!");
		return;
	}

	if (!mSim->isInitialized())
	{
		mLogCrawlLn("WARNING: CSimVisWidget::onPaint() cannot be processed because mSim is uninitialized!");
		return;
	}

	// Acquire sim access
	CGenericSimAccess simAccess(mSim);

	// Collect and plot all visualization objects
	mVisObjects.clear();
	mSim->getSimVisObjects(&mVisObjects);
	for (uint32_t iVO=0; iVO<mVisObjects.size(); iVO++)
		drawObject(mVisObjects[iVO]);

	// Release sim access as soon as possible
	simAccess.release();

	if (mTimeDisplayEnabled)
	{
		// Render text
		char timeStr[20];
		sprintf(timeStr, "Time: %.5f", mSim->getTime());

		qglColor(QColor(0, 0, 0, 180));
		// No texture on text
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_TEXTURE_GEN_S);
		glDisable(GL_TEXTURE_GEN_T);
		renderText(8, 12, timeStr);
	}
}

