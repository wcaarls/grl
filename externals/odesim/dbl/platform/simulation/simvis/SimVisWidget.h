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

#ifndef SIMVISWIDGET_H_
#define SIMVISWIDGET_H_

#include <GLWidget.h>
#include "SimVis.h"

class CSimVisWidget: public GLWidget, public CSimVisLoggable
{
	protected:
		CVisualSim*				mSim;
		bool					mTimeDisplayEnabled;
		CSimVisObjectPtrArray	mVisObjects;


	public:
		CSimVisWidget(QWidget *parent = 0);
		// Drawing functions
		void			drawObject(CSimVisObject* object);

		void			setSim(CVisualSim* sim);
		void			enableTimeDisplay(bool enabled);
		virtual void	onPaint();
};


#endif /* SIMVISWIDGET_H_ */
