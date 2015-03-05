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

#ifndef PLACEABLEOBJECT_H_
#define PLACEABLEOBJECT_H_

// Base class for placeable objects
class CPlaceableObject
{
	public:
		virtual ~CPlaceableObject() { }
	
		// Get the position of this object. If you want (like for drawing objects), provide an additional displacement (dx, dy, dz) within the body's reference frame
		//template<class floattype>
		//virtual void	getPosition(floattype *pos, double dx=0, double dy=0, double dz=0)	{}
		virtual void	getPosition(double *pos, double dx=0, double dy=0, double dz=0)=0;
		void			getPositionF(float *pos, double dx=0, double dy=0, double dz=0)
		{
			double dPos[3];
			getPosition(dPos, dx, dy, dz);
			for (int i=0; i<3; i++)
				pos[i] = (float)dPos[i];
		}
		// Get the rotation matrix of this object.
		//template<class floattype>
		//virtual void	getRotation(floattype *rot)	{}
		virtual void	getRotation(double *rot)=0;
		void			getRotationF(float *rot)
		{
			double dRot[12];
			getRotation(dRot);
			for (int i=0; i<12; i++)
				rot[i] = (float)dRot[i];
		}
};

#endif /* PLACEABLEOBJECT_H_ */
