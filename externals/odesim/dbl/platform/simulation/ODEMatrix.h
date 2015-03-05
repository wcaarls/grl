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

#ifndef CODEMATRIX_H_
#define CODEMATRIX_H_

#include <Configuration.h>
#include <ode/ode.h>	//TODO: remove dependency on ODE!
#include <Log2.h>

class CODEMatrix3
{
	protected:
		CLog2		mLog;
		dMatrix3	mMatrix;

	public:
		CODEMatrix3(): mLog("matrix3")		{}

		operator const dMatrix3& () const	{ return mMatrix;}
		operator dMatrix3& ()				{ return mMatrix;}
		void	setRotation();
		void	setOrientation(dReal ax, dReal ay, dReal az, dReal bx, dReal by, dReal bz)
		{
			dRFrom2Axes(mMatrix, ax, ay, az, bx, by, bz);
		}

		void	setRotation(dReal ax, dReal ay, dReal az, dReal angle)
		{
			dRFromAxisAndAngle(mMatrix, ax, ay, az, angle);
		}

		CODEMatrix3 operator *(const CODEMatrix3& other)
		{
			CODEMatrix3 result;
			dMULTIPLY0_333(result.mMatrix, this->mMatrix, other.mMatrix);
			return result;
		}

		void	store(float* dest)
		{
			for (int i=0; i<12; i++)
				dest[i] = (float)mMatrix[i];
		}

		void	store(double* dest)
		{
			for (int i=0; i<12; i++)
			dest[i] = (double)mMatrix[i];
		}

		bool	readConfig(const CConfigSection &configSection)	// Always pass the PARENT section which contains a <rotation> or <orientation> node!
		{
			bool configresult = true;

			CConfigSection orientationSection	= configSection.section("orientation");
			CConfigSection rotationSection		= configSection.section("rotation");
			if (!orientationSection.isNull())
			{
				double ax, ay, az, bx, by, bz;
				CConfigSection xAxisSection = orientationSection.section("Xaxis");
				configresult &= mLogAssert(xAxisSection.get("x", &ax));
				configresult &= mLogAssert(xAxisSection.get("y", &ay));
				configresult &= mLogAssert(xAxisSection.get("z", &az));
				CConfigSection yAxisSection = orientationSection.section("Yaxis");
				configresult &= mLogAssert(yAxisSection.get("x", &bx));
				configresult &= mLogAssert(yAxisSection.get("y", &by));
				configresult &= mLogAssert(yAxisSection.get("z", &bz));
				setOrientation(ax, ay, az, bx, by, bz);
			}
			else if (!rotationSection.isNull())
				// Try reading a rotation
			{
				double ax, ay, az, angle;
				CConfigSection axisSection = rotationSection.section("axis");
				configresult &= mLogAssert(axisSection.get("x", &ax));
				configresult &= mLogAssert(axisSection.get("y", &ay));
				configresult &= mLogAssert(axisSection.get("z", &az));
				configresult &= mLogAssert(rotationSection.get("angle", &angle));
				setRotation(ax, ay, az, angle);
			}

			return configresult;
		}
};

#endif /* CODEMATRIX_H_ */
