/*
 *    A COptionVar keeps track of whether it has been assigned or not.
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
 *
 *  Example:
           configSection.get("steptime", &mOptionDbl);
           logNoticeLn(CLog2("temp"), "After reading steptime, isSet() = " << mOptionDbl.isSet() << ", value = " << mOptionDbl);
           mOptionDbl.reset();
           configSection.get("nonexistent", &mOptionDbl);
           logNoticeLn(CLog2("temp"), "After reading rubbish, isSet() = " << mOptionDbl.isSet() << ", value = " << mOptionDbl);

	Output:
           [temp] NTC: After reading steptime, isSet() = 1, value = 0.0133333
           [temp] NTC: After reading rubbish, isSet() = 0, value = 0.0133333
 *
 */

#ifndef OPTIONVARS_H_
#define OPTIONVARS_H_

template<class T>
class COptionVar
{
	protected:
		bool	mSet;
		T		mValue;

	public:
		COptionVar()									{mValue = 0; mSet=false;}

		bool isSet()									{return mSet;}
		void reset()									{mSet=false;}
		// The object can be cast to a const reference of its native type T
		operator const T&()								{return mValue;}
		// Assignment of a value of its native type T
		COptionVar<T>& operator =(const T& newValue)	{mValue = newValue; mSet = true; return *this;}
};

typedef COptionVar<bool>			COptionBool;
typedef COptionVar<int>				COptionInt;
typedef COptionVar<double>			COptionDouble;
typedef COptionVar<char>			COptionChar;
typedef COptionVar<unsigned char>	COptionByte;
typedef COptionVar<unsigned short>	COptionWord;

#endif /* OPTIONVARS_H_ */
