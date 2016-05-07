/*
 *    Generic file/console logger class
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

#pragma once

#ifdef WIN32
  #include <win32_compat.h>
#endif

#include <iostream>
#include <sstream>
#include <fstream>
#include <streambuf>
#include <iomanip>

#define LOG_MAX_CUSTOMLOGS 10	// Number of custom logs available, besides cout and cerr

#ifdef _WIN32
#	include <windows.h>
#	define FOREGROUND_BROWN		(FOREGROUND_RED|FOREGROUND_GREEN)
#	define FOREGROUND_MAGENTA	(FOREGROUND_RED|FOREGROUND_BLUE)
#	define FOREGROUND_CYAN		(FOREGROUND_BLUE|FOREGROUND_GREEN)
#	define FOREGROUND_GRAY		(FOREGROUND_RED|FOREGROUND_BLUE|FOREGROUND_GREEN)
#	define CONSL_UNDERSCORE		COMMON_LVB_UNDERSCORE
#	define CONSL_INTENSITY		FOREGROUND_INTENSITY
#else
#	define FOREGROUND_RED		31
#	define FOREGROUND_GREEN		32
#	define FOREGROUND_BLUE		34
#	define FOREGROUND_BROWN		33
#	define FOREGROUND_MAGENTA	35
#	define FOREGROUND_CYAN		36
#	define FOREGROUND_GRAY		37
#	define CONSL_UNDERSCORE		4
#	define CONSL_INTENSITY		1
//#	define FOREGROUND_HALFBRIGHT	2
#endif

using std::endl;
using std::ends;
using std::flush;

enum ELogLevel {llCrawl, llDebug, llInfo, llNotice, llWarning, llError, llCritical, llClean};

class CLockable
{
  friend class CCriticalSection;

  private:
    pthread_mutex_t mMutex;

  protected:
    void lock(void)
    {
      pthread_mutex_lock(&mMutex);
    }

    void unlock(void)
    {
      pthread_mutex_unlock(&mMutex);
    }

  public:
    CLockable()
    {
      pthread_mutex_init(&mMutex, NULL);
    }
};

class CCriticalSection
{
  private:
    CLockable &mObj;

  public:
    CCriticalSection(CLockable &obj) : mObj(obj)
    {
      mObj.lock();
    }

    ~CCriticalSection()
    {
      mObj.unlock();
    }
};

class CLogStream: public std::ostream, public CLockable
{
  private:
    ELogLevel mLevel;

	public:
		CLogStream(std::stringbuf* buffer): std::ostream(buffer)			{}
		void setLevel(ELogLevel level)										{ mLevel = level; }
		inline ELogLevel getLevel() const									{ return mLevel; }
		virtual void setSystemHeader(const std::string &text)				{ }

		virtual void	enableConsoleOutput(bool bEnabled)					{}
		virtual void	redirectConsoleOutput(FILE* file)					{}
		// enableFileOutput() can return a FILE pointer if it wants the logFactory to close the opened file (RtLog needs it); otherwise, returns NULL.
		virtual FILE*	enableFileOutput(bool bEnabled, const std::string& filename="")	{ return NULL;}
		virtual void	flushFileOutput()									{}	// Flushes to disk if file output is on
		virtual void	enableTimeStamping(bool enabled)					{}
		virtual void	setHeaderText(const std::string &HeaderText)=0;
		virtual std::string& headerText()=0;
		virtual void	setHeaderColor(int HeaderColor)						{}
		virtual void	setMessageColor(int MessageColor)					{}
};

class CLog
{
	protected:
		int			mNumCustomLogStreams;	// Number of custom logs currently in use
		std::string	mCustomLogStreamNames[LOG_MAX_CUSTOMLOGS];

	public:
		CLog():	mNumCustomLogStreams(0)						{}
		virtual ~CLog()										{}
		virtual CLogStream&	cout()=0;
		virtual CLogStream&	cerr()=0;
		virtual CLogStream&	ccustom(const int index)=0;

		// requestCustomLog() returns the index of the custom log with the specified name
		// If the log did not yet exist, it is 'created'
		int	getCustomLogIndex(const std::string& customLogName)
		{
			for (int i=0; i<mNumCustomLogStreams; i++)
				if (mCustomLogStreamNames[i] == customLogName)
					return i;

			// We did not return -> occupy new seat if available
			if (mNumCustomLogStreams < LOG_MAX_CUSTOMLOGS)
			{
				mCustomLogStreamNames[mNumCustomLogStreams] = customLogName;
				mNumCustomLogStreams++;
				return mNumCustomLogStreams-1;
			}
			else
			{
				// Otherwise, return last log
				cerr() << "[ERROR] Cannot create custom log named \"" << customLogName << "\": maximum reached (" << LOG_MAX_CUSTOMLOGS << ")! Returning \"" << mCustomLogStreamNames[LOG_MAX_CUSTOMLOGS-1] << "\" instead." << endl;
				return LOG_MAX_CUSTOMLOGS-1;
			}
		}
};


