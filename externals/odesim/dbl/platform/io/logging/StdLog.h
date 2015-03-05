/*
 * StdLog.h
 *
 * Standard implementation of CLog for non-realtime logging to the console and to file.
 *
 *  Created on: Jul 30, 2009
 *      Author: Erik Schuitema
 */

#ifndef STDLOG_H_
#define STDLOG_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <streambuf>
#include <iomanip>
#include "Log.h"

#ifdef _WIN32
#include <windows.h>
#include <win32_compat.h>
#endif

class CStdStringbuf: public std::stringbuf
{
	protected:
		pthread_mutex_t *mGlobalLogLock;
		bool		mOutputToConsole;
		bool		mOutputToFile;
		bool		mTimeStamping;
		std::string	mHeaderText;
		std::string mSystemHeader;
		int			mHeaderColor;
		int			mMessageColor;
		FILE		*mFileOut;
#ifdef _WIN32
		HANDLE		mConsoleHandle;
#endif

		// sync(): synchronize the content of the buffer with the designated output
		virtual int sync()
		{
			// Lock globally if desired. This makes sure that the log line is not interrupted by other logging threads.
			if (mGlobalLogLock != NULL)
				pthread_mutex_lock(mGlobalLogLock);

			if (mOutputToConsole || mOutputToFile)
			{
				// See basic_stringbuf::str() for similar code. However, in that code, I couldn't understand the use of egptr():
				// "The current egptr() may not be the actual string end."

				if (this->pptr())
				{
					int len = (int)(this->pptr() - this->pbase());
					if (len > 0)
					{
						// Null-terminate the string before outputting it (this is necessary)
#ifdef _MSC_VER
						if (this->pptr() == this->epptr())
							len--;
#else
						if (len > (int)_M_string.capacity() - 1)
							len = _M_string.capacity() - 1;
#endif
						// Writing in the 'unused' space of the buffer shouldn't do any harm
						// so we don't have to back it up
						this->pbase()[len] = '\0';

						// Call rt_printf and/or rt_fprintf to output the temp buffer, preceded by the header text
						if (mOutputToConsole)
						{
							// Set header color
#ifdef _WIN32
							SetConsoleTextAttribute(mConsoleHandle, mHeaderColor);
#else
							printf("\033[%dm", mHeaderColor);
#endif
							// Print header
							fputs(mHeaderText.c_str(), stdout);
							// Reset all console attributes (because it may contain other attributes besides color that are not reset when setting new color)
#ifdef _WIN32
							SetConsoleTextAttribute(mConsoleHandle, FOREGROUND_GRAY);
#else
							printf("\033[0m");	// reset all attributes
#endif
							// write time of sending if desired
							if (mTimeStamping)
							{
								timespec absTime;
							    clock_gettime(CLOCK_REALTIME, &absTime);
								printf("[TS:%llu] ", absTime.tv_sec*(long long int)(1E9) + absTime.tv_nsec);
							}
							// Set message color
#ifdef _WIN32
							SetConsoleTextAttribute(mConsoleHandle, mMessageColor);
#else
							printf("\033[%dm", mMessageColor);
#endif
							// Print message
							fputs(this->mSystemHeader.c_str(), stdout);
							fputs(this->pbase(), stdout);
							// Reset all console attributes
#ifdef _WIN32
							SetConsoleTextAttribute(mConsoleHandle, FOREGROUND_GRAY);
#else
							printf("\033[0m");	// reset all attributes
#endif
						}
						if (mOutputToFile && mFileOut != NULL)
						{
							fputs(mHeaderText.c_str(), mFileOut);
							fputs(this->pbase(), mFileOut);
						}

						// Adjust the write buffer pointers. This effectively empties the buffer.
#ifdef _MSC_VER
						setg(pbase(), pbase(), epptr());
						setp(pbase(), epptr());
#else
						_M_sync(const_cast<char_type*>(_M_string.data()), 0, 0);
#endif
					}
				}
			}
			int result = std::stringbuf::sync();

			// Unlock if desired
			if (mGlobalLogLock != NULL)
				pthread_mutex_unlock(mGlobalLogLock);

			return result;
		}

	public:
		CStdStringbuf(pthread_mutex_t *globalLogLock=NULL):
			mGlobalLogLock(globalLogLock),
			mOutputToConsole(true),
			mOutputToFile(false),
			mTimeStamping(false),
			mHeaderColor(0),
			mMessageColor(0),
			mFileOut(NULL)
		{
#ifdef _WIN32
			mConsoleHandle = GetStdHandle(STD_OUTPUT_HANDLE);
#endif
		}

		~CStdStringbuf()
		{
			// Force a last sync. Disable the locking mechanism, because the logfactory might destroy itself, and its locking mutex!
			mGlobalLogLock = NULL;
			sync();
			// Force file closure
			enableFileOutput(false);
		}

		void	enableConsoleOutput(bool bEnabled)					{mOutputToConsole = bEnabled;}
		void	enableFileOutput(bool bEnabled, const std::string& filename="")
		{
			if (bEnabled)
			{	// Open file
				// First close the old one if existent
				if (mFileOut != NULL)
				{
					// First, empty our internal buffer to the file buffer using sync()
					sync();
					// Then, close the file (which will flush its own buffer to disc)
					fclose(mFileOut);
				}
				mFileOut = fopen(filename.c_str(), "wt");
			}
			else
			{	// Close file
				if (mFileOut != NULL)
				{
					// First, empty our internal buffer to the file buffer using sync()
					sync();
					// Then, close the file (which will flush its own buffer to disc)
					fclose(mFileOut);
					mFileOut = NULL;
				}
			}

			if (bEnabled && mFileOut == NULL)
			{
				std::cerr << "Unable to enable file output" << std::endl;
				mOutputToFile	= false;
			}
			else
				mOutputToFile	= bEnabled;
		}
		void	flushFileOutput()
		{
			if (mOutputToFile)
				if (mFileOut != NULL)
					fflush(mFileOut);
		}
		void	enableTimeStamping(bool enabled)					{mTimeStamping	= enabled;}
		void	setHeaderText(const std::string &headerText)		{mHeaderText	= headerText;}
		std::string& headerText()									{return mHeaderText;}
		void	setHeaderColor(int headerColor)						{mHeaderColor	= headerColor;}
		void	setMessageColor(int messageColor)					{mMessageColor	= messageColor;}
		void  setSystemHeader(const std::string &text) { mSystemHeader = text;}
};

class CStdLogStream : public CLogStream
{
	protected:
		CStdStringbuf mBuffer;

	public:
		CStdLogStream(pthread_mutex_t *globalLogLock=NULL):
			CLogStream(&mBuffer),
			mBuffer(globalLogLock)
		{
		}

		void	flushFileOutput()									{mBuffer.flushFileOutput();}
		void	enableConsoleOutput(bool bEnabled)					{return mBuffer.enableConsoleOutput(bEnabled);}
		// mBuffer.enableFileOutput closes files locally, so it can return NULL to the log factory.
		FILE*	enableFileOutput(bool bEnabled, const std::string& filename="")	{mBuffer.enableFileOutput(bEnabled, filename); return NULL;}
		void	enableTimeStamping(bool enabled)					{mBuffer.enableTimeStamping(enabled);}
		void	setHeaderText(const std::string &headerText)		{mBuffer.setHeaderText(headerText);}
		std::string& headerText()									{return mBuffer.headerText();}
		void	setHeaderColor(int headerColor)						{mBuffer.setHeaderColor(headerColor);}
		void	setMessageColor(int messageColor)					{mBuffer.setMessageColor(messageColor);}
		void  setSystemHeader(const std::string &text) { mBuffer.setSystemHeader(text); }

};

class CStdLog: public CLog
{
	protected:
		CStdLogStream	mCout;
		CStdLogStream	mCerr;
		CStdLogStream	mCcustom[LOG_MAX_CUSTOMLOGS];

	public:
		CLogStream&	cout()						{return mCout;}
		CLogStream&	cerr()						{return mCerr;}
		CLogStream&	ccustom(const int index)	{return mCcustom[index];}
};

#endif /* STDLOG_H_ */
