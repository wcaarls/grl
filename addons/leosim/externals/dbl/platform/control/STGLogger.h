/*
 * STGLogger.h
 *
 *  Created on: Apr 15, 2010
 *      Author: Erik Schuitema
 *
 *
 *  CSTGLogger can be created for any StateTransitionGenerator
 *  to automatically log all state information to file.
 *
 */

#ifndef STGLOGGER_H_
#define STGLOGGER_H_

#include <Log2.h>
#include "STGListener.h"

#define CONST_STGListener_QueueLength	10	// 0 means no maximum

template<class STGStateType>
class CSTGLogger: public CSTGListener<STGStateType>
{
	protected:
		CLog2			mLog;		// Debugging log
		std::string		mFilename;
		CLog2			mMainLog;	// The *data* log where the STG data is going to
		bool			mPaused;
		bool			mIsLogging;	// Indicates if logging is currently running (no matter the value of mPaused!)

	public:
		CSTGLogger(CStateTransitionGenerator<STGStateType>* stg, const std::string& filename, const std::string& nameExtension):
			CSTGListener<STGStateType>(stg),
			mLog("stglogger"),
			mFilename(filename),
			mMainLog("stl" + nameExtension),	// "stl" is short for state transitions logger. The extension can be used to avoid collisions between loggers.
			mPaused(false),
			mIsLogging(false)
		{
		}

		const std::string&	getFilename() const
		{
			return mFilename;
		}

		void			setFilename(const std::string& filename)
		{
			if (mIsLogging)
				mLogErrorLn("Cannot change STG logger filename while logging! Stop logger first.");
			else
				mFilename = filename;
		}

		virtual bool	init()
		{
			bool result= true;
			mMainLog.enableConsoleOutput(false);
			mMainLog.enableFileOutput(true, mFilename);
			mMainLog.setHeaderText("");
			mMainLog(llClean) << this->getState()->toTextHeader();

			result &= this->startListening(stgReceiveAll, 1, CONST_STGListener_QueueLength, "-STGLogger");
			mIsLogging = result;
			return result;
		}

		void			enable(bool enabled)
		{
			mPaused = !enabled;
		}

		virtual bool	deinit()
		{
			bool result = true;
			result &= this->stopListening();
			mMainLog.enableFileOutput(false);
			mIsLogging = false;
			return result;
		}

		// Make sure log() is called in your thread loop function after waitForNewState()
		void			log()
		{
			if (!mPaused)
				// Flush every line to avoid flooding RTPRINTBUF_BUFLEN (by default 2kb, see RtLog.h)
				// Every state->toText() line cannot be larger than this buffer, so make sure RTPRINTBUF_BUFLEN is large enough!
				mMainLog(llClean) << this->getState()->toText() << flush;
		}
};

#endif /* STGLOGGER_H_ */
