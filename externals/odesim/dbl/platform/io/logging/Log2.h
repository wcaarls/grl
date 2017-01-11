/*
 *    Log class with levels
 *    Copyright (C) 2012 Wouter Caarls (DBL)
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
 * Basic usage: define a member variable
 *
 * class Class {
 *   CLog2 mLog;
 * };
 *
 * and initialize it in the constructor
 *
 *  Class() : mLog("group") { }
 *
 * where group is the name of the logging group (with default header [group]).
 * Then in any member function just use
 *
 *   mLogInfoLn("This is a log message");
 *
 * The usual options (console/file output, header text and colors) can be set with
 *
 *   mLog.setHeaderText("[MyFancyHeader] ");
 *
 * All these options will carry over to all log classes with the same group name.
 * The logging level is set with
 *
 *   mLog.setLevel(llCrawl);
 *
 * The logging level of all groups can be set at once with
 *
 *   gLogFactory->setLevel(llDebug);
 *
 * IMPORTANT NOTICE: For now, these classes are not thread-safe.
 * However, StdLog.h ensures that log lines are not interrupted.
 */

#ifndef __LOG2_H_INCLUDED
#define __LOG2_H_INCLUDED

#include <map>
#include <vector>

#include <Log.h>

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define AT __FILE__ ":" TOSTRING(__LINE__)

#define LOG2(obj, str, level) do {if ((level) >= (obj).getLevel()) { CCriticalSection logsection((obj).getStream()); (obj)(level) << str;}} while (0)
//#define LOG2(obj, str, level) do {std::cout << str;} while (0)

#ifdef DEBUG
  #define logCrawl(obj, str)    LOG2(obj, str, llCrawl)
#else
  #define logCrawl(obj, str)    do {} while (0)
#endif

#define logDebug(obj, str)      LOG2(obj, str, llDebug)
#define logInfo(obj, str)       LOG2(obj, str, llInfo)
#define logNotice(obj, str)     LOG2(obj, str, llNotice)
#define logWarning(obj, str)    LOG2(obj, str, llWarning)
#define logError(obj, str)      LOG2(obj, str, llError)
#define logCritical(obj, str)   LOG2(obj, str, llCritical)
#define logClean(obj, str)   	LOG2(obj, str, llClean)

#define logCrawlLn(obj, str)    logCrawl(obj, str << std::endl)
#define logDebugLn(obj, str)    logDebug(obj, str << std::endl)
#define logInfoLn(obj, str)     logInfo(obj, str << std::endl)
#define logNoticeLn(obj, str)   logNotice(obj, str << std::endl)
#define logWarningLn(obj, str)  logWarning(obj, str << std::endl)
#define logErrorLn(obj, str)    logError(obj, str << std::endl)
#define logCriticalLn(obj, str)	logCritical(obj, str << std::endl)
#define logCleanLn(obj, str)	logClean(obj, str << std::endl)

#define mLogCrawl(str)          logCrawl(mLog, str)
#define mLogDebug(str)          logDebug(mLog, str)
#define mLogInfo(str)           logInfo(mLog, str)
#define mLogNotice(str)         logNotice(mLog, str)
#define mLogWarning(str)        logWarning(mLog, str)
#define mLogError(str)          logError(mLog, str)
#define mLogCritical(str)       logCritical(mLog, str)
#define mLogClean(str)       	logClean(mLog, str)

#define mLogCrawlLn(str)        logCrawlLn(mLog, str)
#define mLogDebugLn(str)        logDebugLn(mLog, str)
#define mLogInfoLn(str)         logInfoLn(mLog, str)
#define mLogNoticeLn(str)       logNoticeLn(mLog, str)
#define mLogWarningLn(str)      logWarningLn(mLog, str)
#define mLogErrorLn(str)        logErrorLn(mLog, str)
#define mLogCriticalLn(str)     logCriticalLn(mLog, str)
#define mLogCleanLn(str) 	    logCleanLn(mLog, str)
#define mLogAssert(B)			logAssertLnInternal(mLog, "Assertion failed at " AT  ": " #B, llError, B)

// Standard header text delimiters
#define LOG2HDRDELIMLEFT		"["
#define LOG2HDRDELIMRIGHT		"] "
#define LOG2OPENFILESSOFTLIMIT	100	// This is a soft limit. You can open more files, but this may lead to memory (re)allocation (unwanted in real-time apps).

// Forward declaration
class CLog2;

class CLog2Factory
{
	friend class CLog2;
  protected:
    ELogLevel	mLevel;
    bool		mTimeStamping;
    std::map<const std::string, CLogStream*>	mLogs;
    std::vector<FILE*>							mOpenFiles;

    /**
     * equalizeHeaderTexts()
     * Can be called in getLog() after creating and adding a new log
     * to make all header texts of equal length.
     */
    void equalizeHeaderTexts()
    {
    	// Determine max header text length
    	unsigned int maxWidth=0;
        for (std::map<const std::string, CLogStream*>::iterator it = mLogs.begin();
             it != mLogs.end(); ++it)
        {
        	if (it->second->headerText().length() > maxWidth)
        		maxWidth = it->second->headerText().length();
        }

        // Make all header text lengths equal. Skip the empty ones, they are empty for a reason!
        for (std::map<const std::string, CLogStream*>::iterator it = mLogs.begin();
             it != mLogs.end(); ++it)
        {
        	// First, pad with spaces
        	if (!it->second->headerText().empty())	// Skip empty headers
        	{
				it->second->headerText().resize(maxWidth, ' ');
				// Second, search if the standard limiter and delimiter are being used.
				std::string delimitStr(LOG2HDRDELIMRIGHT);
				std::string::size_type delimitLoc = it->second->headerText().rfind(delimitStr);
				// If found, move delimiter to the end
				if (delimitLoc != std::string::npos)
					it->second->headerText().erase(delimitLoc, delimitStr.length()).append(delimitStr);
        	}
        }
    }

    // Use reportOpenFile() to report an opened FILE pointer that needs to be closed at a safe moment.
    // The logFactory closes the FILE pointers upon self destruction,
    // In real-time apps, this means after the RTDK lib (rt_printf) has cleaned up (do this at the end of main() using rt_print_cleanup()).
    void reportOpenFile(FILE* file, CLogStream& wrnReportLog)
    {
    	if (file != NULL)
    	{
			mOpenFiles.push_back(file);
			if (mOpenFiles.size() >= LOG2OPENFILESSOFTLIMIT)
				wrnReportLog << "[WARNING] Open file limit of log factory exceeded: memory allocation may occur. Try and increase LOG2OPENFILESSOFTLIMIT." << std::endl;
    	}
    }
    void closeOpenFiles()
    {
    	while (!mOpenFiles.empty())
    	{
    		fclose(mOpenFiles.back());
    		mOpenFiles.pop_back();
    	}
    }

  public:
	  CLog2Factory() : mLevel(llInfo), mTimeStamping(false)
	  {
		  mOpenFiles.reserve(LOG2OPENFILESSOFTLIMIT);
	  }
	  virtual ~CLog2Factory()
	  {
		  // Close all reported open files upon destruction
		  closeOpenFiles();
		  // Cleanup the logs
	       for (std::map<const std::string, CLogStream*>::iterator it = mLogs.begin();
	             it != mLogs.end(); ++it)
	    	   delete it->second;
	  }

    //virtual void setLevel(ELogLevel level) = 0;
    virtual void setLevel(ELogLevel level)
    {
      mLevel = level;

      for (std::map<const std::string, CLogStream*>::iterator it = mLogs.begin();
           it != mLogs.end(); ++it)
        it->second->setLevel(mLevel);
    }
    virtual CLogStream &getLog(const std::string &name) = 0;

    virtual void enableConsoleOutput(bool bEnabled)
    {
      for (std::map<const std::string, CLogStream*>::iterator it = mLogs.begin();
           it != mLogs.end(); ++it)
        it->second->enableConsoleOutput(bEnabled);
    }
    virtual void enableFileOutput(bool bEnabled, const std::string& filename="")
    {
    	for (std::map<const std::string, CLogStream*>::iterator it = mLogs.begin();
            it != mLogs.end(); ++it)
         reportOpenFile(it->second->enableFileOutput(bEnabled, filename), *(it->second));
    }
    virtual void enableTimeStamping(bool bEnabled)
    {
    	mTimeStamping = bEnabled;

    	for (std::map<const std::string, CLogStream*>::iterator it = mLogs.begin();
            it != mLogs.end(); ++it)
         it->second->enableTimeStamping(bEnabled);
    }
    virtual void flushFileOutput()
    {
    	for (std::map<const std::string, CLogStream*>::iterator it = mLogs.begin();
            it != mLogs.end(); ++it)
    	it->second->flushFileOutput();
    }
    virtual void redirectConsoleOutput(FILE* file)
    {
    	for (std::map<const std::string, CLogStream*>::iterator it = mLogs.begin();
            it != mLogs.end(); ++it)
    	it->second->redirectConsoleOutput(file);
    }
    virtual ELogLevel getLevelFromString(const std::string &level)
    {
      if (!level.compare(0, 3, "cra")) return llCrawl;
      if (!level.compare(0, 1, "d"))   return llDebug;
      if (!level.compare(0, 1, "i"))   return llInfo;
      if (!level.compare(0, 1, "n"))   return llNotice;
      if (!level.compare(0, 1, "w"))   return llWarning;
      if (!level.compare(0, 1, "e"))   return llError;
      if (!level.compare(0, 3, "cri")) return llCritical;
      return llInfo;
    }
};

// To prevent a static initialization disaster when using global CLog2 objects,
// we define gLogFactory as gLogFactory() returning a reference.
CLog2Factory&	gLogFactory();

class CLog2
{
  private:
    CLogStream &mStream;
    std::string	mName;

  public:
    CLog2(const std::string &name) :
    	mStream(gLogFactory().getLog(name)),
    	mName(name)
    	{ }
    CLog2& operator=(const CLog2 &obj) { return *this; }

    CLogStream &operator()(ELogLevel level)
    {
      switch (level)
      {
        case llCrawl:
          mStream.setMessageColor(FOREGROUND_GREEN);
          mStream.setSystemHeader("CRL: ");
          break;
        case llDebug:
          mStream.setMessageColor(FOREGROUND_GREEN);
          mStream.setSystemHeader("DBG: ");
          break;
        case llInfo:
          mStream.setMessageColor(FOREGROUND_GRAY);
          mStream.setSystemHeader("INF: ");
          break;
        case llNotice:
          mStream.setMessageColor(FOREGROUND_BLUE);
          mStream.setSystemHeader("NTC: ");
          break;
        case llWarning:
          mStream.setMessageColor(FOREGROUND_BROWN);
          mStream.setSystemHeader("WRN: ");
          break;
        case llError:
          mStream.setMessageColor(FOREGROUND_RED);
          mStream.setSystemHeader("ERR: ");
          break;
        case llCritical:
          mStream.setMessageColor(FOREGROUND_MAGENTA);
          mStream.setSystemHeader("CRT: ");
          break;
        case llClean:
          // Keep it clean!
          mStream.setSystemHeader("");
          break;
      }

      return mStream;
    }

    const std::string& name()				{ return mName; }

    inline ELogLevel getLevel() const       { return mStream.getLevel(); }
    void setLevel(ELogLevel level)          { mStream.setLevel(level); }
    void enableTimeStamping(bool bEnabled)	{ mStream.enableTimeStamping(bEnabled); }
    void setHeaderText(const std::string &text)    { CCriticalSection setheader(mStream); mStream.setHeaderText(text); }
    void setHeaderColor(int color)          { mStream.setHeaderColor(color); }
    void enableConsoleOutput(bool bEnabled) { mStream.enableConsoleOutput(bEnabled); }
    void enableFileOutput(bool bEnabled, const std::string& filename="") { gLogFactory().reportOpenFile(mStream.enableFileOutput(bEnabled, filename), (*this)(llWarning)); }
    void flushFileOutput()                  { mStream.flushFileOutput(); }
    CLogStream &getStream()                        { return mStream; }
};

inline bool logAssertLnInternal(CLog2& log, const char *msg, ELogLevel level, bool condition)
{
	if (!condition)
		LOG2(log, msg << std::endl, level);
	return condition;
}

#endif /* __LOG2_H_INCLUDED */
