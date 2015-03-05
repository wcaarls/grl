/*
 * Non-realtime implementation of the log factory.
 * Wouter Caarls <w.caarls@tudelft.nl>
 */

#ifdef MSC_VER
#include <win32_compat.h>
#endif

#include <pthread.h>
#include <StdLog.h>
#include <Log2.h>

class CStdLog2Factory : public CLog2Factory
{
	protected:
		pthread_mutex_t								mMutexLogLock;

	public:
		CStdLog2Factory()	{ pthread_mutex_init(&mMutexLogLock, NULL); }
		~CStdLog2Factory()	{ pthread_mutex_destroy(&mMutexLogLock); }
		
		virtual CLogStream &getLog(const std::string &name)
		{
			if (mLogs.find(name) == mLogs.end())
			{
				CStdLogStream *log = new CStdLogStream(&mMutexLogLock);
				log->setHeaderText(LOG2HDRDELIMLEFT + name + LOG2HDRDELIMRIGHT);
				log->setHeaderColor(CONSL_INTENSITY);
				log->setLevel(mLevel);
				log->enableTimeStamping(mTimeStamping);
				mLogs[name] = log;

				equalizeHeaderTexts();

				return *log;
			}
			else
				return *mLogs[name];
		}
};

// We use lazy initialization for the log factory
// Since static local objects are constructed the first time control flows over their declaration (only),
// the new logfactory will only be created once: the first time gLogFactory() is called.
CLog2Factory& gLogFactory()
{
	// WARNING: to avoid a static *de*initialization disaster, one should not use gLogFactory() in the destructor of static objects. However, this is unlikely. If so, initialize a static pointer with 'new' and return *logfact.
	static CStdLog2Factory logfact;
	return logfact;
}
