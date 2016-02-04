/*
 * STGAgentQ.h
 *
 *  Created on: Aug 10, 2009
 *      Author: Erik Schuitema
 */

#ifndef STGAGENTQ_H_
#define STGAGENTQ_H_

//#include "agentq.h"
#include <STGPolicy.h>
#include <Log2.h>
#include <randomc.h>
#include <Statistics.h>
#include <Stopwatch.hpp>

template<class STGStateType>
class CSTGAgentQ: public CSTGPolicy<STGStateType>//, public CAgentQ #ivan
{
	protected:
		char						mIsInTerminalState;
		bool						mLearningEnabled;
		bool						mProgressReportingEnabled;
		int							mProgressReportedEpisode;	// Episode for which progress has last been reported
		std::string					mProgressFilename;
		std::string					mSaveFileName;
		CLog2						mProgressLog;
		int							mEpisodeNumber;			// The number of episodes this agent has finished since its creation
		int							mLifeNumber;			// Counter that counts the number of memory resets. After a memreset, the agent is 'reborn'
		STGStateType				mPreviousSTGState;
		STGStateType				*mCurrentSTGState;
		int							mNumRunsBeforeTestRun;	// -1 means every run will be a test run with learning *enabled*. Otherwise, learning is disabled during a test run.
		CAgentState					mPreviousState;
		CAgentAction				mPreviousAction;

		CSimpleStat					mStatPolicyStep;
		CSimpleStat					mStatLearnStep;
		Stopwatch					mStopwatch;

		uint64_t					mLastAgentActiveTime;	// Last registered time that the agent was active
		uint64_t					mLifeDuration;			// Time in microseconds that the agent was in control since last memory reset

		bool						mActivateActionsBeforeLearning;	// If desired, executePolicy() can call activateActions() directly after consulting the policy, and do the learning step afterwards

		void						openProgressFile()
		{
			std::stringstream ss;
			ss << mProgressFilename << "_" << std::setfill('0') << std::setw(3) << mLifeNumber << ".txt";
			mProgressLog.enableFileOutput(true, ss.str());
			mProgressLog.enableConsoleOutput(false);
			mProgressLog.setHeaderText("");
		}

	public:
		CSTGAgentQ(ISTGActuation *actuationInterface, CQSpace* qspace=NULL, CPolicySpace* policySpace=NULL):
			CSTGPolicy<STGStateType>(actuationInterface),
//			CAgentQ(qspace, policySpace),
			mIsInTerminalState(0),
			mLearningEnabled(true),
			mProgressReportingEnabled(false),
			mProgressReportedEpisode(-1),
			mProgressFilename("templearnprogress"),
			mSaveFileName("templearndata"),
			mProgressLog("stgagqprg"),
			mEpisodeNumber(0),
			mLifeNumber(0),
			mNumRunsBeforeTestRun(-1),
			mStatPolicyStep(7500),
			mStatLearnStep(7500),
			mLastAgentActiveTime(0),
			mLifeDuration(0),
			mActivateActionsBeforeLearning(false)
		{
			gRanrotB.RandomInit(mLifeNumber);
		}

		// Always provide access to current and previous system state. Useful for determining rewards, terminal states, etc.
		STGStateType*	getCurrentSTGState()			{return mCurrentSTGState;}
		STGStateType*	getPreviousSTGState()			{return &mPreviousSTGState;}
		//virtual ~CSTGAgentQPolicy()	{}

		void			enableLearning(bool enabled)	{mLearningEnabled = enabled;}

		// If desired, executePolicy() can call activate actions directly after consulting the policy,
		// and do the learn step afterwards.
		// WARNING: With this option enabled, make sure your STG is safe against calling activateActions() multiple times!!!
		void			enableActivateActionsBeforeLearning(bool enabled)	{mActivateActionsBeforeLearning = enabled;}

		// Returns the duration in microseconds that the agent was actively in control since its last memory reset
		uint64_t		getAgentLifeDuration()	{return mLifeDuration;}

		void			enableProgressReporting(bool enabled)
		{
			mProgressReportingEnabled = enabled;

			// Open progress file if logging is enabled
			if (enabled)
				openProgressFile();
		}

		void			setLifeNumber(int lifeNumber)
		{
			mLifeNumber = lifeNumber;
			gRanrotB.RandomInit(mLifeNumber);

			if (mProgressReportingEnabled)
				openProgressFile();
		}

		// getProgressReport() is called inside reset() (just before actually resetting) and should return the policy's progress.
		// It's your own responsibility to end the progress report with endl ('\n') if desired.
		virtual std::string getProgressReport(uint64_t absoluteTimeMicroseconds)	{return "";}

		// updateState(..) fills mAgentState with sensible data (must be implemented)
		virtual void	updateState(STGStateType* currentSTGState) = 0;
		// updateAction(..) converts mAgentAction to actuationInterface commands (must be implemented)
		virtual void	updateAction(ISTGActuation* actuationInterface) = 0;

		virtual bool	isDone(STGStateType* currentState, uint64_t absoluteTimeMicroseconds)
		{
			// Usually, isDone() is called before executePolicy().
			// Therefore, we cannot re-evaluate mAgentQ->TerminalState() here, because
			// if the state is terminal, we would miss the final and most important learning update
			// inside executePolicy(). So mIsInTerminalState is determined in executePolicy() and
			// checked here.
			return mIsInTerminalState != CONST_STATE_NORMAL;	// if state is terminal then done
		}

		virtual bool readConfig(const CConfigSection &configSection)
		{
      bool configresult = true;//CAgentQ::readConfig(configSection);
			// Read life number (optional for the user)
			configSection.get("initialLifeNumber", &mLifeNumber);
			mLogInfoLn("Starting with life number " << mLifeNumber);
			// Use the life number directly to seed the random generator
			gRanrotB.RandomInit(mLifeNumber);
			configSection.get("progressFilename",	&mProgressFilename);
			configSection.get("savedatafilename", &mSaveFileName);
			configSection.get("numRunsBeforeTestrun", &mNumRunsBeforeTestRun);
			configresult &= mLogAssert(configSection.get("activateActionsBeforeLearning", &mActivateActionsBeforeLearning));
			return configresult;
		}

		virtual void	reset(uint64_t absoluteTimeMicroseconds)
		{
			mLastAgentActiveTime = absoluteTimeMicroseconds;
			enableLearning(true);

			CSTGPolicy<STGStateType>::reset(absoluteTimeMicroseconds);
			mIsInTerminalState	= CONST_STATE_NORMAL;
			resetTrial();
			mPreviousSTGState.invalidate();	// Store that this state is not valid yet
			//mAgentQ->UpdateState();
			// Choose first action of the run
			//mAgentQ->GetBestAction(mAction);
			//mIsInTerminalState = TerminalState();
			mEpisodeNumber++;
			// See if the next run should be a test run with learning disabled. Note that mEpisodeNumber++ happens first!
			if ((mNumRunsBeforeTestRun > 0) && ((mEpisodeNumber % mNumRunsBeforeTestRun) == 0))
			{
				mLogInfoLn("*********** Starting TEST RUN (learning disabled) ***********");
				enableLearning(false);
			}
			else
				mLogInfoLn("Starting episode " << mEpisodeNumber);

			// Report timing
			mLogNoticeLn("Learn step: " << mStatLearnStep.toStr("us"));
			mLogNoticeLn("Policy step: " << mStatPolicyStep.toStr("us"));
		}

		virtual void	resetMemory(uint64_t absoluteTimeMicroseconds)
		{
			mLogNoticeLn("Resetting memory, starting a new agent life");

			// Reset episode counter
			mEpisodeNumber = 0;

			// Reset time counter
			mLifeDuration = 0;

			// Increase life number, reseed, and open new progress file
			setLifeNumber(mLifeNumber+1);

			// Reset the memory
			CSTGPolicy<STGStateType>::resetMemory(absoluteTimeMicroseconds);

			// Reset agent's life
			resetLife();

			// Always enable learning
			enableLearning(true);
		}

		virtual bool	init()
		{
			bool result = true;
			mEpisodeNumber = 0;

			result &= CSTGPolicy<STGStateType>::init();
//			result &= CAgentQ::init();
			return result;
		}

		virtual bool	init(const std::string &fileName)
		{
			bool result = true;
			mEpisodeNumber = 0;

			result &= CSTGPolicy<STGStateType>::init();
//			result &= CAgentQ::init(fileName);
			return result;
		}

		virtual bool	init(std::ifstream &fileInStream)
		{
			bool result = true;
			mEpisodeNumber	= 0;

			result &= CSTGPolicy<STGStateType>::init();	// Nothing to load from CSTGPolicy
//			result &= CAgentQ::init(fileInStream);
			return result;
		}

		virtual void	deinit()
		{
//			CAgentQ::deinit();
			mProgressLog.enableFileOutput(false);
		}

		// The two save functions inherited from CSTGPolicy
		virtual bool	save()
		{
			// Save to filename + lifenumber + .dat
			std::stringstream ss;
			ss << mSaveFileName << "_" << std::setfill('0') << std::setw(3) << mLifeNumber << ".dat";
			return save(ss.str());
		}

		virtual bool	save(std::ofstream &fileOutStream)
		{
//			return CAgentQ::save(fileOutStream);
		}
		// An extra save function for convenience
		virtual bool	save(const std::string &fileName)
		{
			std::ofstream fileStream(fileName.c_str(), std::ios::out | std::ios::binary);
			if (!fileStream.good())
				return false;

			bool result = save(fileStream);
			fileStream.close();
			return result;
		}

		virtual int	executePolicy(STGStateType* currentState, uint64_t absoluteTimeMicroseconds)
		{
			int result = 0;
			// Time registration
			mLifeDuration += absoluteTimeMicroseconds - mLastAgentActiveTime;
			mLastAgentActiveTime = absoluteTimeMicroseconds;
			// Update current STG state pointer
			mCurrentSTGState = currentState;

			// Adjust logger header with current time
			char headerText[200];
			sprintf(headerText, "[AgentQ|Sim:%.0fs,Trial:%.4fs] ", (double)absoluteTimeMicroseconds/1E6, (double)(absoluteTimeMicroseconds - this->getResetTime())/1E6);

			//ss << "[AgentQ|Sim:" <<  << "s,Trial:" <<  << "s] ";
			mLog.setHeaderText(headerText);

			// Update internal agent state
			updateState(currentState);

			// calculate whether we landed in a terminal state or not
			mIsInTerminalState = isTerminalState(absoluteTimeMicroseconds - this->getResetTime());

			if (mLearningEnabled)
			{
				mStopwatch.reset();
				mStopwatch.start();
				// Consult greedy and exploration policy
				if (!consultPolicy())
					result = -1;	// Memory error
				mStatPolicyStep.addValue(mStopwatch.stop()/1e3);	// microseconds
			}
			else
			{
				// Retrieve best possible action
				//mQSpace->getBestActionQValue(mAgentState, mAgentAction);
				if (!consultGreedyPolicy())
					result = -1;	// Memory error
			}

			// Sends the agent action to the actuation interface
			updateAction(this->getActuationInterface());

			if (mActivateActionsBeforeLearning)
				// Activate actions before learning
				this->getActuationInterface()->activateActions(currentState->mStateID);

			// Backup current agent state/action to previous state/action
			mPreviousAction		= mAgentAction;

			// Learn if desired
			if (mLearningEnabled)
			{
				mStopwatch.reset();
				mStopwatch.start();
				learn(mIsInTerminalState);
				mStatLearnStep.addValue(mStopwatch.stop()/1e3);	// microseconds
			}
			else
				updateTotalReward();	// Keep track of the rewards the agent earned in a test run

			// Backup current agent state/action to previous state/action
			mPreviousState		= mAgentState;

			// We can only assume that the currentState pointer is valid within this function.
			// To make sure nobody uses the current state outside this function, set its pointer to NULL
			mCurrentSTGState = NULL;
			// Backup currentState to mPreviousSTGState
			mPreviousSTGState	= *currentState;

			// Report progress if desired
			if (mIsInTerminalState != CONST_STATE_NORMAL)	// Episode ended
			{
				if ((mEpisodeNumber % mNumRunsBeforeTestRun) == 0)	// This episode was a test run
				{
					if (mProgressReportingEnabled && (mEpisodeNumber != mProgressReportedEpisode))	// We want reporting and we did not do it yet for this episode
					{
						std::string progressString(getProgressReport(absoluteTimeMicroseconds));
						if (!progressString.empty())
						{
							mProgressLog.setHeaderText("");
							mProgressLog(llClean) << progressString << flush;
							mProgressLog.flushFileOutput();	// We don't want to wait for 4kb to be filled before the file is flushed
							mProgressReportedEpisode = mEpisodeNumber;
						}
					}
				}
			}
			return result;
		}
};

#endif /* STGAGENTQ_H_ */
