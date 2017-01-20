/*
 * STGAgentQLeo.h
 *
 *  Created on: Sep 1, 2009
 *      Author: Erik Schuitema
 */

#ifndef STGAGENTQLEO_H_
#define STGAGENTQLEO_H_

#include "AgentInterface.h"
#include "STGLeo.h"
#include <Configuration.h>
#include <grl/configurable.h>

template<class STGStateType>
class CSTGAgentQ : public grl::Configurable
{
public:
  CSTGAgentQ(ISTGActuation *actuationInterface):
    mActuationInterface(actuationInterface),
    mLog("CSTGAgentQ")
  {}
  STGStateType  *getCurrentSTGState()			{return mCurrentSTGState;}
  STGStateType  *getPreviousSTGState()		{return &mPreviousSTGState;}
  double        getTotalReward()          {return 0;}
  ISTGActuation *getActuationInterface()	{return mActuationInterface;}

protected:
  STGStateType	mPreviousSTGState;
  STGStateType	*mCurrentSTGState;
  CAgentAction	mPreviousAction;
  ISTGActuation *mActuationInterface;
  CAgentAction  mAgentAction;
  CLog2					mLog;

};

typedef CSTGAgentQ<CLeoState> CSTGAgentQLeo;

#endif /* STGAGENTQLEO_H_ */
