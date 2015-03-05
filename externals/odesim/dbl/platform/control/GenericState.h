/*
 *    Generic CSTGState class and associated functions
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
 */

#ifndef __GENERICSTATE_H_INCLUDED
#define __GENERICSTATE_H_INCLUDED

#include <muParser.h>
#include <Configuration.h>
#include <STG.h>
#include <Log2.h>

#define MAX_STATEVARS 128
#define MAX_ACTIONVARS 64

class ISTGGenericSensing
{
  public:
    virtual ~ISTGGenericSensing()	{}
    virtual int getStateIndex(const std::string &name) = 0;
};

class ISTGGenericActuation : public ISTGActuation
{
  public:
    virtual int getActionIndex(const std::string &name) = 0;
};

class GenericState: public CSTGState
{
  public:
    double var[MAX_STATEVARS];
    double prevAction[MAX_ACTIONVARS];

    GenericState()
    {
      memset((void*)var, 0, MAX_STATEVARS*sizeof(double));
      memset((void*)prevAction, 0, MAX_ACTIONVARS*sizeof(double));
    }
};

class CGenericSensorValue : public CSTGLoggable
{
  private:
    std::string mName;
    int mIndex;
    double *mValue;

  public:
    CGenericSensorValue(const std::string &name="unnamed", ISTGGenericSensing *iface=NULL) : mName(name), mIndex(-1)
    {
      mValue = new double(0);
      if (iface)
        resolve(iface);
    }
    ~CGenericSensorValue()
    {
      // Don't delete mValue to avoid tedious bookkeeping when copying. Take the memory leak instead.
    }

    bool resolve(ISTGGenericSensing *iface)
    {
      mIndex = iface->getStateIndex(mName);
      logDebugLn(mStgLog, "SensorValue " << mName << " resolved to " << mIndex);
      return (mIndex >= 0);
    }

    std::string getName() const
    {
      return mName;
    }

    double getValue(GenericState *state)
    {
      if (mIndex >= 0)
      {
        (*mValue) = state->var[mIndex];
        logCrawlLn(mStgLog, "SensorValue " << mName << " (" << mIndex << ") has value " << (*mValue));
      }
      else
      {
        logWarningLn(mStgLog, "Trying to get unresolved sensor value " << mName);
        (*mValue) = 0;
      }

      return (*mValue);
    }

    void RegisterParserVariable(mu::Parser &parser)
    {
      parser.DefineVar(mName, mValue);
    }
};

class CGenericStateVar : public CSTGLoggable
{
  private:
    std::string mExpression;
    std::vector<CGenericSensorValue> mSensorValues;
    mu::Parser mParser;

  public:
    CGenericStateVar(std::string expression="0") : mExpression(expression)
    {
      mParser.SetExpr(mExpression);
    }

    bool readConfig(const CConfigSection &configNode);
    bool resolve(ISTGGenericSensing *iface)
    {
      bool result = true;
      for (unsigned int ii=0; ii != mSensorValues.size(); ++ii)
        result &= mSensorValues[ii].resolve(iface);
      return result;
    }

    double evaluate(GenericState *state);
};

class CGenericActuator : public CSTGLoggable
{
  private:
    std::string mName, mType;
    ESTGActuationMode mActuationMode;
    int mIndex;

  public:
    CGenericActuator(std::string name, ISTGGenericActuation *iface=NULL) : mName(name), mIndex(-1)
    {
      if (iface)
        resolve(iface);
    }

    bool resolve(ISTGGenericActuation *iface)
    {
      // Get variable type
      size_t dot = mName.rfind('.');
      if (dot != mName.npos)
        mType = mName.substr(dot+1, mName.npos);
      else
      {
        logErrorLn(mStgLog, "Undefined action type in " << mName);
        return false;
      }

      mActuationMode = iface->getActuationModeByName(mType);
      mIndex = iface->getActionIndex(mName);
      logDebugLn(mStgLog, "Action " << mName << " resolved to " << mIndex);

      return mIndex >= 0;
    }

    void actuate(double value, ISTGActuation *iface);
    void setAction(double value, ISTGActuation *iface);
    double getAction(GenericState *state);
};

class CGenericActionVar : public CSTGLoggable
{
  private:
    std::vector<CGenericActuator> mActuators;
    std::string mExpression;
    mu::Parser mParser;
    double *mValue;
    ESTGActuationMode mActuationMode;

  public:
    CGenericActionVar(std::string expression="0") : mExpression(expression)
    {
      mParser.SetExpr(mExpression);
      mValue = new double(0);
    }
    ~CGenericActionVar()
    {
      // Don't delete mValue to avoid tedious bookkeeping when copying. Take the memory leak instead.
    }

    bool readConfig(const CConfigSection &configNode);
    bool resolve(ISTGGenericActuation *iface)
    {
      bool success = true;

      for (unsigned int ii=0; ii != mActuators.size(); ++ii)
        success &= mActuators[ii].resolve(iface);

      return success;
    }

    void actuate(double value, ISTGActuation *iface);
    double getAction(GenericState *state);
};

#endif /* __GENERICSTATE_H_INCLUDED */
