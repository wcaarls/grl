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

#include <Configuration.h>
#include <GenericState.h>
#include <Log2.h>

using std::cout;
using std::endl;

static void DefineNameChars(mu::Parser &parser)
{
  parser.DefineNameChars("0123456789_.@"
                         "abcdefghijklmnopqrstuvwxyz"
                         "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
}

bool CGenericStateVar::readConfig(const CConfigSection &configNode)
{
  bool configResult = true;

  configResult &= configNode.get("expression", &mExpression);

  logInfoLn(mStgLog, "StateVar " << mExpression);

  mParser.SetExpr(mExpression);
  DefineNameChars(mParser);

  if (configResult)
  {
    // Extract required sensor values
    const char *str = mExpression.c_str();
    int ii=0, len = mExpression.length();

    while (ii != len)
    {
      // A sensor value
      // (1) starts with an alphabetic character or an underscore
      // (2) is a string of alphanumeric characters, underscores, or dots.
      // (3) has at least one dot

      // Find possible start
      for (; ii != len &&
             (!isalpha(str[ii]) && str[ii] != '_'); ++ii) ;

      if (ii != len)
      {
        int dots = 0, start = ii, end;

        // Find end
        for (; ii != len &&
               (isalnum(str[ii]) || str[ii] == '_' || str[ii] == '@' || str[ii] == '.'); ++ii)
          // Count dots
          if (str[ii] == '.')
            dots++;

        end = ii;

        if (dots >= 1)
        {
          // Proper sensor value
          CGenericSensorValue val(mExpression.substr(start, end-start));
          mSensorValues.push_back(val);
          val.RegisterParserVariable(mParser);
        }
      }
    }

    // Check whether we can parse the expression
    try {
      mParser.Eval();
    }
    catch (mu::Parser::exception_type &e)
    {
      logErrorLn(mStgLog, "Unable to evaluate state expression \"" << mExpression << "\": " << e.GetMsg());
      configResult = false;
    }
  }

  return configResult;
}

double CGenericStateVar::evaluate(GenericState *state)
{
  for (unsigned int ii=0; ii != mSensorValues.size(); ++ii)
    mSensorValues[ii].getValue(state);

  try {
    double result = mParser.Eval();

    logCrawlLn(mStgLog, "StateVar " << mExpression << " has value " << result);
    return result;
  }
  catch (mu::Parser::exception_type &e)
  {
    logErrorLn(mStgLog, "Unable to evaluate state expression \"" << mExpression << "\": " << e.GetMsg());
    return 0;
  }
}

void CGenericActuator::actuate(double value, ISTGActuation *iface)
{
  if (mIndex >= 0)
  {
    switch (mActuationMode)
    {
      case amTorque:
        logCrawlLn(mStgLog, "Actuator " << mName << " (" << mIndex << ") set " << mType << " to " << value);
        iface->setJointTorque(mIndex, value);
        break;
      default:
        logErrorLn(mStgLog, "Unimplemented actuation mode " << mActuationMode << " for ActionVar " << mName);
    }
  }
  else
    logWarningLn(mStgLog, "Trying to set unresolved actuator " << mName);
}

void CGenericActuator::setAction(double value, ISTGActuation *iface)
{
  if (mIndex >= 0)
    iface->setJointAction(mIndex, value);
  else
    logWarningLn(mStgLog, "Trying to set action for unresolved actuator " << mName);
}

double CGenericActuator::getAction(GenericState *state)
{
  if (mIndex >= 0)
    return state->prevAction[mIndex];
  else
  {
    logWarningLn(mStgLog, "Trying to get action for unresolved actuator " << mName);
    return 0;
  }
}

bool CGenericActionVar::readConfig(const CConfigSection &configNode)
{
  bool configResult = true;

  CConfigProperty prop = configNode.firstProperty();
  while (!prop.isNull())
  {
    if (prop.name() == "variable")
      mActuators.push_back(CGenericActuator(prop.value()));
    prop = prop.nextProperty();
  }

  if (mActuators.empty()) configResult = false;
  configResult &= configNode.get("expression", &mExpression);

  if (configResult)
  {
    logInfoLn(mStgLog, "ActionVar " << mExpression);

    mParser.SetExpr(mExpression);
    DefineNameChars(mParser);
    mParser.DefineVar("x", mValue);

    // Check whether we can parse the expression
    try {
      mParser.Eval();
    }
    catch (mu::Parser::exception_type &e)
    {
      logErrorLn(mStgLog, "Unable to evaluate action expression \"" << mExpression << "\": " << e.GetMsg());
      configResult = false;
    }
  }

  return configResult;
}

void CGenericActionVar::actuate(double value, ISTGActuation *iface)
{
  double result;

  *mValue = value;

  try
  {
    result = mParser.Eval();
  }
  catch (mu::Parser::exception_type &e)
  {
    logErrorLn(mStgLog, "Unable to evaluate action expression \"" << mExpression << "\": " << e.GetMsg());
    return;
  }

  for (unsigned int ii=0; ii != mActuators.size(); ++ii)
  {
    mActuators[ii].actuate(result, iface);
    mActuators[ii].setAction(value, iface);
  }
}

double CGenericActionVar::getAction(GenericState *state)
{
  if (!mActuators.empty())
    return mActuators[0].getAction(state);
  else
  {
    logWarningLn(mStgLog, "Trying to get action for unactuated action expression " << mExpression);
    return 0;
  }
}

