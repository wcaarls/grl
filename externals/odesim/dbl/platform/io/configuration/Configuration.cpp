/*
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

#include "Configuration.h"
#include <stdlib.h>

#ifdef WIN32
#include <win32_compat.h>
#endif

// ************************************************************************ //
// *************************** IConfigProperty **************************** //
// ************************************************************************ //
bool IConfigProperty::toBool() const
{
	if (strcasecmp(toString().c_str(), "true") == 0)
		return true;
	else if (strcasecmp(toString().c_str(), "yes") == 0)
		return true;
	else if (strcasecmp(toString().c_str(), "false") == 0)
		return false;
	else if (strcasecmp(toString().c_str(), "no") == 0)
		return false;
	else
		return atoi(toString().c_str())!=0;
}

long IConfigProperty::toInt() const
{
	return strtol(toString().c_str(), NULL, 10);
}

unsigned long IConfigProperty::toUInt() const
{
	return strtoul(toString().c_str(), NULL, 10);
}

double IConfigProperty::toFloat() const
{
	std::istringstream i(toString());
	double result;
	// Convert and check result. Make sure that there are no characters left over in the stream afterwards.
	if (!(i >> result) || !i.eof())
		logErrorLn(CLog2("config"), "Could not convert \"" << toString() << "\" to a floating point value");
	else
		logCrawlLn(CLog2("config"), name() << "=" << toString());
	return result;
}

// ************************************************************************ //
// *************************** CConfigProperty **************************** //
// ************************************************************************ //

CConfigProperty::CConfigProperty(IConfigProperty* configPropertyInterface)
{
	mIConfigProperty = configPropertyInterface;
}

CConfigProperty::~CConfigProperty()
{
}

std::string CConfigProperty::name() const
{
	if (mIConfigProperty)
		return mIConfigProperty->name();
	else
		return "";
}


// ************************************************************************ //
// **************************** CConfigSection **************************** //
// ************************************************************************ //

CConfigSection::CConfigSection(IConfigSection* configSectionInterface)
{
	mIConfigSection = configSectionInterface;
}

CConfigSection::~CConfigSection()
{
}

std::string CConfigSection::name() const
{
	if (mIConfigSection)
		return mIConfigSection->name();
	else
		return "";
}

bool CConfigSection::hasSection(const std::string& section) const
{
	if (mIConfigSection)
		return mIConfigSection->hasSection(section);
	else
		return false;
}

CConfigSection CConfigSection::parent() const
{
	IConfigSection *newSection = NULL;
	if (mIConfigSection)
		newSection = mIConfigSection->parent();
	return CConfigSection(newSection);
}

CConfigSection CConfigSection::section(const std::string& section) const
{
	IConfigSection *newSection = NULL;
	if (mIConfigSection)
		newSection = mIConfigSection->section(section);
	return CConfigSection(newSection);
}

CConfigSection CConfigSection::firstSection() const
{
	IConfigSection *newSection = NULL;
	if (mIConfigSection)
		newSection = mIConfigSection->firstSection();
	return CConfigSection(newSection);
}

CConfigSection CConfigSection::nextSection() const
{
	IConfigSection *newSection = NULL;
	if (mIConfigSection)
		newSection = mIConfigSection->nextSection();
	return CConfigSection(newSection);
}

CConfigSection CConfigSection::nextSimilarSection() const
{
	IConfigSection *newSection = NULL;
	if (mIConfigSection)
		newSection = mIConfigSection->nextSimilarSection();
	return CConfigSection(newSection);
}

bool CConfigSection::has(const std::string& property) const
{
	if (mIConfigSection)
		return mIConfigSection->has(property);
	else
		return false;
}

/**
 * Gets the value of the given property
 *
 * @param property  The property
 * @param value     Pointer to the value field
 * @return true if property was set from configuration
 */

CConfigProperty CConfigSection::get(const std::string& property) const
{
	IConfigProperty *newProperty = NULL;
	if (mIConfigSection)
		newProperty = mIConfigSection->get(property);
	return CConfigProperty(newProperty);
}

CConfigProperty CConfigSection::firstProperty() const
{
	IConfigProperty *newProperty = NULL;
	if (mIConfigSection)
		newProperty = mIConfigSection->firstProperty();
	return CConfigProperty(newProperty);
}

#define CCONFIGSECTION_GET_MACRO(TYPE, ASSIGNMENT_STATEMENT) 				\
bool CConfigSection::get(const std::string& property, TYPE *value) const	\
{																			\
	if (mIConfigSection)													\
	{																		\
		IConfigProperty* iConfigProp = mIConfigSection->get(property);		\
		if (iConfigProp)													\
		{																	\
			ASSIGNMENT_STATEMENT;											\
			return true;													\
		}																	\
		else																\
			return false;													\
	}																		\
	else																	\
		return false;														\
}

CCONFIGSECTION_GET_MACRO(std::string	, *value = iConfigProp->toString())
CCONFIGSECTION_GET_MACRO(bool			, *value = iConfigProp->toBool())
CCONFIGSECTION_GET_MACRO(char			, *value = (char)iConfigProp->toInt())
CCONFIGSECTION_GET_MACRO(unsigned char	, *value = (unsigned char)iConfigProp->toUInt())
CCONFIGSECTION_GET_MACRO(short			, *value = (short)iConfigProp->toInt())
CCONFIGSECTION_GET_MACRO(unsigned short	, *value = (unsigned short)iConfigProp->toUInt())
CCONFIGSECTION_GET_MACRO(int			, *value = (int)iConfigProp->toInt())
CCONFIGSECTION_GET_MACRO(unsigned int	, *value = (unsigned int)iConfigProp->toUInt())
CCONFIGSECTION_GET_MACRO(long			, *value = iConfigProp->toInt())
CCONFIGSECTION_GET_MACRO(unsigned long	, *value = iConfigProp->toUInt())
CCONFIGSECTION_GET_MACRO(long long		, *value = iConfigProp->toInt())
CCONFIGSECTION_GET_MACRO(unsigned long long	, *value = iConfigProp->toUInt())
CCONFIGSECTION_GET_MACRO(float			, *value = (float)iConfigProp->toFloat())
CCONFIGSECTION_GET_MACRO(double			, *value = iConfigProp->toFloat())
// OptionVar variants
CCONFIGSECTION_GET_MACRO(COptionBool	, *value = iConfigProp->toBool())
CCONFIGSECTION_GET_MACRO(COptionInt		, *value = iConfigProp->toInt())
CCONFIGSECTION_GET_MACRO(COptionDouble	, *value = iConfigProp->toFloat())
CCONFIGSECTION_GET_MACRO(COptionChar	, *value = (char)iConfigProp->toInt())
CCONFIGSECTION_GET_MACRO(COptionByte	, *value = (unsigned char)iConfigProp->toUInt())
CCONFIGSECTION_GET_MACRO(COptionWord	, *value = (unsigned short)iConfigProp->toUInt())

#define CCONFIGSECTION_GET_PRESET_MACRO(TYPE)											\
bool CConfigSection::get(const std::string& property, TYPE *value, TYPE preset) const	\
{																						\
	if (get(property, value))															\
		return true;																	\
	else																				\
	{																					\
		*value = preset;																\
		return false;																	\
	}																					\
}

CCONFIGSECTION_GET_PRESET_MACRO(std::string)
CCONFIGSECTION_GET_PRESET_MACRO(bool)
CCONFIGSECTION_GET_PRESET_MACRO(char)
CCONFIGSECTION_GET_PRESET_MACRO(unsigned char)
CCONFIGSECTION_GET_PRESET_MACRO(short)
CCONFIGSECTION_GET_PRESET_MACRO(unsigned short)
CCONFIGSECTION_GET_PRESET_MACRO(int)
CCONFIGSECTION_GET_PRESET_MACRO(unsigned int)
CCONFIGSECTION_GET_PRESET_MACRO(long)
CCONFIGSECTION_GET_PRESET_MACRO(unsigned long)
CCONFIGSECTION_GET_PRESET_MACRO(long long)
CCONFIGSECTION_GET_PRESET_MACRO(unsigned long long)
CCONFIGSECTION_GET_PRESET_MACRO(double)
CCONFIGSECTION_GET_PRESET_MACRO(float)

bool CConfigSection::isNull() const
{
	return (mIConfigSection == NULL);
}

bool CConfigSection::getArray(const std::string& property, CConfigPropertyArray* array) const
{
	if (mIConfigSection)
	{
		IConfigProperty* iConfigProp = mIConfigSection->get(property);
		if (iConfigProp)
		{
			array->setData(iConfigProp->toString());
			return true;
		}
		else
			return false;
	}
	else
		return false;
}

bool CConfigSection::getArray(const std::string& property, double* array, unsigned int maxNumElements) const
{
	CConfigPropertyArray propArray;
	if (getArray(property, &propArray))
	{
		for (unsigned int i=0; i<propArray.size() && i<maxNumElements; i++)
			array[i] = propArray[i].toFloat();
		return true;
	}
	else
		return false;
}


// ************************************************************************ //
// **************************** CConfigProperty *************************** //
// ************************************************************************ //

std::string CConfigProperty::value() const
{
	if (mIConfigProperty)
		return mIConfigProperty->toString();
	else
		return "";
}

CConfigProperty	CConfigProperty::nextProperty() const
{
	IConfigProperty *newProperty = NULL;
	if (mIConfigProperty)
		newProperty = mIConfigProperty->nextProperty();
	return CConfigProperty(newProperty);
}

bool CConfigProperty::isNull() const
{
	return (mIConfigProperty == NULL);
}

void CConfigProperty::set(const std::string& value)
{
	if (mIConfigProperty)
		mIConfigProperty->set(value);
}

// ************************************************************************ //
// ************************* CConfigPropertyArray ************************* //
// ************************************************************************ //

void CConfigPropertyArray::setData(const std::string& data, const char delimiter)
{
	clear();
	std::istringstream ss(data);
	std::string dataFieldStr;

	while (std::getline(ss, dataFieldStr, delimiter))
		push_back(IConfigPropertyString(dataFieldStr));
}

CConfigProperty CConfigPropertyArray::operator[](size_type __n)
{
	return CConfigProperty(&at(__n));
}

// ************************************************************************ //
// **************************** CConfiguration **************************** //
// ************************************************************************ //

/*
int CConfiguration::resolveSectionConstants(const CConfigSection& section, CConfigConstants *constants)
{
	int numResolvedConstants = 0;

	if (section.name() == std::string("constants"))
	{
		//printf("(resolveSectionConstants) skipping constants section\n");
		return 0;
	}

	//printf("(resolveSectionConstants) processing section \"%s\"\n", section.name().c_str());
	// process its properties
	for (CConfigProperty iProp = section.firstProperty(); !iProp.isNull(); iProp = iProp.nextProperty())
	{
		//printf("processing property %s\n", iProp.name().c_str());
		for (unsigned int iConst=0; iConst<constants->size(); iConst++)
		{
			if (iProp.value() == constants->at(iConst).mKey)
			{
				//printf("\treplacing value \"%s\" with \"%s\"\n", iProp.value().c_str(), constants->at(iConst).mValue.c_str());
				iProp.set(constants->at(iConst).mValue);
				numResolvedConstants++;
			}
		}
	}

	// process its sections
	for (CConfigSection iSection = section.firstSection(); !iSection.isNull(); iSection = iSection.nextSection())
	{
		numResolvedConstants += resolveSectionConstants(iSection, constants);
	}

	return numResolvedConstants;
}

int CConfiguration::resolveConstants()
{
	CConfigConstants constants;
	CConfigSection constantsSection = root().section("constants");
	if (!constantsSection.isNull())
	{
		for (CConfigProperty iProp = constantsSection.firstProperty(); !iProp.isNull(); iProp = iProp.nextProperty())
		{
			CConfigConstant newConstant("@" + iProp.name(), iProp.value());
			//printf("Constant found: key = \"%s\", value = \"%s\"\n", iProp.name().c_str(), iProp.value().c_str());
			constants.push_back(newConstant);
		}
		//printf("now processing sections\n");
		return resolveSectionConstants(root(), &constants);
	}
	else
	{
		//printf("no constants section found\n");
		return 0;
	}
}
*/

std::string CConfiguration::replaceConstants(const std::string& expr, mu::Parser *parser)
{
	char buf[256];
	std::string result(expr);
	mu::valmap_type cmap = parser->GetConst();
	if (cmap.size())
	{
		for (mu::valmap_type::const_iterator item = cmap.begin(); item!=cmap.end(); ++item)
		{
			int pos=-1;
			while ((pos = result.find(item->first, pos+1)) != (int)result.npos)
			{
				if ((pos == 0 || !isalpha(result[pos-1])) &&
				    (pos+item->first.length() == result.length() || !isalnum(result[pos+item->first.length()])))
				{
					if (snprintf(buf, 255, "%.20g", item->second) >= 255) buf[255] = '\0';
					result.replace(pos, item->first.length(), buf);
				}
			}
		}
	}
	return result;
}

std::string CConfiguration::replaceStringConstants(const std::string& expr)
{
	std::string result(expr);
	if (mStringConstants.size())
	{
		for (std::map<std::string, std::string>::const_iterator item = mStringConstants.begin(); item!=mStringConstants.end(); ++item)
		{
			int pos=-1;
			while ((pos = result.find(item->first, pos+1)) != (int)result.npos)
			{
				if ((pos == 0 || !isalpha(result[pos-1])) &&
				    (pos+item->first.length() == result.length() || !isalnum(result[pos+item->first.length()])))
				{
					result.replace(pos, item->first.length(), item->second);
				}
			}
		}
	}
	return result;
}

int CConfiguration::resolveExpressionsInSection(const CConfigSection& section, mu::Parser *parser)
{
	int numResolvedExpressions = 0;

	if (shouldParseNode(section.name()))
	{
		//printf("(resolveSectionConstants) processing section \"%s\"\n", section.name().c_str());
		// process its properties
		for (CConfigProperty iProp = section.firstProperty(); !iProp.isNull(); iProp = iProp.nextProperty())
		{
			//printf("processing property %s\n", iProp.name().c_str());
			// Make sure we should not skip this property: check name and value for exclusion
			if (shouldParseNode(iProp.name()) && !isRegisteredString(iProp.value()))
			{
				std::string expression = iProp.value();
				char buf[256];

				// Try to evaluate the expression as a floating point value
				parser->SetExpr(expression);
				try
				{
					double resolvedValue = parser->Eval();
					if (snprintf(buf, 255, "%.20g", resolvedValue) >= 255) buf[255] = '\0';
					expression = std::string(buf);
				}
				catch (...)
				{
					// Unable to resolve expression, replace constants instead
					expression = replaceConstants(expression, parser);

					// Try to define all string constants as variables and then see if there are unknowns left
					//TODO: It might be slow to define and remove variables repeatedly at this place in the code
					double dummy=0;
					for (std::map<std::string, std::string>::const_iterator item = mStringConstants.begin(); item!=mStringConstants.end(); ++item)
						parser->DefineVar(item->first, &dummy);
					parser->SetExpr(expression);
					try
					{
						parser->Eval();
						// If we survived Eval(), all unknowns are defined as string constants, so we are happy
						//mLogCrawlLn("Found expression with string constants!");
					}
					catch (...)
					{
						// There are some unknowns left. Perform a final check to ignore array-type entries
						if (expression.find(CONFIGURATION_ARRAY_DELIMITER) == std::string::npos)
							mLogNoticeLn("Found expression with unknowns: \"" << expression << "\"");
					}
					// Undefine the string constants again.
					for (std::map<std::string, std::string>::const_iterator item = mStringConstants.begin(); item!=mStringConstants.end(); ++item)
						parser->RemoveVar(item->first);
					// Replace string constants in the expression
					expression = replaceStringConstants(expression);
				}

				if (expression != iProp.value())
				{
					mLogCrawlLn("Resolved expression \"" << iProp.value() << "\" to \"" << expression << "\".");
					numResolvedExpressions++;
					iProp.set(expression);
				}
				// If the property is verbose, print its value
				if (iProp.isVerbose())
					mLogNoticeLn("**VERBOSE** " << iProp.name() << " = " << expression);
			}
		}

		// process its sections
		for (CConfigSection iSection = section.firstSection(); !iSection.isNull(); iSection = iSection.nextSection())
			numResolvedExpressions += resolveExpressionsInSection(iSection, parser);
	}

	return numResolvedExpressions;
}

int CConfiguration::resolveExpressions()
{
	// Define math parser
	mu::Parser parser;

	// Define constants for parser.
	// Also parse the constant expressions, using the defined constants from preceding entries :)
	CConfigSection constantsSection = root().section(CONST_ConfConstantsSectionName);
	if (!constantsSection.isNull())
	{
		for (CConfigProperty iProp = constantsSection.firstProperty(); !iProp.isNull(); iProp = iProp.nextProperty())
		{
			// Convert the property value into a constant value using the parser itself
			parser.SetExpr(iProp.value());
			try
			{
				// Evaluate. String constants will automatically lead to the catch(&e) section.
				double resolvedValue = parser.Eval();
				// Check if constant was defined before, and whether that was with a *different* value.
				// If so, throw an error. Multiply defining a constant, such as pi, is okay.
				bool constHasConflict=false;
				mu::valmap_type cmap = parser.GetConst();
				if (cmap.size())
				{
					mu::valmap_type::const_iterator item = cmap.begin();
					for (; item!=cmap.end(); ++item)
						if (item->first == iProp.name())
							if (item->second != resolvedValue)
							{
								constHasConflict = true;
								break;
							}
				}
				// Check if this name is in use as a string constant
				if (mStringConstants.find(iProp.name()) != mStringConstants.end())
				{
					// The value cannot be the same since string constants don't evaluate, while this property did
					// Therefore, this always means there's a conflict.
					constHasConflict = true;
				}

				// If it's a new one, define it!
				if (!constHasConflict)
				{
					parser.DefineConst(iProp.name(), resolvedValue);
					mLogCrawlLn(iProp.name() << "=" << std::setprecision(16) << resolvedValue);
					// If it's verbose, print its value
					if (iProp.isVerbose())
						mLogNoticeLn("**VERBOSE** " << iProp.name() << " = " << std::setprecision(16) << resolvedValue);
				}
				else
					mLogErrorLn("Constant \"" << iProp.name() << "\" was redefined with different value " << resolvedValue << "!");
				//printf("[DEBUG] Found constant with name \"%s\" and value \"%g\"\n", iProp.name().c_str(), resolvedValue);
			}
			catch (mu::Parser::exception_type &e)
			{
				// Define it as a string constant, using the constants and string constants defined so far.
				// Check if constant was defined before
				std::map<std::string, std::string>::iterator strConst = mStringConstants.find(iProp.name());
				if ((strConst == mStringConstants.end()) && (parser.GetConst().find(iProp.name()) == parser.GetConst().end()))
				{
					mStringConstants[iProp.name()] = replaceStringConstants(replaceConstants(iProp.value(), &parser));
					// The following log message acts as a 'soft warning' (info) to the user in case
					// an expression should resolve to a float, but doesn't due to a typo.
					mLogInfoLn("Defined string constant \"" << iProp.name() << "\"");
				}
				else
					// If it is redefined with the same value, it's ok. Otherwise, throw error
					if (strConst->second != iProp.value())
						mLogErrorLn("Constant \"" << iProp.name() << "\" was redefined with different value: " << iProp.value());
			}
		}
	}
	else
	{
		//printf("no constants section found\n");
	}

	// Define registered strings that will be ignored by the math parser
	CConfigSection stringsSection = root().section(CONST_ConfStringsSectionName);
	if (!stringsSection.isNull())
	{
		for (CConfigProperty iProp = stringsSection.firstProperty(); !iProp.isNull(); iProp = iProp.nextProperty())
		{
			if ((iProp.name() == "s") || (iProp.name() == "string"))
				mRegisteredStrings.push_back(iProp.value());
			else if ((iProp.name() == "n") || (iProp.name() == "node"))
				mNodesExcludedFromParsing.push_back(iProp.value());
			else
				mLogErrorLn("Strings section contains item of unknown type <" << iProp.name() << ">. Either use <s> or <string> for strings, and <n> or <node> for trusted node names.");
		}
	}
	else
		mLogNoticeLn("You did not specify a <strings> section with validated strings occurring in your configuration file. You may encounter numerous parser warnings.");

	// We register the constants and strings sections too, because they don't have to be parsed
	mNodesExcludedFromParsing.push_back(CONST_ConfConstantsSectionName);
	mNodesExcludedFromParsing.push_back(CONST_ConfStringsSectionName);

	// Start resolving expressions by processing the root node
	return resolveExpressionsInSection(root(), &parser);
}

bool CConfiguration::isRegisteredString(const std::string& str)
{
	for (unsigned int i=0; i<mRegisteredStrings.size(); i++)
		if (mRegisteredStrings[i] == str)
			return true;

	return false;
}

bool CConfiguration::shouldParseNode(const std::string& nodeName)
{
	for (unsigned int i=0; i<mNodesExcludedFromParsing.size(); i++)
		if (mNodesExcludedFromParsing[i] == nodeName)
			return false;

	return true;
}
