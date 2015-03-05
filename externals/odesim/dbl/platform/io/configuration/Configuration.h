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

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <string>
#include <vector>
#include <stdlib.h>
#include <muParser.h>
#include <OptionVars.h>
#include <Log2.h>


#define CONFIGURATION_ARRAY_DELIMITER	';'

// Forward declarations
class CConfigSection;
class IConfig;
class IConfigProperty;

// Base class for a config interface. Useful for giving out interfaces and deleting them later (hence the virtual destructor)
// Whenever your interface returns a pointer to a new interface,
// pull it through RegisterPendingInterface (which returns what you put in it)
// and it will automatically be deleted in the destructor.
class IConfig
{
	protected:
		std::vector<IConfig*>	mPendingInterfaces;
		// RegisterPendingInterface() returns its parameter configInterface after adding it to mPendingInterfaces
		IConfig*				registerPendingInterface(IConfig* configInterface)
		{
			mPendingInterfaces.push_back(configInterface);
			return configInterface;
		}

	public:
		virtual	~IConfig()
		{
			// Cleanup pending interfaces
			//printf("Deleting %d pending config interfaces.\n", mPendingInterfaces.size());
			while (mPendingInterfaces.size() > 0)
			{
				delete mPendingInterfaces.back();
				mPendingInterfaces.pop_back();
			}
		}
};

class IConfigSection: public IConfig
{
	friend class CConfigSection;
	public:
		virtual						~IConfigSection()												{}
	protected:
		// Make the interface protected so that noone will accidentally use this class
		virtual std::string			name() const													{return "";}
		virtual bool 				hasSection(const std::string& section) const					{return false;}
		// The following functions return a pointer to an IConfigSection object, or NULL if not available
		virtual IConfigSection*		parent()														{return NULL;}
		virtual IConfigSection*		section(const std::string& section)								{return NULL;}
		virtual IConfigSection*		firstSection()													{return NULL;}
		virtual IConfigSection*		nextSection()													{return NULL;}
		virtual IConfigSection*		nextSimilarSection()											{return NULL;}
		// The following functions return a pointer to an IConfigProperty object, or NULL if not available
		virtual IConfigProperty*	get(const std::string& property)								{return NULL;}
		virtual IConfigProperty*	firstProperty() 												{return NULL;}
		virtual bool				has(const std::string& property) const							{return false;}
};

class IConfigProperty: public IConfig
{
	friend class CConfigSection;
	friend class CConfigProperty;
	public:
		virtual						~IConfigProperty()				{}
	protected:
		virtual std::string			name() const					{return "";}
		// Virtual abstract toString() - must be implemented
		virtual std::string 		toString() const=0;
		// Automatic conversions based on toString()
		bool						toBool() const;
		long						toInt() const;
		unsigned long				toUInt() const;
		double						toFloat() const;
		virtual bool				isVerbose() const				{return false;}

		virtual void				set(const std::string& value)	{}

		// The following function returns a pointer to an IConfigProperty object, or NULL if not available
		virtual IConfigProperty*	nextProperty() 					{return NULL;}
};

// All the handy functions belonging to IConfigProperty, implemented for an std::string
class IConfigPropertyString: public IConfigProperty
{
	protected:
		std::string		mData;

	public:
		IConfigPropertyString(const std::string& data):
			mData(data)
		{}
		virtual std::string 		toString() const				{return mData;}
};

class CConfigProperty;

class CConfigPropertyArray: protected std::vector<IConfigPropertyString>
{
	public:
		void		setData(const std::string& data, const char delimiter=CONFIGURATION_ARRAY_DELIMITER);
		//size_type	size() const	{return std::vector<IConfigPropertyString>::size();}
		using std::vector<IConfigPropertyString>::size;
		CConfigProperty	operator[](size_type __n);
};

class CConfigProperty
{
	protected:
		IConfigProperty	*mIConfigProperty;

	public:
		CConfigProperty(IConfigProperty* configPropertyInterface);
		virtual ~CConfigProperty();

		std::string			name() const;
		std::string 		value() const;
		void				set(const std::string& value);
		CConfigProperty		nextProperty() const;
		bool				isNull() const;

		// Primitive 'casts'
		virtual bool				toBool() const					{if (!isNull()) return mIConfigProperty->toBool(); else return false;}
		virtual long				toInt() const					{if (!isNull()) return mIConfigProperty->toInt(); else return 0;}
		virtual unsigned long		toUInt() const					{if (!isNull()) return mIConfigProperty->toUInt(); else return 0;}
		virtual double				toFloat() const					{if (!isNull()) return mIConfigProperty->toFloat(); else return 0;}
		// isVerbose() gives the parser/expression-solver a hint that the user wants to see its value on the screen
		virtual bool				isVerbose() const				{if (!isNull()) return mIConfigProperty->isVerbose(); else return false;}
		// Array-type properties
		virtual void				toArray(CConfigPropertyArray* array) const					{array->setData(mIConfigProperty->toString());}

		operator bool() const
		{
			return isNull();
		}
};

class CConfigSection
{
	protected:
		IConfigSection *mIConfigSection;

	public:
		CConfigSection(IConfigSection* configSectionInterface);
		~CConfigSection();

		/**
		 * Returns the name of the section
		 *
		 * @return name
		 */
		std::string name() const;

		/**
		 * Checks if the section exists.
		 *
		 * @param section   The section
		 * @return true if the section exists
		 */
		bool hasSection(const std::string& section) const;

		/**
		 * Returns the parent section.
		 */
		CConfigSection parent() const;

		/**
		 * Gets the first given section with the specified name.
		 *
		 * @param section   The section
		 * @return true if ...
		 */
		CConfigSection section(const std::string& section) const;

		/**
		 * Gets the very first section (useful for iteration).
		 * @param foundSection returns the name of the section, if one is found
		 * @return true if there is a section within this section
		 */
		CConfigSection firstSection() const;

		/**
		 * Gets the next section, regardless of its name
		 *
		 * @param section   The section
		 * @return true if ...
		 */
		CConfigSection nextSection() const;

		/**
		 * Gets the next section in line with the same name.
		 *
		 * @param section   The section
		 * @return true if ...
		 */
		CConfigSection nextSimilarSection() const;

		/**
		 * Checks if the property exists in the section.
		 *
		 * @param section   The section
		 * @param property  The property
		 * @return true if the property exists
		 */
		bool has(const std::string& property) const;

		// Get any property
		CConfigProperty get(const std::string& property) const;
		// Get first property
		CConfigProperty	firstProperty() const;

		/**
		 * Gets the value of the given property
		 *
		 * @param property  The property
		 * @param value     Pointer to the value field
		 * @return true if property was set from configuration
		 */



		bool get(const std::string& property, std::string *value) const;
		bool get(const std::string& property, bool *value) const;
		bool get(const std::string& property, char *value) const;
		bool get(const std::string& property, unsigned char *value) const;
		bool get(const std::string& property, short *value) const;
		bool get(const std::string& property, unsigned short *value) const;
		bool get(const std::string& property, int *value) const;
		bool get(const std::string& property, unsigned int *value) const;
		bool get(const std::string& property, long *value) const;
		bool get(const std::string& property, unsigned long *value) const;
		bool get(const std::string& property, long long *value) const;
		bool get(const std::string& property, unsigned long long *value) const;
		bool get(const std::string& property, float *value) const;
		bool get(const std::string& property, double *value) const;

		// Option var variants
		bool get(const std::string& property, COptionBool *value) const;
		bool get(const std::string& property, COptionInt *value) const;
		bool get(const std::string& property, COptionDouble *value) const;
		bool get(const std::string& property, COptionChar *value) const;
		bool get(const std::string& property, COptionByte *value) const;
		bool get(const std::string& property, COptionWord *value) const;

		// Array variants
		bool getArray(const std::string& property, CConfigPropertyArray* array) const;
		bool getArray(const std::string& property, double* array, unsigned int maxNumElements) const;

		/**
		 * Gets the value of the given property, using a preset value that is set when the property cannot be read
		 *
		 * @param property  The property
		 * @param value     Pointer to the value field
		 * @param preset    The default value
		 * @return true if property was set from configuration
		 */
		bool get(const std::string& property, std::string *value, std::string preset) const;
		bool get(const std::string& property, bool *value, bool preset) const;
		bool get(const std::string& property, char *value, char preset) const;
		bool get(const std::string& property, unsigned char *value, unsigned char preset) const;
		bool get(const std::string& property, short *value, short preset) const;
		bool get(const std::string& property, unsigned short *value, unsigned short preset) const;
		bool get(const std::string& property, int *value, int preset) const;
		bool get(const std::string& property, unsigned int *value, unsigned int preset) const;
		bool get(const std::string& property, long *value, long preset) const;
		bool get(const std::string& property, unsigned long *value, unsigned long preset) const;
		bool get(const std::string& property, long long *value, long long preset) const;
		bool get(const std::string& property, unsigned long long *value, unsigned long long preset) const;
		bool get(const std::string& property, float *value, float preset) const;
		bool get(const std::string& property, double *value, double preset) const;

		/**
		 * Sets the value of the property in the given section.
		 *
		 * @param section   The section
		 * @param property  The property
		 * @param value     The value
		 */
		//void set(const std::string& property, const std::string& value);

		bool isNull() const;

		operator bool() const
		{
			return isNull();
		}
};


class CConfigConstant
{
	public:
		std::string	mKey;	// Identifier of the constant, always starts with an @, for example @maxhipangle
		std::string	mValue;	// Value to replace the identifier with

		CConfigConstant()	{}
		CConfigConstant(const std::string& key, const std::string& value): mKey(key), mValue(value)	{}

};

typedef std::vector<CConfigConstant> CConfigConstants;


/**
 * The configuration class reads the values from the specified configuration file.
 *
 *
 */
#define CONST_ConfRootSectionName		"configuration"
#define CONST_ConfConstantsSectionName	"constants"
#define CONST_ConfStringsSectionName	"strings"

class CConfiguration
{
	protected:
		CLog2						mLog;
		/*
		 * Replaces all registered constants in an expression
		 */
		std::string	replaceConstants(const std::string& expr, mu::Parser *parser);

		/*
		 * Resolves expressions in a single section
		 */
		int resolveExpressionsInSection(const CConfigSection& section, mu::Parser *parser);

		/*
		 * Registered strings will be ignored during expression resolving (this reduces the number of parser warnings).
		 */
		std::vector<std::string>	mRegisteredStrings;
		std::vector<std::string>	mNodesExcludedFromParsing;	// Nodes with this name will not be parsed by the math parser
		bool	isRegisteredString(const std::string& str);
		bool	shouldParseNode(const std::string& nodeName);

		/*
		 * String constants.
		 */
		std::map<std::string, std::string>	mStringConstants;

		/*
		 * Replaces all registered string constants in an expression
		 */
		std::string replaceStringConstants(const std::string& expr);

	public:
		/**
		 * Empty constructor.
		 */
		CConfiguration(): mLog("config")	{}

		/**
		 * Destructor.
		 */
		virtual ~CConfiguration()	{}

		/**
		 * Sets the file to use, calls load().
		 *
		 * @param filename The configuration file to read.
		 */
		virtual bool loadFile(const std::string& filename)=0;

		/**
		 * Saves the configuration.
		 * @param filename The filename to write the configuration to. If omitted, the filename from loadFile is used. Otherwise FALSE is returned
		 */
		virtual bool saveFile(const std::string& filename = "")=0;

		/**
		 * Reloads the configuration from file.
		 */
		virtual bool reload()=0;

		/**
		 * Returns the error string if something went wrong (read: if a function returned false)
		 */
		virtual std::string errorStr()=0;

		/**
		 * Clears the configuration.
		 */
		virtual void clear()=0;

		/**
		 * Returns the root node with the actual configuration
		 */
		virtual CConfigSection root()=0;

		/**
		 * Processes expressions into a single number if possible
		 */
		int resolveExpressions();
};

#endif /*CONFIGURATION_H_*/
