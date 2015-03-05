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

//#include <stdlib.h>

#include "XMLConfiguration.h"
#include <string.h>
#include <Log2.h>

#ifdef _MSC_VER
	#define CONST_PATH_SEPARATOR	'\\'
#else
	#define CONST_PATH_SEPARATOR	'/'
#endif

#define CONST_XML_PATH_SEPARATOR	'/'	// Separator for paths defined inside XML files: <path>node/subnode/subsubnode</path>

// Define this constant if you want to output two debug files: one before and one after processing the <include> tags
//#define XMLCONF_INCLUDETAG_DEBUG_OUTPUT


// The global logger for XML Configuration.
// This avoids a CLog2 in every IConfig, and avoids logging in const functions like toFloat() const;
CLog2	gXmlLog("xml");


// ************************************************************************ //
// ************************** IXMLConfigSection *************************** //
// ************************************************************************ //

IXMLConfigSection::IXMLConfigSection(TiXmlElement* pElement)
{
	mPElement = pElement;
}

IXMLConfigSection::~IXMLConfigSection()
{
	//printf("Deleted an IXMLConfigSection.\n");
}

bool IXMLConfigSection::isSection(TiXmlElement* pElement) const
{
	// An element is a section if it has child elements (which are either properties or sections)
	return pElement->FirstChildElement() != NULL;
}

bool IXMLConfigSection::isProperty(TiXmlElement* pElement) const
{
	TiXmlNode *firstChild = pElement->FirstChild();
	if (firstChild != NULL)
		return pElement->FirstChild()->ToText() != NULL;
	else
		return false;
}

std::string IXMLConfigSection::name() const
{
	return mPElement->Value();	//TODO: make this ValueStr()?
}

bool IXMLConfigSection::hasSection(const std::string& section) const
{
	return mPElement->FirstChildElement(section.c_str()) != NULL;
}

IConfigSection* IXMLConfigSection::parent()
{
	if (mPElement->Parent() != NULL)
	{
#if 1
		if (mPElement->Parent()->Type() == TiXmlElement::TINYXML_ELEMENT)
#else
		if (mPElement->Parent()->Type() == TiXmlElement::ELEMENT)
#endif
			return (IXMLConfigSection*)registerPendingInterface(new IXMLConfigSection((TiXmlElement*)mPElement->Parent()));
		else
			return NULL;
	}
	else
		return NULL;
}

IConfigSection* IXMLConfigSection::section(const std::string& section)
{
	TiXmlElement* foundElement = mPElement->FirstChildElement(section.c_str());
	if (foundElement)
		return (IXMLConfigSection*)registerPendingInterface(new IXMLConfigSection(foundElement));
	else
		return NULL;
}

IConfigSection* IXMLConfigSection::firstSection()
{
	// Search for a section, which is a TiXmlElement in the first place.
	TiXmlElement* foundSection = NULL;
	TiXmlElement* iElement = mPElement->FirstChildElement();
	while (iElement != NULL)
	{
		if (isSection(iElement))
		{
			foundSection = iElement;
			break;
		}
		iElement = iElement->NextSiblingElement();
	}

	if (foundSection)
		return (IXMLConfigSection*)registerPendingInterface(new IXMLConfigSection(foundSection));
	else
		return NULL;
}

IConfigSection* IXMLConfigSection::nextSection()
{
	TiXmlElement* foundSection = NULL;
	TiXmlElement* iElement = mPElement->NextSiblingElement();
	while (iElement != NULL)
	{
		if (isSection(iElement))
		{
			foundSection = iElement;
			break;
		}
		iElement = iElement->NextSiblingElement();
	}

	if (foundSection)
		return (IXMLConfigSection*)registerPendingInterface(new IXMLConfigSection(foundSection));
	else
		return NULL;
}

IConfigSection* IXMLConfigSection::nextSimilarSection()
{
	TiXmlElement* foundElement = mPElement->NextSiblingElement(mPElement->Value());
	if (foundElement)
		return (IXMLConfigSection*)registerPendingInterface(new IXMLConfigSection(foundElement));
	else
		return NULL;
}

/*
bool IXMLConfigSection::get(const std::string& property, std::string *value) const
{
	TiXmlElement *element = mPElement->FirstChildElement(property.c_str());
	if (element != NULL)
	{
		*value = element->GetText();
		return true;
	}
	else
		return false;
}

bool IXMLConfigSection::get(const std::string& property, bool *value) const
{
	TiXmlElement *element = mPElement->FirstChildElement(property.c_str());
	if (element != NULL)
	{
		const char* valStr = element->GetText();

		if (strcasecmp(valStr, "true") == 0)
			*value = true;
		else if (strcasecmp(valStr, "yes") == 0)
			*value = true;
		else if (strcasecmp(valStr, "false") == 0)
			*value = false;
		else if (strcasecmp(valStr, "no") == 0)
			*value = false;
		else
			*value = (bool)atoi(valStr);

		return true;
	}
	else
		return false;
}

bool IXMLConfigSection::get(const std::string& property, char *value) const
{
	long temp;
	if (get(property, &temp))
	{
		*value = (char)temp;
		return true;
	}
	else
		return false;
}

bool IXMLConfigSection::get(const std::string& property, unsigned char *value) const
{
	unsigned long temp;
	if (get(property, &temp))
	{
		*value = (unsigned char)temp;
		return true;
	}
	else
		return false;
}

bool IXMLConfigSection::get(const std::string& property, short *value) const
{
	long temp;
	if (get(property, &temp))
	{
		*value = (short)temp;
		return true;
	}
	else
		return false;
}

bool IXMLConfigSection::get(const std::string& property, unsigned short *value) const
{
	unsigned long temp;
	if (get(property, &temp))
	{
		*value = (unsigned short)temp;
		return true;
	}
	else
		return false;
}

bool IXMLConfigSection::get(const std::string& property, int *value) const
{
	long temp;
	if (get(property, &temp))
	{
		*value = (short)temp;
		return true;
	}
	else
		return false;
}

bool IXMLConfigSection::get(const std::string& property, unsigned int *value) const
{
	unsigned long temp;
	if (get(property, &temp))
	{
		*value = (unsigned short)temp;
		return true;
	}
	else
		return false;
}

bool IXMLConfigSection::get(const std::string& property, long *value) const
{
	TiXmlElement *element = mPElement->FirstChildElement(property.c_str());
	if (element != NULL)
	{
		*value = strtol(element->GetText(), NULL, 10);
		return true;
	}
	else
		return false;
}

bool IXMLConfigSection::get(const std::string& property, unsigned long *value) const
{
	TiXmlElement *element = mPElement->FirstChildElement(property.c_str());
	if (element != NULL)
	{
		*value = strtoul(element->GetText(), NULL, 10);
		return true;
	}
	else
		return false;
}

bool IXMLConfigSection::get(const std::string& property, double *value) const
{
	TiXmlElement *element = mPElement->FirstChildElement(property.c_str());
	if (element != NULL)
	{
		*value = atof(element->GetText());
		return true;
	}
	else
		return false;
}

void IXMLConfigSection::set(const std::string& property, const std::string& value)
{

}

*/
bool IXMLConfigSection::has(const std::string& property) const
{
	return mPElement->FirstChildElement(property.c_str()) != NULL;
}

/*
bool IXMLConfigSection::isNull() const
{
	return (mPElement == NULL);
}
*/

IConfigProperty* IXMLConfigSection::get(const std::string& property)
{
	TiXmlElement* foundElement = mPElement->FirstChildElement(property.c_str());
	//printf("woohoo, called IXMLConfigSection::get(%s), which is %s\n", property.c_str(), foundElement?"non-null.":"NULL!");
	if (foundElement)
		return (IXMLConfigProperty*)registerPendingInterface(new IXMLConfigProperty(foundElement));
	else
		return NULL;
}

IConfigProperty* IXMLConfigSection::firstProperty()
{
	// Search for a property, which is a TiXmlElement in the first place.
	TiXmlElement* foundProperty = NULL;
	TiXmlElement* iElement = mPElement->FirstChildElement();
	while (iElement != NULL)
	{
		if (isProperty(iElement))
		{
			foundProperty = iElement;
			break;
		}
		iElement = iElement->NextSiblingElement();
	}

	if (foundProperty)
		return (IXMLConfigProperty*)registerPendingInterface(new IXMLConfigProperty(foundProperty));
	else
		return NULL;
}


// ************************************************************************ //
// ************************** IXMLConfigProperty ************************** //
// ************************************************************************ //

IXMLConfigProperty::IXMLConfigProperty(TiXmlElement* pElement)
{
	mPElement = pElement;
}

IXMLConfigProperty::~IXMLConfigProperty()
{
	//printf("Deleted an IXMLConfigProperty.\n");
}

bool IXMLConfigProperty::isProperty(TiXmlElement* pElement) const
{
	if (pElement->FirstChild() != NULL)
		return pElement->FirstChild()->ToText() != NULL;
	else
		// No first child at all -> cannot be a property
		return false;
}

std::string IXMLConfigProperty::name() const
{
	return mPElement->Value();	//TODO: make this ValueStr()?
}

std::string IXMLConfigProperty::toString() const
{
	const char* result = mPElement->GetText();
	if (result)
		return result;
	else
		return "";
}

void IXMLConfigProperty::set(const std::string& value)
{
	TiXmlText* text = mPElement->FirstChild()->ToText();
	if (text != NULL)
		text->SetValue(value.c_str());
}

IConfigProperty* IXMLConfigProperty::nextProperty()
{
	// Search for a property, which is a TiXmlElement in the first place.
	TiXmlElement* foundProperty = NULL;
	TiXmlElement* iElement = mPElement->NextSiblingElement();
	while (iElement != NULL)
	{
		if (isProperty(iElement))
		{
			foundProperty = iElement;
			break;
		}
		iElement = iElement->NextSiblingElement();
	}

	if (foundProperty)
		return (IXMLConfigProperty*)registerPendingInterface(new IXMLConfigProperty(foundProperty));
	else
		return NULL;
}

bool IXMLConfigProperty::isVerbose() const
{
	const char* verboseStr = mPElement->Attribute("verbose");
	if (verboseStr != NULL)
	{
		// Everything other than 'false' or '0' will be regarded as true
		if ((strcasecmp(verboseStr, "false") == 0) || (strcasecmp(verboseStr, "0") == 0))
			return false;
		else
			return true;
	}
	else
		return false;
}

/*
bool IXMLConfigProperty::isNull() const
{
	return (mPElement == NULL);
}
*/

// ************************************************************************ //
// ************************** CXMLConfiguration *************************** //
// ************************************************************************ //

CXMLConfiguration::CXMLConfiguration():
	mFilename("")
{
	mPRootConfigSection = NULL;
}

CXMLConfiguration::~CXMLConfiguration()
{
	if (mPRootConfigSection != NULL)
		delete mPRootConfigSection;
}

// Searches for the XML nodes that fall into the nodePath (can contain wildcards, slashes, etc).
bool CXMLConfiguration::findXmlNode(TiXmlElement* rootElement, const std::string& nodePath, TiXmlElementList* resultList)
{
	std::string::size_type searchPos = nodePath.find(CONST_XML_PATH_SEPARATOR, 0);
	std::string::size_type prevSearchPos = searchPos;
	std::string::size_type firstNodeLength = (searchPos==std::string::npos)?nodePath.length():searchPos;
	std::string nodeName = nodePath.substr(0, firstNodeLength);
	TiXmlElement *e = rootElement;
	// Keep searching for CONST_XML_PATH_SEPARATOR's
	while ((searchPos  != std::string::npos))
	{
		// When the path starts with "/" or contains multiple consecutive separators ("///"), the nodeName is empty and e should not be changed
		if (!nodeName.empty())
			e = e->FirstChildElement(nodeName.c_str());
		// Check if node exists
		if (e == NULL)
			return false;

		// Search for next XML path separator
		searchPos = nodePath.find(CONST_XML_PATH_SEPARATOR, searchPos+1);
		if (searchPos != std::string::npos)
			// We found another path separator; store the new node name so that it can be processed the next iteration
			nodeName = nodePath.substr(prevSearchPos+1, searchPos-prevSearchPos-1);
		else
			// Store the remainder of the path into nodeName - this can result in an empty string if the path ends with a path separator.
			nodeName = nodePath.substr(prevSearchPos+1, nodePath.length()-prevSearchPos-1);

		prevSearchPos = searchPos;
	}

	if (!nodeName.empty())
	{
		// Only one child node was specified; add it, if it exists
		e = e->FirstChildElement(nodeName.c_str());
		if (e == NULL)
			return false;
		else
			resultList->push_back(e);
	}
	else
	{
		// Add all subnodes of the found node
		for (TiXmlElement *child = e->FirstChildElement(); child; child = child->NextSiblingElement())
			resultList->push_back(child);
	}

	//resultList->push_back(rootNode->FirstChild("ode"));
	return true;
//	return rootNode->FirstChild("configuration")->FirstChild("ode");
}

bool CXMLConfiguration::loadFile(const std::string& filename)
{
	mFilename = filename;
	bool result = mXMLDocument.LoadFile(filename.c_str());
	if (mPRootConfigSection != NULL)
		delete mPRootConfigSection;
	TiXmlElement *rootElement = mXMLDocument.FirstChildElement(CONST_ConfRootSectionName);
	if (rootElement != NULL)
	{
		mPRootConfigSection = new IXMLConfigSection(rootElement);
		// Process includes inside the configuration. Can only come from other configuration files!

#ifdef XMLCONF_INCLUDETAG_DEBUG_OUTPUT
		mXMLDocument.SaveFile("XmlDEBUG_BeforeIncludes.xml");
#endif

		std::string filePath = mFilename.substr( 0, mFilename.rfind(CONST_PATH_SEPARATOR)+1);
		processIncludes(rootElement, rootElement, filePath);

#ifdef XMLCONF_INCLUDETAG_DEBUG_OUTPUT
		mXMLDocument.SaveFile("XmlDEBUG_AfterIncludes.xml");
#endif

		return result;
	}
	else
		return false;
}


void CXMLConfiguration::processIncludes(TiXmlElement* rootNode, TiXmlElement* node, const std::string& filePath)
{
	// Walk through entire document and replace all include nodes
	TiXmlElement* nextChild;
	for(TiXmlElement* child = node->FirstChildElement(); child; child = nextChild)
	{
		nextChild = child->NextSiblingElement();	// Request next child here, because we may delete child if it turns out to be an <include> node!
		if (child->Value() == std::string("include"))
		{
			// Process the include
			TiXmlElement* filenameNode = child->FirstChildElement("filename");
			if (filenameNode != NULL)
			{
				std::string filename = filenameNode->GetText();
				// If filename is not absolute, interpret the filename as relative to the main filename (mFilename)
				if (filename[1] != ':' && filename[0] != '/')
				{
					// Add mFilename's path to filename
					filename = filePath + filename;
				}
				mLogDebugLn("Processing XML include file " << filename << " ...");
				TiXmlDocument includeDoc;
				if (includeDoc.LoadFile(filename.c_str()))
				{
					// We loaded the file, now do the following: 1. Add source paths, 2. Add constants and strings.
					TiXmlElement *includeRootNode = includeDoc.FirstChildElement(CONST_ConfRootSectionName);
					if (includeRootNode != NULL)
					{
						// 1) Add all provided paths to the original file, but find nested includes in the includeDoc itself first.
						//    Paths are assumed to be defined relative to the "configuration" root node of the file
						// Also process this root node
						std::string includeFilePath = filename.substr( 0, filename.rfind(CONST_PATH_SEPARATOR)+1);
						processIncludes(includeRootNode, includeRootNode, includeFilePath);
						// Process all path nodes
						TiXmlNode	*insertAfter = child;	// Maintain node order by incrementing insertion point
						for (TiXmlElement *pathNode = child->FirstChildElement("path"); pathNode; pathNode = pathNode->NextSiblingElement("path"))
						{
							TiXmlElementList resultNodes;
							bool searchResult = findXmlNode(includeRootNode, pathNode->GetText(), &resultNodes);
							if (searchResult)
							{
								for (unsigned int iResNode=0; iResNode<resultNodes.size(); iResNode++)
								{
									// Skip constants and strings sections
									if ((resultNodes[iResNode] != includeRootNode->FirstChildElement(CONST_ConfConstantsSectionName))
											&& (resultNodes[iResNode] != includeRootNode->FirstChildElement(CONST_ConfStringsSectionName)))
									insertAfter = node->InsertAfterChild(insertAfter, *(resultNodes[iResNode]));
								}
							}
							else
								mLogErrorLn("Could not find path \"" << pathNode->GetText() << "\" in XML file \"" << filename << "\"!");
						}
						// We're done with this include node; remove it
						node->RemoveChild(child);


						// 2) Add the constants and strings sections of this file automatically to the rootNode
						//    If they do not exist in the original document, create such a section. Otherwise, merge.
						//
						// Constants (CONST_ConfConstantsSectionName)
						TiXmlElement *includeConstantsSection = includeRootNode->FirstChildElement(CONST_ConfConstantsSectionName);
						if (includeConstantsSection != NULL)
						{
							TiXmlElement *rootConstantsSection = rootNode->FirstChildElement(CONST_ConfConstantsSectionName);
							if (rootConstantsSection == NULL)
								// Insert
								rootNode->InsertEndChild(*includeConstantsSection);
							else
							{	// Merge
								for (TiXmlElement *constElement = includeConstantsSection->FirstChildElement(); constElement; constElement = constElement->NextSiblingElement())
									rootConstantsSection->InsertEndChild(*constElement);
							}
						}
						// Strings (CONST_ConfStringsSectionName)
						TiXmlElement *includeStringsSection = includeRootNode->FirstChildElement(CONST_ConfStringsSectionName);
						if (includeStringsSection != NULL)
						{
							TiXmlElement *rootStringsSection = rootNode->FirstChildElement(CONST_ConfStringsSectionName);
							if (rootStringsSection == NULL)
								// Insert
								rootNode->InsertEndChild(*includeStringsSection);
							else
							{	// Merge
								for (TiXmlElement *stringElement = includeStringsSection->FirstChildElement(); stringElement; stringElement = stringElement->NextSiblingElement())
									rootStringsSection->InsertEndChild(*stringElement);
							}
						}
					}
					else
						mLogErrorLn("Could not find root node \"" << CONST_ConfRootSectionName << "\" in XML file \"" << filename << "\"! Include paths are always relative to this root node.");
				}
				else
					mLogErrorLn("[ERROR] Could not load input XML file \"" << filename << "\"! Error: " << includeDoc.ErrorDesc());
			}
		}
		else
			// Recursive processing
			processIncludes(rootNode, child, filePath);
	}

}

bool CXMLConfiguration::saveFile(const std::string& filename)
{
	if (filename == "")	// Default
	{
		// No deviating filename was provided
		if (mFilename == "")
			// If we also don't have an mFilename, return false
			return false;
		else
			return mXMLDocument.SaveFile(mFilename.c_str());
	}
	else
		// Save using the provided filename
		return mXMLDocument.SaveFile(filename.c_str());
}

bool CXMLConfiguration::reload()
{
	if (mFilename == "")
		return false;
	else
		return mXMLDocument.LoadFile(mFilename.c_str());
}

std::string CXMLConfiguration::errorStr()
{
	std::string errStr(mXMLDocument.ErrorDesc());
	char errStr2[100];
	sprintf(errStr2, " at row %d, col %d", mXMLDocument.ErrorRow(), mXMLDocument.ErrorCol());
	return  errStr + errStr2;
}

void CXMLConfiguration::clear()
{
	mXMLDocument.Clear();
}


CConfigSection CXMLConfiguration::root()
{
	return CConfigSection(mPRootConfigSection);
}

void CXMLConfiguration::print()
{
	mXMLDocument.Print();
}
