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

#include "ODECollisions.h"
#include "ODEGeoms.h"
#include "ODESim.h"

CODEContactProps::CODEContactProps(CODECollisionHandler* parentHandler):
	mpHandler(parentHandler),
	mMaterial1(0),
	mMaterial2(0)
{
	memset(&mParams, 0, sizeof(mParams));	// mode=0 and mu=0 means frictionless contact by default
}

void CODEContactProps::getParams(dSurfaceParameters *params)
{
	memcpy(params, &mParams, sizeof(dSurfaceParameters));
}

bool CODEContactProps::isMatch(int material1, int material2)
{
	if (    ((material1 == mMaterial1) && (material2 == mMaterial2))
		 || ((material1 == mMaterial2) && (material2 == mMaterial1)) )
		return true;
	else
		return false;
}

bool CODEContactProps::readConfig(const CConfigSection &configSection)
{
	bool configresult = true;

	mParams.mode = 0;
	configresult &= mLogAssert(configSection.get("material1", &mMaterial1));
	configresult &= mLogAssert(configSection.get("material2", &mMaterial2));
	configSection.get("mu", &mParams.mu);
	if (mParams.mu < 0)
		mParams.mu = dInfinity;
	// Handle K-D or ERP-CFM setings
	if (configSection.has("K"))
	{
		double K=0, D=0, ERP=0, CFM=0;
		configresult &= mLogAssert(configSection.get("K", &K));
		configresult &= mLogAssert(configSection.get("D", &D));
		mpHandler->convertKDToERPCFM(K, D, &ERP, &CFM);
		mParams.mode |= dContactSoftERP | dContactSoftCFM;
		mParams.soft_erp = ERP;
		mParams.soft_cfm = CFM;
	}
	else
	if (configSection.has("ERP"))
	{
		double ERP=0, CFM=0;
		configresult &= mLogAssert(configSection.get("ERP", &ERP));
		configresult &= mLogAssert(configSection.get("CFM", &CFM));
		mParams.mode |= dContactSoftERP | dContactSoftCFM;
		mParams.soft_erp = ERP;
		mParams.soft_cfm = CFM;
	}

	// Other settings
	if (configSection.get("bounce", &mParams.bounce))
	{
		mParams.mode |= dContactBounce;
		configSection.get("bouncevel", &mParams.bounce_vel);
	}

	double slip=0;
	if (configSection.get("slip", &slip))
	{
		// No direction-dependent slip yet
		mParams.mode |= dContactSlip1 | dContactSlip2;
		mParams.slip1 = mParams.slip2 = slip;
	}

	// Read contact force approximation mode
	int approx = 0;
	configSection.get("approximation", &approx);
	if (approx == 1)
		mParams.mode |= dContactApprox1;

	//TODO: implement other parameters

	return configresult;
}

CODECollideInfo::CODECollideInfo(CODECollisionHandler* parentHandler):
	mpHandler(parentHandler),
	mSpace1(NULL),
	mSpace2(NULL)
{
}

bool CODECollideInfo::readConfig(const CConfigSection &configSection, CODESimAccess* simAccess)
{
	bool configresult = true;

	std::string objectName1, objectName2;
	configresult &= mLogAssert(configSection.get("object1", &objectName1));
	configresult &= mLogAssert(configSection.get("object2", &objectName2));
	mSpace1 = mpHandler->getObjectSpaceID(objectName1, simAccess);
	mSpace2 = mpHandler->getObjectSpaceID(objectName2, simAccess);

	//getObjectSpaceID();

	return configresult;
}

CODECollisionHandler::CODECollisionHandler(CODESim* parentSim):
	mpSim(parentSim),
	mDefaultContactProps(this),
	mShouldDrawContacts(false)
{
	mContactGroup	= dJointGroupCreate(0);
}

CODECollisionHandler::~CODECollisionHandler()
{
	clearAll();
	dJointGroupDestroy(mContactGroup);
}

bool CODECollisionHandler::shouldDrawContacts()
{
	return mShouldDrawContacts;
}

dSpaceID CODECollisionHandler::getObjectSpaceID(const std::string &objectName, CODESimAccess* simAccess)
{
	CODEObject* foundObject = simAccess->resolveObject(objectName);
	if (foundObject != NULL)
		return foundObject->getSpaceID();
	else
		return NULL;
}

void CODECollisionHandler::clearCollideInfos()
{
	for (unsigned int iInfo=0; iInfo<mCollideInfos.size(); iInfo++)
		delete mCollideInfos[iInfo];

	mCollideInfos.clear();
}

void CODECollisionHandler::clearContactProps()
{
	for (unsigned int iInfo=0; iInfo<mContactProps.size(); iInfo++)
		delete mContactProps[iInfo];

	mContactProps.clear();
}

void CODECollisionHandler::clearDrawingObjects()
{
	for (unsigned int i=0; i<mDrawingObjects.size(); i++)
		delete mDrawingObjects[i];

	mDrawingObjects.clear();
}

void CODECollisionHandler::clearJointGroup()
{
	// Clear contact joint group
	dJointGroupEmpty(mContactGroup);
}

void CODECollisionHandler::clearAll()
{
	// Clear joint group, contact props, collide infos and drawing objects
	clearJointGroup();
	clearContactProps();
	clearCollideInfos();
	clearDrawingObjects();
}

void CODECollisionHandler::addCollideInfo(CODECollideInfo* collideInfo)
{
	mCollideInfos.push_back(collideInfo);
}

void CODECollisionHandler::addContactProps(CODEContactProps* contactProps)
{
	mContactProps.push_back(contactProps);
}

void CODECollisionHandler::addDrawingObject(const dContactGeom &contactGeom)
{
	CSimVisSphere* newObject = new CSimVisSphere(NULL);
	newObject->mX	= contactGeom.pos[0];
	newObject->mY	= contactGeom.pos[1];
	newObject->mZ	= contactGeom.pos[2];
	newObject->mRadius	= 0.015f;
	newObject->mColor.set(0.5, 0.5, 0.5, 1.0);
	mDrawingObjects.push_back(newObject);
}

void CODECollisionHandler::convertKDToERPCFM(double K, double D, double *ERP, double *CFM)
{
	mpSim->convertKDToERPCFM(K, D, ERP, CFM);
}

bool CODECollisionHandler::readConfig(const CConfigSection &configSection, CODESimAccess* simAccess)
{
	bool configresult = true;

	// Remove everything before reading a new configuration
	clearAll();

	// Should we draw contacts?
	configSection.get("drawcontacts", &mShouldDrawContacts);

	for (CConfigSection contactNode = configSection.section("contact"); !contactNode.isNull(); contactNode = contactNode.nextSimilarSection())
	{
		CODEContactProps *newProps = new CODEContactProps(this);
		if (!newProps->readConfig(contactNode))
		{
			mLogErrorLn("CODECollisionHandler::readConfig() failed to read contact props!");
			configresult = false;
		}
		addContactProps(newProps);
	}

	for (CConfigSection collisionNode = configSection.section("collision"); !collisionNode.isNull(); collisionNode = collisionNode.nextSimilarSection())
	{
		CODECollideInfo *newInfo = new CODECollideInfo(this);
		if (!newInfo->readConfig(collisionNode, simAccess))
		{
			mLogErrorLn("CODECollisionHandler::readConfig() failed to read collision info!");
			configresult = false;
		}
		addCollideInfo(newInfo);
	}

	return configresult;
}

void CODECollisionHandler::collisionCallbackFunc(void *data, dGeomID o1, dGeomID o2)
{
	CODECollisionHandler *handler = (CODECollisionHandler*)data;

	if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
	{
		logErrorLn(CLog2("ode"), "This collision handler is not built to handle spaces inside spaces!\n");
		return;
	}
	// Request CODEGeom pointers. If we are dealing with geomtransform objects, request the encapsulated geom's pointer.
	CODEGeom *geom1 = (CODEGeom*)dGeomGetData(o1);
	if (geom1->getClass() == dGeomTransformClass)
		geom1 = ((CODEGeomTransform*)geom1)->getGeom();
	CODEGeom *geom2 = (CODEGeom*)dGeomGetData(o2);
	if (geom2->getClass() == dGeomTransformClass)
		geom2 = ((CODEGeomTransform*)geom2)->getGeom();

	// Request contact properties
	CODEContactProps* contactProps = handler->getContactProps(geom1->getMaterial(), geom2->getMaterial());
	//dbgprintf("[DEBUG] Handling collision between geoms of class %s and %s.\n", geom1->getClassName().c_str(), geom2->getClassName().c_str());

	const int maxNumContacts = 10;
	dContact contacts[maxNumContacts];
	int numContacts = dCollide(o1, o2, maxNumContacts, &contacts[0].geom, sizeof(dContact));
	if (numContacts > 0)
	{
		for (int iContact=0; iContact<numContacts; iContact++)
		{
			contactProps->getParams(&contacts[iContact].surface);
			dJointID c = dJointCreateContact(handler->mpSim->getWorldID(), handler->mContactGroup, &contacts[iContact]);
			dJointAttach(c, geom1->getBody(), geom2->getBody());
			//dbgprintf("[DEBUG] Contact joint created.\n");
			// Report contact to both geoms
			geom1->reportContact(geom2, contacts[iContact].geom.pos);
			geom2->reportContact(geom1, contacts[iContact].geom.pos);
			// Add drawing ojbect for this contact, if drawing objects are to be shown
			if (handler->shouldDrawContacts())
				handler->addDrawingObject(contacts[iContact].geom);
		}
	}
}

CODEContactProps* CODECollisionHandler::getContactProps(int material1, int material2)
{
	CODEContactProps* result = &mDefaultContactProps;
	for (unsigned int iInfo=0; iInfo<mContactProps.size(); iInfo++)
	{
		if (mContactProps[iInfo]->isMatch(material1, material2))
		{
			result = mContactProps[iInfo];
			//dbgprintf("[DEBUG] Found contact props for materials %d and %d.\n", material1, material2);
			break;
		}
	}
//	if (result == &mDefaultContactProps)
//		dbgprintf("[DEBUG] Couldn't find contact props for materials %d and %d!\n", material1, material2);

	return result;
}

void CODECollisionHandler::copySurfaceParameters(dSurfaceParameters* surfaceParams, CODEContactProps* contactProps)
{
	contactProps->getParams(surfaceParams);
}

void CODECollisionHandler::calculateCollisions(CODESimAccess *simAccess)
{
	// Clear contact joint group
	clearJointGroup();
	// Clear drawing objects
	clearDrawingObjects();
	// Clear reported contacts within all geoms
	for (unsigned int iObject=0; iObject<simAccess->getObjects().size(); iObject++)
	{
		CODEObject *obj = simAccess->getObjects()[iObject];
		for (unsigned int iGeom=0; iGeom<obj->getGeoms().size(); iGeom++)
		{
			obj->getGeoms()[iGeom]->clearReportedContacts();
		}
	}

	// Call collide function for all desired space combinations, as defined in mCollideInfos
	for (unsigned int iInfo=0; iInfo<mCollideInfos.size(); iInfo++)
		dSpaceCollide2((dGeomID)mCollideInfos[iInfo]->getSpace1(), (dGeomID)mCollideInfos[iInfo]->getSpace2(), this, collisionCallbackFunc);
}
