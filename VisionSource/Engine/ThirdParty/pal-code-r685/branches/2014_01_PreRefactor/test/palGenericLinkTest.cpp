/*
 * palGenericLinkTest.cpp
 *
 *  Created on: Sep 7, 2010
 *      Author: chris
 */

#include <gtest/gtest.h>
#include <pal/pal.h>
#include "AbstractPalTest.h"
#include <pal/palFactory.h>

class palGenericLinkTest: public AbstractPalTest {
public:
	palGenericLinkTest();
	virtual ~palGenericLinkTest();
protected:
	virtual void SetUp();
	virtual void TearDown();

	palGenericLink* link;
	palBox* floater;
	palStaticBox* anchor;
};

palGenericLinkTest::palGenericLinkTest() {
}

palGenericLinkTest::~palGenericLinkTest() {
}

static const Float FLOATER_OFFSET = 10.0f;

void palGenericLinkTest::SetUp() {
	AbstractPalTest::SetUp();

	anchor = dynamic_cast<palStaticBox*> (PF->CreateObject("palStaticBox"));
	anchor->Init(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);

	floater = PF->CreateBox();
	floater->Init(0.0f, 0.0f, FLOATER_OFFSET, 1.0f, 1.0f, 1.0f, 1.0f);
}

void palGenericLinkTest::TearDown() {
	delete anchor;
	anchor = 0;
	delete floater;
	floater = 0;
}

using namespace std;

TEST_F(palGenericLinkTest, testXAxis)
{
	palMatrix4x4 anchorFrame, floaterFrame;
	mat_identity(&anchorFrame);
	mat_identity(&floaterFrame);
	mat_set_translation(&floaterFrame, 0, 0, -FLOATER_OFFSET);
	palVector3 linearLowerLimits(0.f, 0.f, 0.f);
	palVector3 linearUpperLimits(-linearLowerLimits);
	palVector3 angularLowerLimits(-M_PI_2, 0.f, 0.f);
	palVector3 angularUpperLimits(-angularLowerLimits);
	palLink* link = PF->CreateGenericLink(anchor, floater,
						  anchorFrame, floaterFrame, linearLowerLimits,
						  linearUpperLimits, angularLowerLimits, angularUpperLimits);
	ASSERT_NE((palLink*)(NULL), link);
	cout << "Supports parameters :" << link->SupportsParameters() << endl;
	cout << "Supports parameters per axis :" << link->SupportsParametersPerAxis() << endl;

	if (link->SupportsParameters()) {
		if (link->SupportsParametersPerAxis()) {
			for (unsigned i = 0; i < 6; ++i)
			{
				if (link->SetParam(PAL_LINK_PARAM_CFM, 0.0625f, 0))
					ASSERT_FLOAT_EQ(0.0625f, link->GetParam(PAL_LINK_PARAM_CFM, 0));
				if (link->SetParam(PAL_LINK_PARAM_ERP, 0.25f, 0))
					ASSERT_FLOAT_EQ(0.25f, link->GetParam(PAL_LINK_PARAM_ERP, 0));
				if (link->SetParam(PAL_LINK_PARAM_STOP_CFM, 0.125f, 0))
					ASSERT_FLOAT_EQ(0.125f, link->GetParam(PAL_LINK_PARAM_STOP_CFM, 0));
				if (link->SetParam(PAL_LINK_PARAM_STOP_ERP, 0.75f, 0))
					ASSERT_FLOAT_EQ(0.75f, link->GetParam(PAL_LINK_PARAM_STOP_ERP, 0));
			}
		}
		else {
			if (link->SetParam(PAL_LINK_PARAM_CFM, 0.0625f))
				ASSERT_FLOAT_EQ(0.0625f, link->GetParam(PAL_LINK_PARAM_CFM));
			if (link->SetParam(PAL_LINK_PARAM_ERP, 0.25f))
				ASSERT_FLOAT_EQ(0.25f, link->GetParam(PAL_LINK_PARAM_ERP));
			if (link->SetParam(PAL_LINK_PARAM_STOP_CFM, 0.125f))
				ASSERT_FLOAT_EQ(0.125f, link->GetParam(PAL_LINK_PARAM_STOP_CFM));
			if (link->SetParam(PAL_LINK_PARAM_STOP_ERP, 0.75f))
				ASSERT_FLOAT_EQ(0.75f, link->GetParam(PAL_LINK_PARAM_STOP_ERP));
		}

	}

	palVector3 pos;
	for (int i = 0; i < 100; i++) {

		floater->GetPosition(pos);

		cout << "Box position at time " << physics->GetTime() << "\tis" << pos
						<< endl;
		physics->Update(0.02f);
	}
}
