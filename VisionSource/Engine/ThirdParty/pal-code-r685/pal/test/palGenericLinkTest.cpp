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
	palGenericBody* floater;
	palGenericBody* anchor;
};

palGenericLinkTest::palGenericLinkTest()
: link(0)
, floater(0)
, anchor(0)
{
}

palGenericLinkTest::~palGenericLinkTest() {
}

static const Float FLOATER_OFFSET = 10.0f;

void palGenericLinkTest::SetUp() {
	AbstractPalTest::SetUp();

	palMatrix4x4 mat;
	mat_identity(&mat);
	anchor = PF->CreateGenericBody(mat);
	palBoxGeometry* box = PF->CreateBoxGeometry();
	box->Init(mat, 1.0, 1.0, 1.0, 1.0f);
	anchor->ConnectGeometry(box);
	anchor->SetDynamicsType(PALBODY_STATIC);

	mat_set_translation(&mat, 0.0f, 0.0f, FLOATER_OFFSET);
	floater = PF->CreateGenericBody(mat);
	//palBoxGeometry* box2 = PF->CreateBoxGeometry();
	//box2->Init(mat, 1.0, 1.0, 1.0, 1.0f);
	floater->SetMass(1.0f);
	floater->ConnectGeometry(box);
	floater->SetDynamicsType(PALBODY_DYNAMIC); // the default.
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
	palLink* link = PF->CreateLink(PAL_LINK_GENERIC, anchor, floater,
						  anchorFrame, floaterFrame);

	ASSERT_NE((palLink*)(NULL), link);
	cout << "Supports parameters :" << link->SupportsParameters() << endl;
	cout << "Supports parameters per axis :" << link->SupportsParametersPerAxis() << endl;

	if (link->SupportsParameters()) {
		if (link->SupportsParametersPerAxis()) {
			for (unsigned i = 0; i < 6; ++i)
			{
				if (link->SetParam(PAL_LINK_PARAM_CFM, 0.0625f, i))
					ASSERT_FLOAT_EQ(0.0625f, link->GetParam(PAL_LINK_PARAM_CFM, i));
				if (link->SetParam(PAL_LINK_PARAM_ERP, 0.25f, i))
					ASSERT_FLOAT_EQ(0.25f, link->GetParam(PAL_LINK_PARAM_ERP, i));
				if (link->SetParam(PAL_LINK_PARAM_STOP_CFM, 0.125f, i))
					ASSERT_FLOAT_EQ(0.125f, link->GetParam(PAL_LINK_PARAM_STOP_CFM, i));
				if (link->SetParam(PAL_LINK_PARAM_STOP_ERP, 0.75f, i))
					ASSERT_FLOAT_EQ(0.75f, link->GetParam(PAL_LINK_PARAM_STOP_ERP, i));
			}
			link->SetParam(PAL_LINK_PARAM_DOF_MIN, 0.0f, 0);
			link->SetParam(PAL_LINK_PARAM_DOF_MIN, 0.0f, 1);
			link->SetParam(PAL_LINK_PARAM_DOF_MIN, 0.0f, 2);
			link->SetParam(PAL_LINK_PARAM_DOF_MAX, 0.0f, 0);
			link->SetParam(PAL_LINK_PARAM_DOF_MAX, 0.0f, 1);
			link->SetParam(PAL_LINK_PARAM_DOF_MAX, 0.0f, 2);
			link->SetParam(PAL_LINK_PARAM_DOF_MIN, -M_PI_2, 3);
			link->SetParam(PAL_LINK_PARAM_DOF_MIN, 0.0f, 4);
			link->SetParam(PAL_LINK_PARAM_DOF_MIN, 0.0f, 5);
			link->SetParam(PAL_LINK_PARAM_DOF_MAX, M_PI_2, 3);
			link->SetParam(PAL_LINK_PARAM_DOF_MAX, 0.0f, 4);
			link->SetParam(PAL_LINK_PARAM_DOF_MAX, 0.0f, 5);
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
