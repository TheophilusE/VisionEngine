/*
 * palLinkFeedbackTest.cpp
 *
 *  Created on: Nov 15, 2010
 *      Author: chris
 */

#include "palLinkFeedbackTest.h"
#include <pal/palFactory.h>
#include <pal/palLinks.h>
#include <pal/palMath.h>
#include <ostream>

using namespace std;

palLinkFeedbackTest::palLinkFeedbackTest() {
}

palLinkFeedbackTest::~palLinkFeedbackTest() {
}

TEST_F(palLinkFeedbackTest, prismaticTest) {
	anchor = dynamic_cast<palStaticBox*> (PF->CreateObject("palStaticBox"));
	anchor->Init(0.0f, 10.0f, 0.0f, 1.0f, 1.0f, 1.0f);
	floater = PF->CreateBox();
	float mass = 1.0f;
	floater->Init(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, mass);

	palRevoluteLink* revoluteLink = dynamic_cast<palRevoluteLink*>(PF->CreateLink(PAL_LINK_REVOLUTE, anchor, floater, palVector3(0.0f, 10.0f, 0.0f), palVector3(0.0f, 0.0f, -1.0f)));
	revoluteLink->SetParam(PAL_LINK_PARAM_DOF_MIN, -M_PI/2.0);
	revoluteLink->SetParam(PAL_LINK_PARAM_DOF_MAX, M_PI/2.0);

	palLinkFeedback* feedback = revoluteLink->GetFeedback();
	ASSERT_NE((void*) 0, feedback);
	feedback->SetEnabled(true);


	for (int i = 0; i < 10; i++) {
		float force = feedback->GetValue();
		palVector3 position;
		floater->GetPosition(position);
		cout << "step " << i << "\tforce = " << force << "\tangle=" << revoluteLink->GetAngle() << "\tpos=" << position << endl;
		physics->Update(0.02f);
	}
}
