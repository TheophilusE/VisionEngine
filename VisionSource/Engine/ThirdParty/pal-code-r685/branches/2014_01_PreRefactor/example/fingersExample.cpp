#include "graphics.h"
#include <pal/pal.h>
#include <pal/palFactory.h>
#include <iostream>
#include <unistd.h>
#include <pal/palStringable.h>
#include <pal/palMath.h>

palBox* boxA;
palBox* boxB;
palRevoluteLink* linkA;
palRevoluteLink* linkB;
palAngularMotor* motorA;
palAngularMotor* motorB;

using namespace std;

bool handleInput() {
	SDL_Event event;
	bool done = false;
	while (SDL_PollEvent(&event) && !done) {
		switch (event.type) {
		case SDL_QUIT:
			done = true;
			break;
		}
	}
	return done;
}

#ifdef SIMPLE_OUTPUT
static int degrees(float a, float b) {
    return int(atan2f(a, b) / M_PI * 180.0f);
}
#endif

void EventLoop(palPhysics* physics) {
	bool done = false;
	while (!(done = handleInput())) {
#ifdef SIMPLE_OUTPUT
		palVector3 aPos, bPos;
		boxA->GetPosition(aPos);
        aPos.y -= 0.5;
		boxB->GetPosition(bPos);
        bPos.y -= 0.5;
		cout << "A: " << aPos << " r=" << vec_mag(&aPos) << " a=" << degrees(aPos.x, aPos.z) << "\t"
			 << "B: " << bPos << " r=" << vec_mag(&bPos) << " a=" << degrees(bPos.x, bPos.z) << endl;
#else
        PF->DumpObjects();
#endif

        motorA->Init(linkA, 100);
        motorB->Init(linkB, 100);
        motorA->Update(-30);
        motorB->Update(30);
		physics->Update(0.01);

		g_eng->Clear();
		for (unsigned int i = 0; i < g_Graphics.size(); i++) {
			g_Graphics[i]->Display();
		}
		g_eng->Flip();
		usleep(100000);
	}
}

int main(int argc, char* argv[]) {
#ifndef PAL_STATIC
	PF->LoadPhysicsEngines();
#endif

	PF->SelectEngine("Bullet");
	palPhysics *pp = PF->CreatePhysics(); //create the main physics class
	if (pp == NULL) {
		std::cerr
						<< "Failed to create the physics engine. Check to see if you spelt the engine name correctly, or that the engine DLL is in the right location"
						<< std::endl;
		return 1;
	}

	palPhysicsDesc desc;
	pp->Init(desc);

	// init graphics system
	g_eng = new SDLGLEngine();
	if (!g_eng->Init(640, 480)) {
		std::cerr << "Failed to create rendering engine" << std::endl;
		return 2;
	}
	g_eng->SetViewMatrix(20.0f, 20.0f, 20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
					0.0f);

	pp->SetDebugDraw(new palDebugDraw(50.0f));

	// make our objects
	palTerrainPlane *pt = PF->CreateTerrainPlane(); //create the ground
	pt->Init(0, 0, 0, 50.0f); //initialize it, set its location to 0,0,0 and minimum size to 50
	palMaterials* materials = PF->CreateMaterials();
	palMaterialDesc iceDesc;
	iceDesc.m_fKinetic = 0;
	iceDesc.m_fStatic = 0;
	palMaterial* ice = materials->NewMaterial("ice", iceDesc);
	pt->SetMaterial(ice);
	BuildGraphics(pt);

	float altitude = 0.5f;

	palBox* hand = PF->CreateBox();
	hand->Init(0, altitude, 0, 1, 1, 6, 10);
	BuildGraphics(hand);
    
	boxA = PF->CreateBox();
	boxA->Init(6, altitude, 2, 10, 1, 1, 2.0f);
	BuildGraphics(boxA);

	linkA = PF->CreateRevoluteLink(hand, boxA, 0, altitude, 2, 0, 1, 0);
	linkA->SetLimits(-M_PI, M_PI);
	motorA = PF->CreateAngularMotor(linkA, 100);
	motorA->Update(-30);
	BuildGraphics(linkA);

	boxB = PF->CreateBox();
	boxB->Init(6, altitude, -2, 10, 1, 1, 2.0f);
	/*GraphicsObject* gObjectB =*/ BuildGraphics(boxB);

	linkB = PF->CreateRevoluteLink(hand, boxB, 0, altitude, -2, 0, 1, 0);
	linkB->SetLimits(-M_PI, M_PI);
	motorB = PF->CreateAngularMotor(linkB, 100);
	motorB->Update(30);
	BuildGraphics(linkB);

	EventLoop(pp);

	PF->Cleanup();

        return 0;
}
