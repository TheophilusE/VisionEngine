#ifndef IRRLICHT_GLOBALS_H
#define IRRLICHT_GLOBALS_H

/*
 * Globals for DemoScene, which is used by lots of tests.
 */

#include "main.h"

#include <irrlicht/irrlicht.h>
#include <vector>
#include <string>

using namespace irr;
using namespace irr::video;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

irr::IrrlichtDevice*		g_device = 0;
video::IVideoDriver*		g_driver = 0;
scene::ISceneManager*		g_smgr = 0;
gui::IGUIEnvironment*		g_gui = 0;
std::vector<std::string>	g_engines;
std::vector<SColor>			g_colors;
bool						g_SceneFinished = false;

void ApplyMaterialToNode(ISceneNode* node) {
	if (node) {
		node->setMaterialFlag(EMF_LIGHTING, true);
		node->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
		node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);

		node->setMaterialType(EMT_SOLID);
		for (unsigned int ii=0;ii<node->getMaterialCount();ii++) {
			SColor emis(255,64,64,64);
			SColor col(255,128,128,128);
			SColor white(255,255,255,255);
			node->getMaterial(ii).DiffuseColor = col;
			node->getMaterial(ii).AmbientColor = col;
			node->getMaterial(ii).EmissiveColor= emis;
			node->getMaterial(ii).SpecularColor= white;
			node->getMaterial(ii).Shininess = 0.0f;
		}
	}
}

#endif
