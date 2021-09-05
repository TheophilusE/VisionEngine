#include "pal/palFactory.h"
#include "pal/pal.h"
#include <stdio.h>
//#include "main.h"

#ifdef NDEBUG
#pragma comment(lib, "libpal.lib")
#else
#pragma comment(lib, "libpald.lib")
#endif


std::vector<std::string> g_engines;

int main(int argc, char *argv[]) {
	g_engines.push_back("Bullet");
//	g_engines.push_back("Dynamechs"); //experimental
//	g_engines.push_back("IBDS"); //experimental
	g_engines.push_back("Jiggle");
	g_engines.push_back("Newton");
	g_engines.push_back("Novodex");
	g_engines.push_back("ODE");
	g_engines.push_back("Tokamak");
//	g_engines.push_back("OpenTissue"); //experimental
	g_engines.push_back("TrueAxis");

	PF->LoadPALfromDLL(); 
	FILE *fout = fopen("info.txt","w");
	fprintf(fout,"PAL Benchmark Tests: Compiled on %s at %s\n",__DATE__,__TIME__);
	unsigned int i=0;
	for (i=0;i<g_engines.size();i++) {
		PF->SelectEngine(g_engines[i]);
		palPhysics *pp = PF->CreatePhysics();
		if (pp) {
			palPhysicsDesc desc;
			desc.m_nUpAxis = 0;
			//desc.m_vGravity = 9.8;
			pp->Init(desc);
			
			fprintf(fout,"%s\n",pp->GetVersion());
		}
	}
	fclose(fout);
}
