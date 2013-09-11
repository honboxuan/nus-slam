#include "CApp.h"

void CApp::OnCleanup() {
	for (uint16_t i = 0; i < PARTICLECOUNT; i++) {
		delete Particles[i];
	}
	delete[] Particles;

	SDL_FreeSurface(Surf_Display);
	SDL_FreeSurface(Surf_Map);
	SDL_FreeSurface(Surf_Overlay);
	SDL_FreeSurface(Surf_Odometry);
	TTF_CloseFont(Font);
	TTF_Quit();
	SDL_Quit();
}