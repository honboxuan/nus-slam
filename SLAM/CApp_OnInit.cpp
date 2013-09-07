#include "CApp.h"

bool CApp::OnInit() {
	if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
		return false;
	}
	//if ((Surf_Display = SDL_SetVideoMode(800,600,32,SDL_HWSURFACE|SDL_DOUBLEBUF)) == NULL) {
	if ((Surf_Display = SDL_SetVideoMode(1920,1080,32,SDL_HWSURFACE|SDL_DOUBLEBUF)) == NULL) {
		return false;
	}
	SDL_WM_SetCaption("SLAM",0);
	if ((Surf_Overlay = SDL_CreateRGBSurface(SDL_HWSURFACE,Surf_Display->w,Surf_Display->h,32,0,0,0,0)) == NULL) {
		return false;
	}

	if (TTF_Init() < 0) {
		return false;
	}
	if ((Font = TTF_OpenFont("FreeMono.ttf",16)) == NULL) {
		return false;
	}
	if (UTM30.Init() != true) {
		return false;
	}

	return true;
}