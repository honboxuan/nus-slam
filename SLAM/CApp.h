#ifndef _CAPP_H_
	#define _CAPP_H_

#include <SDL.h>
#include <SDL_ttf.h>
#include "CEvent.h"
#include "CSurface.h"

#include "SLAM.h"

#include <vld.h>

class CApp : public CEvent {
	//Standard
	private:
		bool Running;
		SDL_Surface* Surf_Display;
	public:
		CApp();
		int OnExecute();
		bool OnInit();
		void OnEvent(SDL_Event* Event);
		void OnExit();
		void OnLoop();
		void OnRender();
		void OnCleanup();
	
	//SLAM
	private:
		LIDAR UTM30;
		IMU sbgIMU;
		SDL_Surface* Surf_Overlay;
		SDL_Surface* Surf_Odometry;
		HistoryClass Previous;
		ParticleClass** Particles;
		SDL_Color FontColor;
		TTF_Font* Font;
		bool Pause;
		int Step;
	public:
		void OnKeyDown(SDLKey sym, SDLMod mod, Uint16 unicode);
};

#endif