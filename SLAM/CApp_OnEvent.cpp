#include "CApp.h"

void CApp::OnEvent(SDL_Event* Event) {
	CEvent::OnEvent(Event);
}

void CApp::OnExit() {
	Running = false;
}

void CApp::OnKeyDown(SDLKey sym, SDLMod mod, Uint16 unicode) {
	switch (sym) {
		case SDLK_ESCAPE:
			Running = false;
			break;
		case SDLK_SPACE:
			Pause = !Pause;
			break;
		case SDLK_LEFT:
			Step = -1;
			break;
		case SDLK_RIGHT:
			Step = 1;
			break;
	}
}