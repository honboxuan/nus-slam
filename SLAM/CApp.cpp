#include "CApp.h"

CApp::CApp() {
	Surf_Display = NULL;
	Surf_Overlay = NULL;
	Surf_Odometry = NULL;
	Font = NULL;
	FontColor.r = 255;
	FontColor.g = 255;
	FontColor.b = 255;
	Running = true;
	Pause = false;
	Step = 0;
	Particles = new ParticleClass*[PARTICLECOUNT];
	for (uint16_t i = 0; i < PARTICLECOUNT; i++) {
		Particles[i] = new ParticleClass;
	}
}

int CApp::OnExecute() {
	if (OnInit() == false) {
		return -1;
	}
	SDL_Event Event;
	while(Running) {
		while(SDL_PollEvent(&Event)) {
			OnEvent(&Event);
		}
		if (Pause == false) {
			OnLoop();
			OnRender();
		} else {
			if (Step == -1) {
				Step = 0;
				UTM30.StepBack();
				OnLoop();
				OnRender();
			}
			if (Step == 1) {
				Step = 0;
				OnLoop();
				OnRender();
			}
		}
	}
	OnCleanup();
	return 0;
}

int main(int argc, char* argv[]) {
	CApp SLAM;
	return SLAM.OnExecute();
}