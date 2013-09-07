#include "CSurface.h"

CSurface::CSurface() {
}

SDL_Surface* CSurface::OnLoad(char* File) {
	SDL_Surface* Surf_Temp = NULL;
	SDL_Surface* Surf_Return = NULL;
	if ((Surf_Temp = SDL_LoadBMP(File)) == NULL) {
		return NULL;
	}
	Surf_Return = SDL_DisplayFormat(Surf_Temp);
	SDL_FreeSurface(Surf_Temp);
	return Surf_Return;
}

bool CSurface::OnDraw(SDL_Surface* Surf_Dest,SDL_Surface* Surf_Src,int X,int Y) {
	if (Surf_Dest == NULL || Surf_Src == NULL) {
		return false;
	}
	SDL_Rect DestR;
	DestR.x = X;
	DestR.y = Y;
	if (SDL_BlitSurface(Surf_Src,NULL,Surf_Dest,&DestR) == -1) {
		return false;
	}
	return true;
}

bool CSurface::OnDraw(SDL_Surface* Surf_Dest,SDL_Surface* Surf_Src,int X,int Y,int X2,int Y2,int W,int H) {
	if (Surf_Dest == NULL || Surf_Src == NULL) {
		return false;
	}
	SDL_Rect DestR;
	DestR.x = X;
	DestR.y = Y;
	SDL_Rect SrcR;
	SrcR.x = X2;
	SrcR.y = Y2;
	SrcR.w = W;
	SrcR.h = H;
	if (SDL_BlitSurface(Surf_Src,&SrcR,Surf_Dest,&DestR) == -1) {
		return false;
	}
	return true;
}

bool CSurface::Transparent(SDL_Surface* Surf_Dest,int R,int G,int B) {
	if(Surf_Dest == NULL) {
		return false;
	}
	SDL_SetColorKey(Surf_Dest,SDL_SRCCOLORKEY|SDL_RLEACCEL,SDL_MapRGB(Surf_Dest->format,R,G,B));
	return true;
}

bool CSurface::PutPixel(SDL_Surface* Surf_Dest,int X,int Y,int R,int G,int B) {
	if(Surf_Dest == NULL) {
		return false;
	}
	if (X >= 0 && Y >= 0 && X < Surf_Dest->w && Y < Surf_Dest->h) {
		Uint32* pixels = (Uint32*)Surf_Dest->pixels;
		Uint32* pixel = pixels + Y*Surf_Dest->pitch/4 + X;
		*pixel = SDL_MapRGB(Surf_Dest->format,R,G,B);
		return true;
	} else {
		return false;
	}
}

bool CSurface::DrawLine(SDL_Surface* Surf_Dest,int X1,int Y1,int X2,int Y2,int R,int G,int B) {
	if(Surf_Dest == NULL) {
		return false;
	}
	//------------------------OPTIMISATION------------------------
	//Could truncate calling PutPixel if X or Y already out of bounds, depending on direction
	float YDelta = float(Y2-Y1);
	float XDelta = float(X2-X1);
	if (XDelta == 0) {
		//Vertical
		if (Y2 > Y1) {
			for (int Y = Y1; Y <= Y2; Y++) {
				CSurface::PutPixel(Surf_Dest,X1,Y,R,G,B);
			}
		} else {
			for (int Y = Y1; Y >= Y2; Y--) {
				CSurface::PutPixel(Surf_Dest,X1,Y,R,G,B);
			}
		}
	} else if (YDelta == 0) {
		//Horizontal
		if (X2 > X1) {
			for (int X = X1; X <= X2; X++) {
				CSurface::PutPixel(Surf_Dest,X,Y1,R,G,B);
			}
		} else {
			for (int X = X1; X >= X2; X--) {
				CSurface::PutPixel(Surf_Dest,X,Y1,R,G,B);
			}
		}
	} else {
		float Error = 0;
		float Gradient = YDelta/XDelta;
		if (Gradient >= -1 && Gradient <= 1) {
			if (X2 > X1) {
				int Y = Y1;
				for (int X = X1; X <= X2; X++) {
					CSurface::PutPixel(Surf_Dest,X,Y,R,G,B);
					Error += Gradient;
					if (Gradient > 0) {
						while (Error >= 0.5) {
							Y++;
							Error--;
						}
					} else {
						while (Error <= -0.5) {
							Y--;
							Error++;
						}
					}
				}
			} else {
				int Y = Y2;
				for (int X = X2; X <= X1; X++) {
					CSurface::PutPixel(Surf_Dest,X,Y,R,G,B);
					Error += Gradient;
					if (Gradient > 0) {
						while (Error >= 0.5) {
							Y++;
							Error--;
						}
					} else {
						while (Error <= -0.5) {
							Y--;
							Error++;
						}
					}
				}
			}
		} else {
			Gradient = XDelta/YDelta;
			if (Y2 > Y1) {
				int X = X1;
				for (int Y = Y1; Y <= Y2; Y++) {
					CSurface::PutPixel(Surf_Dest,X,Y,R,G,B);
					Error += Gradient;
					if (Gradient > 0) {
						while (Error >= 0.5) {
							X++;
							Error--;
						}
					} else {
						while (Error <= -0.5) {
							X--;
							Error++;
						}
					}
				}
			} else {
				int X = X2;
				for (int Y = Y2; Y <= Y1; Y++) {
					CSurface::PutPixel(Surf_Dest,X,Y,R,G,B);
					Error += Gradient;
					if (Gradient > 0) {
						while (Error >= 0.5) {
							X++;
							Error--;
						}
					} else {
						while (Error <= -0.5) {
							X--;
							Error++;
						}
					}
				}
			}
		}
	}
	return true;
}

bool CSurface::DrawEllipse(SDL_Surface* Surf_Dest,int X,int Y,int Major,int Minor,int Theta,int R,int G,int B) {
	if(Surf_Dest == NULL) {
		return false;
	}
	float D = 0;
	for (float Alpha = -3.14F; Alpha < 3.141592F; Alpha += 0.01F) {
		D = Major*Minor/sqrt(pow(Major*sin(Alpha),2)+pow(Minor*cos(Alpha),2));
		CSurface::PutPixel(Surf_Dest,int(X+D*cos(Alpha+Theta)),int(Y+D*sin(Alpha+Theta)),R,G,B);
	}
	return true;
}