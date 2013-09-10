#include "SLAM.h"

HistoryClass::HistoryClass() {
	Points = NULL;
	Lines = NULL;
	Corners = NULL;
	Odometry = NULL;
}
HistoryClass::~HistoryClass() {
	delete Points;
	delete Lines;
	delete Corners;
	delete Odometry;
}