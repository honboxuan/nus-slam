#include "SLAM.h"

HistoryClass::HistoryClass() {
	Points = NULL;
	IMUState = NULL;
	Lines = NULL;
	Corners = NULL;
	Odometry = NULL;
}
HistoryClass::~HistoryClass() {
	delete Points;
	delete IMUState;
	delete Lines;
	delete Corners;
	delete Odometry;
}