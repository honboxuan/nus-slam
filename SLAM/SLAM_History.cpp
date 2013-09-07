#include "SLAM.h"

HistoryClass::HistoryClass() {
	Points = NULL;
	Corners = NULL;
	Odometry = NULL;
}
HistoryClass::~HistoryClass() {
	delete Points;
	delete Corners;
	delete Odometry;
}