#include "SLAM.h"

LIDAR::LIDAR() {
	ScanBuffer = NULL;
	FileLength = 0;
	Index = 0;
	for (uint16_t i = 0; i < LIDARPOINTCOUNT; i++) {
		SinBearing[i] = sin(DEG2RAD(LIDARSTARTANGLE+LIDARANGULARRESOLUTION*i));
		CosBearing[i] = cos(DEG2RAD(LIDARSTARTANGLE+LIDARANGULARRESOLUTION*i));
		//Points[i].Bearing = DEG2RAD(LIDARSTARTANGLE+LIDARANGULARRESOLUTION*i);
	}
}
LIDAR::~LIDAR() {
	for (uint32_t i = 0; i < FileLength/(2*LIDARPOINTCOUNT); i++) {
		delete[] ScanBuffer[i];
	}
	delete[] ScanBuffer;
}
bool LIDAR::Init() {
	std::ifstream ScanSet ("output.bin",std::ifstream::binary);
	if (ScanSet.is_open()) {
		ScanSet.seekg(0,ScanSet.end);
		FileLength = uint32_t(ScanSet.tellg());
		ScanSet.seekg(0,ScanSet.beg);

		ScanBuffer = new uint16_t*[FileLength/(2*LIDARPOINTCOUNT)];
		for (uint32_t i = 0; i < FileLength/(2*LIDARPOINTCOUNT); i++) {
			ScanBuffer[i] = new uint16_t[LIDARPOINTCOUNT];
			ScanSet.read((char*)ScanBuffer[i],LIDARPOINTCOUNT*2);
		}
		ScanSet.close();
	} else {
		return false;
	}
	return true;
}
uint16_t* LIDAR::GetScan() {
	if (Index >= FileLength/(2*LIDARPOINTCOUNT)) {
		return NULL;
	}
	return ScanBuffer[Index++];
}
PointClass* LIDAR::GetPoints() {
	uint16_t* SingleScan = GetScan();
	if (SingleScan != NULL) {
		PointClass* Points = new PointClass[LIDARPOINTCOUNT];
		for (uint16_t i = 0; i < LIDARPOINTCOUNT; i++){
			if (SingleScan[i] > 15 && SingleScan[i] < LIDARCLIPPINGRANGE) {
				Points[i].Range = SingleScan[i];

				Points[i].Bearing = DEG2RAD(LIDARSTARTANGLE+LIDARANGULARRESOLUTION*i);

				Points[i].X = Points[i].Range*CosBearing[i];
				Points[i].Y = Points[i].Range*SinBearing[i];
			} else {
				Points[i].Range = 0;
				Points[i].X = 0;
				Points[i].Y = 0;
			}
		}
		return Points;
	} else {
		return NULL;
	}
}
void LIDAR::StepBack() {
	//Only used to step in debugging
	Index -= 2;
}