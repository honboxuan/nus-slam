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
IMUStateClass::IMUStateClass() {
	Roll = 0;
	Pitch = 0;
	Yaw = 0;
}
IMU::IMU() {
	IMUBuffer = NULL;
	FileLength = 0;
	Index = 0;
}
IMU::~IMU() {
	for (uint32_t i = 0; i < FileLength/12; i++) {
		delete[] IMUBuffer[i];
	}
	delete[] IMUBuffer;
}
bool LIDAR::Init() {
	std::ifstream ScanSet ("lidar.bin",std::ifstream::binary);
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
bool IMU::Init() {
	std::ifstream ScanSet ("imu.bin",std::ifstream::binary);
	if (ScanSet.is_open()) {
		ScanSet.seekg(0,ScanSet.end);
		FileLength = uint32_t(ScanSet.tellg());
		ScanSet.seekg(0,ScanSet.beg);

		IMUBuffer = new float*[FileLength/12];
		for (uint32_t i = 0; i < FileLength/12; i++) {
			IMUBuffer[i] = new float[3];
			ScanSet.read((char*)IMUBuffer[i],12);
		}
		ScanSet.close();
	} else {
		return false;
	}
	return true;
}
float* IMU::GetValues() {
	if (Index >= FileLength/12) {
		return NULL;
	}
	return IMUBuffer[Index++];
}
IMUStateClass* IMU::GetIMUState() {
	float* Values = GetValues();
	if (Values != NULL) {
		IMUStateClass* IMUState = new IMUStateClass;
		IMUState->Roll = Values[0];
		IMUState->Pitch = Values[1];
		IMUState->Yaw = Values[2];
		return IMUState;
	} else {
		return NULL;
	}
}
void IMU::StepBack() {
	//Only used to step in debugging
	Index -= 2;
}