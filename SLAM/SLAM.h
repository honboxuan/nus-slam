#ifndef _SLAM_H_
	#define _SLAM_H_

#include <algorithm>
#include <Eigen\Dense>
#include <fstream>
#include <math.h>
#include <random>
#include <stdint.h>

//General
#define PI 3.141592F
#define DEG2RAD(X) (X*PI/180.0F)
#define RAD2DEG(X) (X*180.0F/PI)

//HOKUYO LIDAR
#define LIDARPOINTCOUNT 1081
#define LIDARANGULARRESOLUTION 0.25F
#define LIDARSTARTANGLE -(LIDARANGULARRESOLUTION*(LIDARPOINTCOUNT-1)-180.0F)/2
#define LIDARCLIPPINGRANGE 15000
#define LIDARANGULARVARIANCE 0.01F //Degrees
#define LIDARRANGEVARIANCE 0.001F //Coefficient

//SICK LIDAR
/*#define LIDARPOINTCOUNT 181
#define LIDARANGULARRESOLUTION 1.0F
#define LIDARSTARTANGLE -(LIDARANGULARRESOLUTION*(LIDARPOINTCOUNT-1)-180.0F)/2
#define LIDARCLIPPINGRANGE 50000
#define LIDARANGULARVARIANCE 0.01F //Degrees
#define LIDARRANGEVARIANCE 0.001F //Coefficient*/

//Particle allocation
#define PARTICLEINITIALWEIGHT 1e200 //Using double
#define PARTICLECOUNT 500
#define FEATURESBLOCKSIZE 10

//---------------------------------------------------
//--------------------MapElements--------------------
class PointClass {
	private:
	public:
		PointClass();
		float X, Y, Bearing, Range;
};
class CornerClass {
	private:
	public:
		CornerClass();
		uint16_t PointIndex; //Assists ScanMatching
		float X, Y, Angle, Heading;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Matrix4f Covariance;
};
class LineClass {
	private:
	public:
		LineClass();
		bool Good;
		uint16_t ClosedPointIndex; //Assists ScanMatching
		PointClass PointA, PointB;
		float Theta, R, Length;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Matrix2f Covariance;
};

//---------------------------------------------------------
//--------------------FeatureExtraction--------------------
class SplitMergeItem {
	private:
	public:
		SplitMergeItem();
		SplitMergeItem* Next;
		uint16_t IndexA, IndexB;
		bool AOpen, BOpen;
};
class LinesHolderClass {
	private:
	public:
		LinesHolderClass();
		~LinesHolderClass();
		uint16_t Count;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		LineClass* Lines;
};
class CornerItem {
	private:
	public:
		CornerItem();
		CornerItem* Next;
		uint16_t Index;
		float Angle; //Only angle is calculated at LinkedList stage
};
class CornersHolderClass {
	private:
	public:
		CornersHolderClass();
		~CornersHolderClass();
		uint16_t Count;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		CornerClass* Corners;
};
bool LinkedListDeallocate(SplitMergeItem* Head);
bool LinkedListDeallocate(CornerItem* Head);
SplitMergeItem* SplitMerge(PointClass* Points,uint16_t IndexA,uint16_t IndexB,float Threshold);
LinesHolderClass* LineFit(PointClass* Points,SplitMergeItem* Head);
CornersHolderClass* CornerExtraction(LinesHolderClass* Lines);

//-------------------------------------------------------
//--------------------DataAcquisition--------------------
class LIDAR {
	private:
		uint32_t FileLength, Index;
		uint16_t** ScanBuffer;
	public:
		LIDAR();
		~LIDAR();
		bool Init();
		uint16_t* GetScan();
		PointClass* GetPoints();
		float SinBearing[LIDARPOINTCOUNT];
		float CosBearing[LIDARPOINTCOUNT];
		void StepBack();
};
class IMUStateClass {
	private:
	public:
		IMUStateClass();
		float Roll, Pitch, Yaw;
};
class IMU {
	private:
		uint32_t FileLength, Index;
		float** IMUBuffer;
	public:
		IMU();
		~IMU();
		bool Init();
		float* GetValues();
		IMUStateClass* GetIMUState();
		void StepBack();
};

//-------------------------------------------------------
//--------------------DataAssociation--------------------
//Note: The data association algorithm can benefit from the use of more efficient neighbourhood search.
class HypothesisItem {
	private:
	public:
		HypothesisItem();
		~HypothesisItem();
		HypothesisItem* Next;
		uint16_t* Hypothesis;
		uint16_t AssociationCount;
};
bool LinkedListDeallocate(HypothesisItem* Head);
bool HypothesisPlus(uint16_t* Hypothesis,uint16_t TargetIndex,uint16_t Count,bool** Table);
float CornersMahalanobis(CornerClass* A,CornerClass* B);
HypothesisItem* CornersAssociate(CornersHolderClass* CornersA,CornersHolderClass* CornersB,float Threshold);

//----------------------------------------------------
//--------------------ScanMatching--------------------
//Procrustes analysis
//Specific association for corners, does not consider the heading of the corner
class OdometryClass {
	private:
	public:
		OdometryClass();
		float X, Y, Theta;
};
//float ScanMatchingCornersMahalanobis(CornerClass* A,CornerClass* B);
//HypothesisItem* ScanMatchingCornersAssociate(CornersHolderClass* CornersA,CornersHolderClass* CornersB,float Threshold);
OdometryClass* ScanMatching(PointClass* PointsA,PointClass* PointsB,CornersHolderClass* CornersA,CornersHolderClass* CornersB,HypothesisItem* Hypothesis);
OdometryClass* FindTranslation(PointClass* PointsA,PointClass* PointsB,CornersHolderClass* CornersA,CornersHolderClass* CornersB,HypothesisItem* Hypothesis);

//-------------------------------------------------
//--------------------Particles--------------------
//Optimally, R* tree for each type of feature
class ParticleClass {
	private:
	public:
		ParticleClass();
		~ParticleClass();
		ParticleClass(const ParticleClass &Original);
		bool StateUpdate(OdometryClass* Odometry);
		
		//Needs map management method(s)

		float X, Y, Theta;
		double Weight;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Matrix3f Covariance;
		uint16_t TimesSelected;

		CornersHolderClass* Loc2Glo(CornersHolderClass* Local);
		void AddCorner(CornerClass* Corner);

		uint16_t CornersAllocatedCount;
		CornersHolderClass* CornersHolder;
		void CornersMemoryExpansion();
};
double KalmanUpdate(CornerClass* Known,CornerClass* Detected);
ParticleClass** ParticleResample(ParticleClass** Particles);

//-----------------------------------------------
//--------------------History--------------------
class HistoryClass {
	private:
	public:
		HistoryClass();
		~HistoryClass();
		PointClass* Points;
		IMUStateClass* IMUState;
		LinesHolderClass* Lines;
		CornersHolderClass* Corners;
		OdometryClass* Odometry;
};

#endif