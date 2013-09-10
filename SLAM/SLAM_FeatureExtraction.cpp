#include "SLAM.h"

SplitMergeItem::SplitMergeItem() {
	Next = NULL;
	IndexA = 0;
	IndexB = 0;
	AOpen = false;
	BOpen = false;
}
LinesHolderClass::LinesHolderClass() {
	Count = 0;
	Lines = NULL;
}
LinesHolderClass::~LinesHolderClass() {
	delete[] Lines;
}
CornerItem::CornerItem() {
	Index = 0;
	Angle = 0;
	Next = NULL;
}
CornersHolderClass::CornersHolderClass() {
	Count = 0;
	Corners = NULL;
}
CornersHolderClass::~CornersHolderClass() {
	delete[] Corners;
}
bool LinkedListDeallocate(SplitMergeItem* Head) {
	if (Head == NULL) {
		return false;
	}
	while (Head != NULL) {
		SplitMergeItem* Current = Head;
		Head = Head->Next;
		delete Current;
	}
	return true;
}
bool LinkedListDeallocate(CornerItem* Head) {
	if (Head == NULL) {
		return false;
	}
	while (Head != NULL) {
		CornerItem* Current = Head;
		Head = Head->Next;
		delete Current;
	}
	return true;
}
SplitMergeItem* SplitMerge(PointClass* Points,uint16_t IndexA,uint16_t IndexB,float Threshold) {
	//Using Hesse standard form directly
	float Theta = atan2(Points[IndexA].X-Points[IndexB].X,Points[IndexB].Y-Points[IndexA].Y);
	float R = Points[IndexA].X*cos(Theta)+Points[IndexA].Y*sin(Theta);

	float DistanceCurrent = 0, DistanceWinner = 0;
	uint16_t IndexWinner = 0;
	for (uint16_t i = IndexA+1; i <= IndexB-1; i++) {
		DistanceCurrent = abs(Points[i].Range*cos(Theta-Points[i].Bearing)-R);
		if (DistanceWinner < DistanceCurrent) {
			DistanceWinner = DistanceCurrent;
			IndexWinner = i;
		}
	}
	if (DistanceWinner > Threshold) {
		//Split
		SplitMergeItem* A = SplitMerge(Points,IndexA,IndexWinner,Threshold);
		SplitMergeItem* B = SplitMerge(Points,IndexWinner,IndexB,Threshold);
		SplitMergeItem* Current = A;
		//Merge
		while (Current->Next != NULL) {
			Current = Current->Next;
		}
		Current->Next = B;
		return A;
	} else {
		SplitMergeItem* LineSegment = new SplitMergeItem;
		LineSegment->IndexA = IndexA;
		LineSegment->IndexB = IndexB;
		return LineSegment;
	}
}
LinesHolderClass* LineFit(PointClass* Points,SplitMergeItem* Head) {
	SplitMergeItem* Current = Head;
	uint16_t LineCount = 1;
	while (Current->Next != NULL) {
		Current = Current->Next;
		LineCount++;
	}
	LinesHolderClass* LinesHolder = new LinesHolderClass;
	LinesHolder->Lines = new LineClass[LineCount];
	LinesHolder->Count = LineCount;
	Current = Head;
	for (uint16_t j = 0; j < LineCount; j++) {
		uint16_t N = Current->IndexB-Current->IndexA+1;
		if (N < 10) {
			LinesHolder->Lines[j].Good = false;
		}

		
		//----------------------------------------------------------------------------------------------------------------
		//Weighted Line Fitting Algorithms for Mobile Robot Map Building and Efficient Data Representation, Pfister 2003
		//Initial estimate of theta using extreme points
		float Theta = atan2(Points[Current->IndexA].X-Points[Current->IndexB].X,Points[Current->IndexB].Y-Points[Current->IndexA].Y);
		float R = 0, P_RR = 0, DeltaTheta = 0;
		uint16_t Loops = 0;
		do {
			R = 0;
			P_RR = 0;
			Theta += DeltaTheta;
			//Proposition 1
			for (uint16_t i = Current->IndexA; i <= Current->IndexB; i++) {
				float var_d = LIDARRANGEVARIANCE*Points[i].Range;
				float var_theta = DEG2RAD(LIDARANGULARVARIANCE);
				float d_k = Points[i].Range;
				float c_k = cos(Theta-Points[i].Bearing);
				float s_k = sin(Theta-Points[i].Bearing);
				float P_k = var_d*pow(c_k,2)+var_theta*pow(d_k*s_k,2);
				R += d_k*c_k/P_k;
				P_RR += 1/P_k;
			}
			P_RR = 1/P_RR;
			R *= P_RR;
			//Proposition 2
			float Nominator = 0, Denominator = 0;
			for (uint16_t i = Current->IndexA; i <= Current->IndexB; i++) {
				float var_d = LIDARRANGEVARIANCE*Points[i].Range;
				float var_theta = DEG2RAD(LIDARANGULARVARIANCE);
				float d_k = Points[i].Range;
				float c_k = cos(Theta-Points[i].Bearing);
				float s_k = sin(Theta-Points[i].Bearing);
				float a_k = pow(d_k*c_k-R,2);
				float a_k_prime = -2*d_k*s_k*(d_k*c_k-R);
				float a_k_dprime = 2*pow(d_k*s_k,2)-2*d_k*c_k*(d_k*c_k-R);
				float b_k = var_d*pow(c_k,2)+var_theta*pow(d_k*s_k,2);
				float b_k_prime = 2*(pow(d_k,2)*var_theta-var_d)*c_k*s_k;
				float b_k_dprime = 2*(pow(d_k,2)*var_theta-var_d)*(pow(c_k,2)-pow(s_k,2));
				Nominator += (b_k*a_k_prime-a_k*b_k_prime)/pow(b_k,2);
				Denominator += ((a_k_dprime*b_k-a_k*b_k_dprime)*b_k-2*(a_k_prime*b_k-a_k*b_k_prime)*b_k_prime)/pow(b_k,3);
			}
			DeltaTheta = -Nominator/Denominator;
			Loops++;
		} while (abs(DeltaTheta) > 0.001 && Loops < 1000);
		//Proposition 3
		float P_RTheta = 0, P_ThetaTheta = 0, G_T_dprime = 0;
		for (uint16_t i = Current->IndexA; i <= Current->IndexB; i++) {
			float var_d = LIDARRANGEVARIANCE*Points[i].Range;
			float var_theta = DEG2RAD(LIDARANGULARVARIANCE);
			float d_k = Points[i].Range;
			float c_k = cos(Theta-Points[i].Bearing);
			float s_k = sin(Theta-Points[i].Bearing);
			float a_k = pow(d_k*c_k-R,2);
			float a_k_prime = -2*d_k*s_k*(d_k*c_k-R);
			float a_k_dprime = 2*pow(d_k*s_k,2)-2*d_k*c_k*(d_k*c_k-R);
			float b_k = var_d*pow(c_k,2)+var_theta*pow(d_k*s_k,2);
			float b_k_prime = 2*(pow(d_k,2)*var_theta-var_d)*c_k*s_k;
			float b_k_dprime = 2*(pow(d_k,2)*var_theta-var_d)*(pow(c_k,2)-pow(s_k,2));
			P_RTheta += (2*Points[i].Range*sin(Theta-Points[i].Bearing))/b_k;
			P_ThetaTheta += 4*pow(Points[i].Range*sin(Theta-Points[i].Bearing),2)/b_k;
			G_T_dprime += ((a_k_dprime*b_k-a_k*b_k_dprime)*b_k-2*(a_k_prime*b_k-a_k*b_k_prime)*b_k_prime)/pow(b_k,3);
		}
		P_RTheta *= P_RR/G_T_dprime;
		P_ThetaTheta /= pow(G_T_dprime,2);

		LinesHolder->Lines[j].Theta = Theta;
		LinesHolder->Lines[j].R = R;
		LinesHolder->Lines[j].Covariance(0,0) = P_RR;
		LinesHolder->Lines[j].Covariance(0,1) = P_RTheta;
		LinesHolder->Lines[j].Covariance(1,0) = P_RTheta;
		LinesHolder->Lines[j].Covariance(1,1) = P_ThetaTheta;

		if (Current->Next != NULL) {
			Current = Current->Next;
		}
		//----------------------------------------------------------------------------------------------------------------

		/*
		//Old algorithm
		LinesHolder->Lines[j].Theta = atan2(Points[Current->IndexA].X-Points[Current->IndexB].X,Points[Current->IndexB].Y-Points[Current->IndexA].Y);
		LinesHolder->Lines[j].R = Points[Current->IndexA].X*cos(LinesHolder->Lines[j].Theta)+Points[Current->IndexA].Y*sin(LinesHolder->Lines[j].Theta);
		LinesHolder->Lines[j].Covariance = Eigen::Matrix2f::Identity();
		if (Current->Next != NULL) {
			Current = Current->Next;
		}
		*/
	}

	//Find line points
	Current = Head;
	float SinTheta = 0, CosTheta = 0;
	for (uint16_t i = 0; i < LineCount; i++) {
		SinTheta = sin(LinesHolder->Lines[i].Theta);
		CosTheta = cos(LinesHolder->Lines[i].Theta);
		if (Current->AOpen) {
			LinesHolder->Lines[i].PointA.X = Points[Current->IndexA].X-(Points[Current->IndexA].Range*cos(LinesHolder->Lines[i].Theta-Points[Current->IndexA].Bearing)-LinesHolder->Lines[i].R)*CosTheta;
			LinesHolder->Lines[i].PointA.Y = Points[Current->IndexA].Y-(Points[Current->IndexA].Range*cos(LinesHolder->Lines[i].Theta-Points[Current->IndexA].Bearing)-LinesHolder->Lines[i].R)*SinTheta;
		}
		if (Current->BOpen) {
			LinesHolder->Lines[i].PointB.X = Points[Current->IndexB].X-(Points[Current->IndexB].Range*cos(LinesHolder->Lines[i].Theta-Points[Current->IndexB].Bearing)-LinesHolder->Lines[i].R)*CosTheta;
			LinesHolder->Lines[i].PointB.Y = Points[Current->IndexB].Y-(Points[Current->IndexB].Range*cos(LinesHolder->Lines[i].Theta-Points[Current->IndexB].Bearing)-LinesHolder->Lines[i].R)*SinTheta;
		} else {
			float SinTheta2 = sin(LinesHolder->Lines[i+1].Theta);
			float CosTheta2 = cos(LinesHolder->Lines[i+1].Theta);
			LinesHolder->Lines[i].PointB.X = (LinesHolder->Lines[i].R*SinTheta2-LinesHolder->Lines[i+1].R*SinTheta)/(CosTheta*SinTheta2-CosTheta2*SinTheta);
			LinesHolder->Lines[i].PointB.Y = (LinesHolder->Lines[i+1].R*CosTheta-LinesHolder->Lines[i].R*CosTheta2)/(CosTheta*SinTheta2-CosTheta2*SinTheta);
			LinesHolder->Lines[i+1].PointA.X = LinesHolder->Lines[i].PointB.X;
			LinesHolder->Lines[i+1].PointA.Y = LinesHolder->Lines[i].PointB.Y;
			
			LinesHolder->Lines[i].ClosedPointIndex = Current->IndexB;
		}
		if (Current->Next != NULL) {
			Current = Current->Next;
		}
	}
	//Find lengths
	for (uint16_t i = 0; i < LineCount; i++) {
		LinesHolder->Lines[i].Length = sqrt(pow(LinesHolder->Lines[i].PointA.X-LinesHolder->Lines[i].PointB.X,2)+pow(LinesHolder->Lines[i].PointA.Y-LinesHolder->Lines[i].PointB.Y,2));
	}
	return LinesHolder;
}
CornersHolderClass* CornerExtraction(LinesHolderClass* LinesHolder) {
	CornerItem* Head = NULL;
	CornerItem* Current = NULL;
	uint16_t CornerCount = 0;
	for (uint16_t i = 0; i < LinesHolder->Count-1; i++ ) {
		if (LinesHolder->Lines[i].Good && LinesHolder->Lines[i+1].Good) {
			if (LinesHolder->Lines[i].PointB.X == LinesHolder->Lines[i+1].PointA.X && LinesHolder->Lines[i].PointB.Y == LinesHolder->Lines[i+1].PointA.Y) {
				//Lines of corner must be above certain length
				if (LinesHolder->Lines[i].Length > 250 && LinesHolder->Lines[i+1].Length > 250) {
					//Limit acceptable corner angles
					float Angle = abs(LinesHolder->Lines[i+1].Theta-LinesHolder->Lines[i].Theta);
					if (Angle > PI) {
						Angle -= PI;
					}
					if (Angle > DEG2RAD(45) && Angle < DEG2RAD(135)) {
						if (Head == NULL) {
							Head = new CornerItem;
							Head->Index = i;
							Head->Angle = Angle;
							Current = Head;
						} else {
							Current->Next = new CornerItem;
							Current = Current->Next;
							Current->Index = i;
							Current->Angle = Angle;
						}
						CornerCount++;
					}
				}
			}
		}
	}
	CornersHolderClass* CornersHolder = new CornersHolderClass;
	if (CornerCount != 0) {
		CornersHolder->Count = CornerCount;
		CornersHolder->Corners = new CornerClass[CornerCount];
		Current = Head;
		for (uint16_t i = 0; i < CornerCount; i++) {
			CornersHolder->Corners[i].PointIndex = LinesHolder->Lines[Current->Index].ClosedPointIndex;

			CornersHolder->Corners[i].X = LinesHolder->Lines[Current->Index].PointB.X;
			CornersHolder->Corners[i].Y = LinesHolder->Lines[Current->Index].PointB.Y;
			/*CornersHolder->Corners[i].Angle = abs(LinesHolder->Lines[Current->Index+1].Theta-LinesHolder->Lines[Current->Index].Theta);
			if (CornersHolder->Corners[i].Angle > PI) {
				CornersHolder->Corners[i].Angle -= PI;
			}*/
			CornersHolder->Corners[i].Angle = Current->Angle;
			float Ax = (LinesHolder->Lines[Current->Index].PointB.X-LinesHolder->Lines[Current->Index].PointA.X)/LinesHolder->Lines[Current->Index].Length;
			float Bx = (LinesHolder->Lines[Current->Index+1].PointA.X-LinesHolder->Lines[Current->Index+1].PointB.X)/LinesHolder->Lines[Current->Index+1].Length;
			float Ay = (LinesHolder->Lines[Current->Index].PointB.Y-LinesHolder->Lines[Current->Index].PointA.Y)/LinesHolder->Lines[Current->Index].Length;
			float By = (LinesHolder->Lines[Current->Index+1].PointA.Y-LinesHolder->Lines[Current->Index+1].PointB.Y)/LinesHolder->Lines[Current->Index+1].Length;
			CornersHolder->Corners[i].Heading = atan2(Ay+By,Ax+Bx);


			
			/*CornersHolder->Corners[i].Covariance(0,0) = 0;
			CornersHolder->Corners[i].Covariance(0,1) = 0;
			CornersHolder->Corners[i].Covariance(0,2) = 0;
			CornersHolder->Corners[i].Covariance(0,3) = 0;
			CornersHolder->Corners[i].Covariance(1,0) = 0;
			CornersHolder->Corners[i].Covariance(1,1) = 0;
			CornersHolder->Corners[i].Covariance(1,2) = 0;
			CornersHolder->Corners[i].Covariance(1,3) = 0;
			CornersHolder->Corners[i].Covariance(2,0) = 0;
			CornersHolder->Corners[i].Covariance(2,1) = 0;
			CornersHolder->Corners[i].Covariance(2,2) = 0;
			CornersHolder->Corners[i].Covariance(2,3) = 0;
			CornersHolder->Corners[i].Covariance(3,0) = 0;
			CornersHolder->Corners[i].Covariance(3,1) = 0;
			CornersHolder->Corners[i].Covariance(3,2) = 0;
			CornersHolder->Corners[i].Covariance(3,3) = 0;*/


			CornersHolder->Corners[i].Covariance = Eigen::Matrix4f::Identity();
			CornersHolder->Corners[i].Covariance(2,2) = 0.001F;
			CornersHolder->Corners[i].Covariance(3,3) = 0.0001F;

			//Covariance matrix guesstimation (dropping square roots, assume uncorrelated)
			/*
			float SinTheta = sin(LinesHolder->Lines[Current->Index].Theta);
			float CosTheta = cos(LinesHolder->Lines[Current->Index].Theta);
			float SinTheta2 = sin(LinesHolder->Lines[Current->Index+1].Theta);
			float CosTheta2 = cos(LinesHolder->Lines[Current->Index+1].Theta);
			float SinThetaError = sin(LinesHolder->Lines[Current->Index].Theta+LinesHolder->Lines[Current->Index].Covariance(1,1))-SinTheta;
			float CosThetaError = cos(LinesHolder->Lines[Current->Index].Theta+LinesHolder->Lines[Current->Index].Covariance(1,1))-CosTheta;
			float SinThetaError2 = sin(LinesHolder->Lines[Current->Index+1].Theta+LinesHolder->Lines[Current->Index+1].Covariance(1,1))-SinTheta2;
			float CosThetaError2 = cos(LinesHolder->Lines[Current->Index+1].Theta+LinesHolder->Lines[Current->Index+1].Covariance(1,1))-CosTheta2;
			//Propagation from finding X and Y:
				//LinesHolder->Lines[i].PointB.X = (LinesHolder->Lines[i].R*SinTheta2-LinesHolder->Lines[i+1].R*SinTheta)/(CosTheta*SinTheta2-CosTheta2*SinTheta);
				//LinesHolder->Lines[i].PointB.Y = (LinesHolder->Lines[i+1].R*CosTheta-LinesHolder->Lines[i].R*CosTheta2)/(CosTheta*SinTheta2-CosTheta2*SinTheta);
			CornersHolder->Corners[i].Covariance = Eigen::Matrix4f::Zero();
			CornersHolder->Corners[i].Covariance(0,0) = (LinesHolder->Lines[Current->Index].Covariance(0,0)/pow(LinesHolder->Lines[Current->Index].R,2)+SinThetaError2/pow(SinTheta2,2)+
														LinesHolder->Lines[Current->Index+1].Covariance(0,0)/pow(LinesHolder->Lines[Current->Index+1].R,2)+SinThetaError/pow(SinTheta,2))/
															pow(LinesHolder->Lines[Current->Index].R*SinTheta2-LinesHolder->Lines[Current->Index+1].R*SinTheta,2)+
														(CosThetaError/pow(CosTheta,2)+SinThetaError2/pow(SinTheta2,2))+
														(CosThetaError2/pow(CosTheta2,2)+SinThetaError/pow(SinTheta,2))/
															pow(CosTheta*SinTheta2-CosTheta2*SinTheta,2);
			CornersHolder->Corners[i].Covariance(1,1) = (LinesHolder->Lines[Current->Index+1].Covariance(0,0)/pow(LinesHolder->Lines[Current->Index+1].R,2)+CosThetaError/pow(CosTheta,2)+
														LinesHolder->Lines[Current->Index].Covariance(0,0)/pow(LinesHolder->Lines[Current->Index].R,2)+CosThetaError2/pow(CosTheta2,2))/
															pow(LinesHolder->Lines[Current->Index+1].R*CosTheta-LinesHolder->Lines[Current->Index].R*CosTheta2,2)+
														(CosThetaError/pow(CosTheta,2)+SinThetaError2/pow(SinTheta2,2))+
														(CosThetaError2/pow(CosTheta2,2)+SinThetaError/pow(SinTheta,2))/
															pow(CosTheta*SinTheta2-CosTheta2*SinTheta,2);
			CornersHolder->Corners[i].Covariance(2,2) = LinesHolder->Lines[Current->Index].Covariance(1,1)+LinesHolder->Lines[Current->Index+1].Covariance(1,1);
			CornersHolder->Corners[i].Covariance(3,3) = 0.5F*(LinesHolder->Lines[Current->Index].Covariance(1,1)+LinesHolder->Lines[Current->Index+1].Covariance(1,1));
			CornersHolder->Corners[i].Covariance *= 1e3F;
			*/



			Current = Current->Next;
		}
	}
	
	LinkedListDeallocate(Head);
	return CornersHolder;
}