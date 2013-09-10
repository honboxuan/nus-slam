#include "SLAM.h"

OdometryClass::OdometryClass() {
	X = 0;
	Y = 0;
	Theta = 0;
}

/*The normal scan matching algorithms are used instead of these
float ScanMatchingCornersMahalanobis(CornerClass* CornerA,CornerClass* CornerB) {
	Eigen::Vector3f Innovation;
	Innovation(0) = CornerB->X-CornerA->X;
	Innovation(1) = CornerB->Y-CornerA->Y;
	Innovation(2) = CornerB->Angle-CornerA->Angle;
	Eigen::Matrix3f S = CornerA->Covariance.block<3,3>(0,0)+CornerB->Covariance.block<3,3>(0,0);
	return sqrt(abs(*(Innovation.transpose()*S.inverse()*Innovation).data()));
}
HypothesisItem* ScanMatchingCornersAssociate(CornersHolderClass* CornersA,CornersHolderClass* CornersB,float Threshold) {
	//CornersA should be detected corners, CornersB should be known corners
	bool** Table = new bool*[CornersA->Count];
	for (uint16_t i = 0; i < CornersA->Count; i++) {
		Table[i] = new bool[CornersB->Count];
	}
	//Fill individual compatibility matrix
	for (uint16_t j = 0; j < CornersB->Count; j++) {
		for (uint16_t i = 0; i < CornersA->Count; i++) {
			if (ScanMatchingCornersMahalanobis(&CornersA->Corners[i],&CornersB->Corners[j]) < Threshold) {
				Table[i][j] = true;
			} else {
				Table[i][j] = false;
			}
		}
	}
	//First hypothesis (no associations)
	//Note that 0 in hypothesis means no association. Association with first CornerB should give 1
	uint16_t* Hypothesis = new uint16_t[CornersA->Count];
	for (uint16_t i = 0; i < CornersA->Count; i++) {
		Hypothesis[i] = 0;
	}
	uint16_t AssociationCount = 0;
	bool Invalid = false;
	HypothesisItem* Head = new HypothesisItem;
	Head->Hypothesis = new uint16_t[CornersA->Count];
	for (uint16_t i = 0; i < CornersA->Count; i++) {
		Head->Hypothesis[i] = Hypothesis[i];
	}
	HypothesisItem* Tail;
	Tail = Head;
	while (HypothesisPlus(Hypothesis,CornersA->Count-1,CornersB->Count,Table)) {
		AssociationCount = 0;
		Invalid = false;
		for (uint16_t i = 0; i < CornersA->Count; i++) {
			//Check if zero
			if (Hypothesis[i] != 0) {
				//Check for repeats (greedy algorithm)
				for (uint16_t j = i+1; j < CornersA->Count; j++) {
					if (Hypothesis[j] == Hypothesis[i]) {
						Invalid = true;
						break;
					}
				}
				if (Invalid == true) {
					break;
				} else {
					AssociationCount++;
				}
			}
		}
		//The more the merrier here
		if (AssociationCount > Head->AssociationCount) {
			LinkedListDeallocate(Head->Next);
			Head->Next = NULL;
			Tail = Head;
			Head->AssociationCount = AssociationCount;
			for (uint16_t i = 0; i < CornersA->Count; i++) {
				Head->Hypothesis[i] = Hypothesis[i];
			}
		} else if (AssociationCount == Head->AssociationCount) {
			Tail->Next = new HypothesisItem;
			Tail = Tail->Next;
			Tail->AssociationCount = AssociationCount;
			Tail->Hypothesis = new uint16_t[CornersA->Count];
			for (uint16_t i = 0; i < CornersA->Count; i++) {
				Tail->Hypothesis[i] = Hypothesis[i];
			}
		}
	}
	//Calculate joint compatibility for candidates
	//If compatibility is greater than current winner, replace current hypothesis and compatibility winner
	HypothesisItem* WinnerItem = NULL;
	float WinnerCompatibility = 0;
	if (Head->AssociationCount != 0 && Head->Next != NULL) {
		HypothesisItem* Current = Head;
		while (Current != NULL) {
			Eigen::VectorXf Innovation(3*Current->AssociationCount);
			Eigen::MatrixXf Covariance;
			Covariance = Eigen::MatrixXf::Zero(3*Current->AssociationCount,3*Current->AssociationCount);
			uint32_t Index = 0;
			for (uint16_t i = 0; i < CornersA->Count; i++) {
				if (Current->Hypothesis[i] != 0) {
					Innovation(3*Index) = CornersA->Corners[i].X-CornersB->Corners[Current->Hypothesis[i]-1].X;
					Innovation(3*Index+1) = CornersA->Corners[i].Y-CornersB->Corners[Current->Hypothesis[i]-1].Y;
					Innovation(3*Index+2) = CornersA->Corners[i].Angle-CornersB->Corners[Current->Hypothesis[i]-1].Angle;
					Covariance.block<3,3>(3*Index,3*Index) = CornersA->Corners[i].Covariance.block<3,3>(0,0)+CornersB->Corners[Current->Hypothesis[i]-1].Covariance.block<3,3>(0,0);
					Index++;
				}
			}
			float JointCompatibility = sqrt(abs(*(Innovation.transpose()*Covariance.inverse()*Innovation).data()));
			if (WinnerCompatibility == 0 || JointCompatibility < WinnerCompatibility) {
				WinnerCompatibility = JointCompatibility;
				WinnerItem = Current;
			}
			Current = Current->Next;
		}
	} else {
		WinnerItem = Head;
	}
	//All items in linked list will be deallocated, have to create new item
	HypothesisItem* Result = new HypothesisItem;
	Result->AssociationCount = WinnerItem->AssociationCount;
	Result->Hypothesis = new uint16_t[CornersA->Count];
	for (uint16_t i = 0; i < CornersA->Count; i++) {
		Result->Hypothesis[i] = WinnerItem->Hypothesis[i];
	}
	//Deallocate
	for (uint32_t i = 0; i < CornersA->Count; i++) {
		delete[] Table[i];
	}
	delete[] Table;
	delete[] Hypothesis;
	LinkedListDeallocate(Head);

	return Result;
}*/

OdometryClass* ScanMatching(PointClass* PointsA,PointClass* PointsB,CornersHolderClass* CornersA,CornersHolderClass* CornersB,HypothesisItem* Hypothesis) {
	OdometryClass* Result = new OdometryClass;
	float XBarA = 0, YBarA = 0, XBarB = 0, YBarB = 0;
	uint16_t TotalCount = 0, NeighboursCount = 0;
	for (int i = 0; i < CornersA->Count; i++) {
		if (Hypothesis->Hypothesis[i] != 0) {
			XBarA += CornersA->Corners[i].X;
			YBarA += CornersA->Corners[i].Y;
			XBarB += CornersB->Corners[Hypothesis->Hypothesis[i]-1].X;
			YBarB += CornersB->Corners[Hypothesis->Hypothesis[i]-1].Y;
			//Attempt to use points in neighbourhood
			/*for (int j = -2; j <= 2; j++) {
				XBarA += PointsA[CornersA->Corners[i].PointIndex+j].X;
				YBarA += PointsA[CornersA->Corners[i].PointIndex+j].Y;
				XBarB += PointsB[CornersB->Corners[Hypothesis->Hypothesis[i]-1].PointIndex+j].X;
				YBarB += PointsB[CornersB->Corners[Hypothesis->Hypothesis[i]-1].PointIndex+j].Y;
				NeighboursCount++;
			}*/
		}
	}
	TotalCount = Hypothesis->AssociationCount+NeighboursCount;
	XBarA /= TotalCount;
	YBarA /= TotalCount;
	XBarB /= TotalCount;
	YBarB /= TotalCount;

	float Numerator = 0, Denominator = 0;
	for (int i = 0; i < CornersA->Count; i++) {
		if (Hypothesis->Hypothesis[i] != 0) {
			//Procrustes analysis, 2D rotation
			float x = CornersB->Corners[Hypothesis->Hypothesis[i]-1].X-XBarB;
			float y = CornersB->Corners[Hypothesis->Hypothesis[i]-1].Y-YBarB;
			float w = CornersA->Corners[i].X-XBarA;
			float z = CornersA->Corners[i].Y-YBarA;
			Numerator += w*y-z*x;
			Denominator += w*x+z*y;
			/*for (int j = -4; j <= 4; j++) {
				x = PointsB[CornersB->Corners[Hypothesis->Hypothesis[i]-1].PointIndex+j].X-XBarB;
				y = PointsB[CornersB->Corners[Hypothesis->Hypothesis[i]-1].PointIndex+j].Y-YBarB;
				w = PointsA[CornersA->Corners[i].PointIndex+j].X-XBarA;
				z = PointsA[CornersA->Corners[i].PointIndex+j].Y-YBarA;
				Numerator += w*y-z*x;
				Denominator += w*x+z*y;
			}*/
		}
	}
	Result->Theta = atan2(Numerator,Denominator);

	Result->X = XBarB-(XBarA*cos(Result->Theta)-YBarA*sin(Result->Theta));
	Result->Y = YBarB-(XBarA*sin(Result->Theta)+YBarA*cos(Result->Theta));

	Result->X += XBarB*cos(Result->Theta)+YBarB*sin(Result->Theta)-XBarA;
	Result->Y += -XBarB*sin(Result->Theta)+YBarB*cos(Result->Theta)-YBarA;

	Result->X /= 2;
	Result->Y /= 2;

	return Result;
}

OdometryClass* FindTranslation(PointClass* PointsA,PointClass* PointsB,CornersHolderClass* CornersA,CornersHolderClass* CornersB,HypothesisItem* Hypothesis) {
	OdometryClass* Result = new OdometryClass;
	float XBarA = 0, YBarA = 0, XBarB = 0, YBarB = 0;
	for (int i = 0; i < CornersA->Count; i++) {
		if (Hypothesis->Hypothesis[i] != 0) {
			XBarA += CornersA->Corners[i].X;
			YBarA += CornersA->Corners[i].Y;
			XBarB += CornersB->Corners[Hypothesis->Hypothesis[i]-1].X;
			YBarB += CornersB->Corners[Hypothesis->Hypothesis[i]-1].Y;
		}
	}
	XBarA /= Hypothesis->AssociationCount;
	YBarA /= Hypothesis->AssociationCount;
	XBarB /= Hypothesis->AssociationCount;
	YBarB /= Hypothesis->AssociationCount;

	Result->X = XBarB-XBarA;
	Result->Y = YBarB-YBarA;

	return Result;
}