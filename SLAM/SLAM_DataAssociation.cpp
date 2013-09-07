#include "SLAM.h"

//Association should only be in observable vicinity
//Use R-trees or variant for spatial indexing

HypothesisItem::HypothesisItem() {
	Next = NULL;
	Hypothesis = NULL;
	AssociationCount = 0;
}
HypothesisItem::~HypothesisItem() {
	delete[] Hypothesis;
}
bool LinkedListDeallocate(HypothesisItem* Head) {
	if (Head == NULL) {
		return false;
	}
	while (Head != NULL) {
		HypothesisItem* Current = Head;
		Head = Head->Next;
		delete Current;
	}
	return true;
}
bool HypothesisPlus(uint16_t* Hypothesis,uint16_t TargetIndex,uint16_t Count,bool** Table) {
	//Try to increase current column value
	for (uint16_t i = Hypothesis[TargetIndex]; i < Count; i++) {
		if (Table[TargetIndex][i]) {
			Hypothesis[TargetIndex] = i+1; //Note +1
			return true;
		}
	}
	if (TargetIndex == 0) {
		//First column is maxed, no new hypotheses
		return false;
	} else {
		//Column is maxed, go to minimum (which is 0 for no association)
		Hypothesis[TargetIndex] = 0;
		//Carry over
		return HypothesisPlus(Hypothesis,TargetIndex-1,Count,Table);
	}
}
float CornersMahalanobis(CornerClass* CornerA,CornerClass* CornerB) {
	Eigen::Vector4f Innovation;
	Innovation(0) = CornerB->X-CornerA->X;
	Innovation(1) = CornerB->Y-CornerA->Y;
	Innovation(2) = CornerB->Angle-CornerA->Angle;
	Innovation(3) = CornerB->Heading-CornerA->Heading;
	Eigen::Matrix4f S = CornerA->Covariance+CornerB->Covariance;
	return sqrt(abs(*(Innovation.transpose()*S.inverse()*Innovation).data()));
}
HypothesisItem* CornersAssociate(CornersHolderClass* CornersA,CornersHolderClass* CornersB,float Threshold) {
	//CornersA should be detected corners, CornersB should be known corners
	bool** Table = new bool*[CornersA->Count];
	for (uint16_t i = 0; i < CornersA->Count; i++) {
		Table[i] = new bool[CornersB->Count];
	}
	//Fill individual compatibility matrix
	for (uint16_t j = 0; j < CornersB->Count; j++) {
		for (uint16_t i = 0; i < CornersA->Count; i++) {
			if (CornersMahalanobis(&CornersA->Corners[i],&CornersB->Corners[j]) < Threshold) {
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
			Eigen::VectorXf Innovation(4*Current->AssociationCount);
			Eigen::MatrixXf Covariance;
			Covariance = Eigen::MatrixXf::Zero(4*Current->AssociationCount,4*Current->AssociationCount);
			uint32_t Index = 0;
			for (uint16_t i = 0; i < CornersA->Count; i++) {
				if (Current->Hypothesis[i] != 0) {
					Innovation(4*Index) = CornersA->Corners[i].X-CornersB->Corners[Current->Hypothesis[i]-1].X;
					Innovation(4*Index+1) = CornersA->Corners[i].Y-CornersB->Corners[Current->Hypothesis[i]-1].Y;
					Innovation(4*Index+2) = CornersA->Corners[i].Angle-CornersB->Corners[Current->Hypothesis[i]-1].Angle;
					Innovation(4*Index+3) = CornersA->Corners[i].Heading-CornersB->Corners[Current->Hypothesis[i]-1].Heading;
					Covariance.block<4,4>(4*Index,4*Index) = CornersA->Corners[i].Covariance+CornersB->Corners[Current->Hypothesis[i]-1].Covariance;
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
}