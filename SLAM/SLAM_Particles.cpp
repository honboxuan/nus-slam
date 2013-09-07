#include "SLAM.h"

ParticleClass::ParticleClass() {
	X = 0;
	Y = 0;
	Theta = 0;
	Weight = PARTICLEINITIALWEIGHT;
	TimesSelected = 0;
	Covariance = Eigen::Matrix3f::Identity();
	CornersAllocatedCount = 0;
	CornersHolder = new CornersHolderClass;
}
ParticleClass::~ParticleClass() {
	delete CornersHolder;
}
bool ParticleClass::StateUpdate(OdometryClass* Odometry) {
	if (Odometry != NULL) {
		//Noise
		std::random_device RD;
		std::default_random_engine Eng(RD());
		std::normal_distribution<float> TranslationDistribution(0.0F,2.0F);
		std::normal_distribution<float> RotationDistribution(0.0F,2.0F);

		float NoiseX = Odometry->X*TranslationDistribution(Eng);
		float NoiseY = Odometry->Y*TranslationDistribution(Eng);
		float NoiseTheta = Odometry->Theta*RotationDistribution(Eng);

		//Rotate odometry values, translate half
		X += float(0.5*((NoiseX+Odometry->X)*cos(Theta)-(NoiseY+Odometry->Y)*sin(Theta)));
		Y += float(0.5*((NoiseX+Odometry->X)*sin(Theta)+(NoiseY+Odometry->Y)*cos(Theta)));
		//Rotate
		Theta += Odometry->Theta;
		Theta += NoiseTheta;
		if (Theta > PI) {
			Theta -= 2*PI;
		}
		if (Theta < -PI) {
			Theta += 2*PI;
		}
		//Complete translation
		X += float(0.5*((NoiseX+Odometry->X)*cos(Theta)-(NoiseY+Odometry->Y)*sin(Theta)));
		Y += float(0.5*((NoiseX+Odometry->X)*sin(Theta)+(NoiseY+Odometry->Y)*cos(Theta)));
	} else {
		return false;
	}
	return true;
}
CornersHolderClass* ParticleClass::Loc2Glo(CornersHolderClass* Local) {
	CornersHolderClass* Global = new CornersHolderClass;
	Global->Count = Local->Count;
	Global->Corners = new CornerClass[Local->Count];
	float Sin_Theta = sin(Theta);
	float Cos_Theta = cos(Theta);
	for (uint16_t i = 0; i < Global->Count; i++) {
		Global->Corners[i].X = Local->Corners[i].X*Cos_Theta-Local->Corners[i].Y*Sin_Theta+X;
		Global->Corners[i].Y = Local->Corners[i].X*Sin_Theta+Local->Corners[i].Y*Cos_Theta+Y;
		Global->Corners[i].Angle = Local->Corners[i].Angle;
		Global->Corners[i].Heading = Local->Corners[i].Heading+Theta;
		if (Global->Corners[i].Heading > PI) {
			Global->Corners[i].Heading -= 2*PI;
		}
		if (Global->Corners[i].Heading < -PI) {
			Global->Corners[i].Heading += 2*PI;
		}
		Global->Corners[i].Covariance = Local->Corners[i].Covariance;
	}
	return Global;
}
void ParticleClass::AddCorner(CornerClass* Corner) {
	//Add corners, call CornersMemoryExpansion if necessary
	if (CornersHolder->Count == CornersAllocatedCount) {
		CornersMemoryExpansion();
	}
	CornersHolder->Corners[CornersHolder->Count].X = Corner->X;
	CornersHolder->Corners[CornersHolder->Count].Y = Corner->Y;
	CornersHolder->Corners[CornersHolder->Count].Angle = Corner->Angle;
	CornersHolder->Corners[CornersHolder->Count].Heading = Corner->Heading;
	CornersHolder->Corners[CornersHolder->Count].Covariance = Corner->Covariance;
	CornersHolder->Count++;
}
void ParticleClass::CornersMemoryExpansion() {
	//Expand allocated space by another block
	CornersAllocatedCount += FEATURESBLOCKSIZE;
	CornerClass* BlockNew = new CornerClass[CornersAllocatedCount];
	for (uint16_t i = 0; i < CornersHolder->Count; i++) {
		BlockNew[i].X = CornersHolder->Corners[i].X;
		BlockNew[i].Y = CornersHolder->Corners[i].Y;
		BlockNew[i].Angle = CornersHolder->Corners[i].Angle;
		BlockNew[i].Heading = CornersHolder->Corners[i].Heading;
		BlockNew[i].Covariance = CornersHolder->Corners[i].Covariance;
	}
	delete[] CornersHolder->Corners;
	CornersHolder->Corners = BlockNew;
}
double KalmanUpdate(CornerClass* Known,CornerClass* Detected) {
	Eigen::Matrix4f Q = Known->Covariance+Detected->Covariance;
	Eigen::Matrix4f K = Known->Covariance*Q.inverse();
	Eigen::Vector4f Innovation;
	Innovation(0) = Detected->X-Known->X;
	Innovation(1) = Detected->Y-Known->Y;
	Innovation(2) = Detected->Angle-Known->Angle;
	Innovation(3) = Detected->Heading-Known->Heading;
	Eigen::Vector4f Delta = K*Innovation;
	Known->X += *Delta.data();
	Known->Y += *(Delta.data()+1);
	Known->Angle += *(Delta.data()+2);
	Known->Heading += *(Delta.data()+3);
	
	
	//Features are not exactly static considering the poor data
	//Known->Covariance = (Eigen::Matrix4f::Identity()-K)*Known->Covariance; 

	
	
	//Importance weight
	//return pow((2*PI*Q).determinant(),-0.5F)*exp(-0.5F*Innovation.transpose()*Q.inverse()*Innovation);
	float p1 = pow((2*PI*Q).determinant(),-0.5F);
	float p2 = -0.5F*Innovation.transpose()*Q.inverse()*Innovation;
	double Weight = p1*exp(0.001F*p2);
	return Weight;
}
ParticleClass** ParticleResample(ParticleClass** Particles) {
	ParticleClass** Selected = new ParticleClass*[PARTICLECOUNT];
	/*double WeightTotal = 0;
	for (uint16_t i = 0; i < PARTICLECOUNT; i++) {
		WeightTotal += Particles[i]->Weight;
	}*/
	std::random_device RD;
	std::default_random_engine Eng(RD());
	//std::uniform_real_distribution<float> Distribution(0.0F,1.0F);
	std::uniform_int_distribution<uint16_t> Distribution(0,PARTICLECOUNT-1);
	for (uint16_t i = 0; i < PARTICLECOUNT; i++) {
		
		
		//---------Systematic Weighted Sampling with Replacement---------
		/*double TheClaw = WeightTotal*Distribution(Eng);
		uint16_t Index = 0;
		while (TheClaw > 0 && Index <= PARTICLECOUNT) {
			TheClaw -= Particles[Index]->Weight;
			Index++;
		}
		Index -= 1;*/


		//---------Tournament Selection with Replacement---------
		uint16_t Index = Distribution(Eng);
		for (uint8_t j = 0; j < 10; j++) {
			uint16_t Compare = Distribution(Eng);
			if (Particles[Index]->Weight < Particles[Compare]->Weight) {
				Index = Compare;
			}
		}



		if (Particles[Index]->TimesSelected == 0) {
			Selected[i] = Particles[Index];
		} else {
			//Create a copy
			//This is extremely bad
			Selected[i] = new ParticleClass;
			Selected[i]->X = Particles[Index]->X;
			Selected[i]->Y = Particles[Index]->Y;
			Selected[i]->Theta = Particles[Index]->Theta;
			Selected[i]->Covariance = Particles[Index]->Covariance;
			Selected[i]->CornersAllocatedCount = Particles[Index]->CornersAllocatedCount;
			Selected[i]->Weight = Particles[Index]->Weight;

			if (Selected[i]->CornersAllocatedCount != 0) {
				Selected[i]->CornersHolder->Corners = new CornerClass[Selected[i]->CornersAllocatedCount];
				Selected[i]->CornersHolder->Count = Particles[Index]->CornersHolder->Count;
				for (uint16_t j = 0; j < Selected[i]->CornersHolder->Count; j++) {
					Selected[i]->CornersHolder->Corners[j].X = Particles[Index]->CornersHolder->Corners[j].X;
					Selected[i]->CornersHolder->Corners[j].Y = Particles[Index]->CornersHolder->Corners[j].Y;
					Selected[i]->CornersHolder->Corners[j].Angle = Particles[Index]->CornersHolder->Corners[j].Angle;
					Selected[i]->CornersHolder->Corners[j].Heading = Particles[Index]->CornersHolder->Corners[j].Heading;
					Selected[i]->CornersHolder->Corners[j].Covariance = Particles[Index]->CornersHolder->Corners[j].Covariance;
				}
			}
		}
		Particles[Index]->TimesSelected++;
	}
	for (uint16_t i = 0; i < PARTICLECOUNT; i++) {
		//Deallocate those not selected
		if (Particles[i]->TimesSelected == 0) {
			delete Particles[i];
		}
	}
	delete[] Particles;
	double WeightTop = 0;
	for (uint16_t i = 0; i < PARTICLECOUNT; i++) {
		//Selected[i]->Weight = PARTICLEINITIALWEIGHT;
		if (Selected[i]->Weight > WeightTop) {
			WeightTop = Selected[i]->Weight;
		}
		Selected[i]->TimesSelected = 0;
	}
	for (uint16_t i = 0; i < PARTICLECOUNT; i++) {
		Selected[i]->Weight *= PARTICLEINITIALWEIGHT/WeightTop;
	}
	return Selected;
}