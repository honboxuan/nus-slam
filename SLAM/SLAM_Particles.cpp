#include "SLAM.h"

ParticleClass::ParticleClass() {
	X = 0;
	Y = 0;
	Theta = 0;
	Covariance = Eigen::Matrix3f::Identity();
	CornersAllocatedCount = 0;
	Weight = PARTICLEINITIALWEIGHT;
	TimesSelected = 0;
	CornersHolder = new CornersHolderClass;
}
ParticleClass::~ParticleClass() {
	delete CornersHolder;
}
ParticleClass::ParticleClass(const ParticleClass &Original) {
	X = Original.X;
	Y = Original.Y;
	Theta = Original.Theta;
	Covariance = Original.Covariance;
	CornersAllocatedCount = Original.CornersAllocatedCount;
	Weight = Original.Weight;
	TimesSelected = 0;
	CornersHolder = new CornersHolderClass;
	CornersHolder->Count = Original.CornersHolder->Count;
	CornersHolder->Corners = new CornerClass[CornersAllocatedCount];
	for (uint16_t i = 0; i < CornersHolder->Count; i++) {
		CornersHolder->Corners[i] = Original.CornersHolder->Corners[i];
	}
}
bool ParticleClass::StateUpdate(OdometryClass* Odometry) {
	if (Odometry != NULL) {
		//Noise
		std::random_device RD;
		std::default_random_engine Eng(RD());
		std::normal_distribution<float> TranslationDistribution(0.0F,2.5F);
		std::normal_distribution<float> RotationDistribution(0.0F,1.0F);

		float NoiseX = Odometry->X*TranslationDistribution(Eng);
		float NoiseY = Odometry->Y*TranslationDistribution(Eng);
		float NoiseTheta = Odometry->Theta*RotationDistribution(Eng);

		/*
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
		*/

		//Rotation before translation
		X += float((NoiseX+Odometry->X)*cos(Theta)-(NoiseY+Odometry->Y)*sin(Theta));
		Y += float((NoiseX+Odometry->X)*sin(Theta)+(NoiseY+Odometry->Y)*cos(Theta));
		Theta += Odometry->Theta;
		Theta += NoiseTheta;
		if (Theta > PI) {
			Theta -= 2*PI;
		}
		if (Theta < -PI) {
			Theta += 2*PI;
		}
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
	CornersHolder->Corners[CornersHolder->Count] = *Corner;
	CornersHolder->Count++;
}
void ParticleClass::CornersMemoryExpansion() {
	//Expand allocated space by another block
	CornersAllocatedCount += FEATURESBLOCKSIZE;
	CornerClass* BlockNew = new CornerClass[CornersAllocatedCount];
	for (uint16_t i = 0; i < CornersHolder->Count; i++) {
		BlockNew[i] = CornersHolder->Corners[i];
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
	Known->Covariance.block<2,2>(2,2) = ((Eigen::Matrix4f::Identity()-K)*Known->Covariance).block<2,2>(2,2);
	//Known->Covariance(3,3) = ((Eigen::Matrix4f::Identity()-K)*Known->Covariance)(3,3);
	
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
		for (uint8_t j = 0; j < 3; j++) {
			uint16_t Compare = Distribution(Eng);
			if (Particles[Index]->Weight < Particles[Compare]->Weight) {
				Index = Compare;
			}
		}



		if (Particles[Index]->TimesSelected == 0) {
			Selected[i] = Particles[Index];
		} else {
			//Create deep copy
			Selected[i] = new ParticleClass(*Particles[Index]);
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