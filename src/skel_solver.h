#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>
#include <memory>
#include "skel_driver.h"


class SkelSolver : public SkelDriver
{
public:
	SkelSolver(const SkelType& _type, const std::string& modelPath);
	virtual ~SkelSolver() = default;

	struct Term
	{
		// joint 3d
		float wJ3d = 0.f;
		Eigen::Matrix4Xf j3dTarget;

		// bone 3d
		float wBone3d = 0.f;
		Eigen::Matrix2Xf bone3dTarget;

		// joint 2d
		float wJ2d = 0.f;
		Eigen::Matrix3Xf projs;
		Eigen::Matrix3Xf j2dTarget;

		// temporal
		float wTemporalTrans = 0.f;
		float wTemporalPose = 0.f;
		float wTemporalShape = 0.f;
		SkelParam paramPrev;

		// regular
		float wRegularPose = 0.f;
		float wRegularShape = 0.f;
	
		float wSquareShape = 0.f;
	};

	void AlignRT(const Term& term, SkelParam& param) const;
	void SolvePose(const Term& term, SkelParam& param, const int& maxIterTime, const bool& hierarchy = false, const float& updateThresh = 1e-4f);
	void SolveShape(const Term& term, SkelParam& param, const int& maxIterTime, const float& updateThresh = 1e-4f) const;

private:
	Eigen::MatrixXf m_boneShapeBlend;
};

