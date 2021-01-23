#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include "skel_solver.h"
#include "math_util.h"


SkelSolver::SkelSolver(const SkelType& _type, const std::string& modelPath)
	:SkelDriver(_type, modelPath) {
	const SkelDef& def = GetSkelDef(m_type);
	m_boneShapeBlend.resize(3 * (def.jointSize - 1), def.shapeSize);
	for (int jIdx = 1; jIdx < def.jointSize; jIdx++)
		m_boneShapeBlend.middleRows(3 * (jIdx - 1), 3) = m_jShapeBlend.middleRows(3 * jIdx, 3)
		- m_jShapeBlend.middleRows(3 * def.parent[jIdx], 3);
}


void SkelSolver::AlignRT(const Term& term, SkelParam& param) const
{
	// align root affine
	param.GetTrans() = term.j3dTarget.col(0).head<3>() - m_joints.col(0);
	auto CalcAxes = [](const Eigen::Vector3f& xAxis, const Eigen::Vector3f& yAxis) {
		Eigen::Matrix3f axes;
		axes.col(0) = xAxis.normalized();
		axes.col(2) = xAxis.cross(yAxis).normalized();
		axes.col(1) = axes.col(2).cross(axes.col(0)).normalized();
		return axes;
	};

	const Eigen::Matrix3f mat = CalcAxes(
		term.j3dTarget.col(2).head<3>() - term.j3dTarget.col(1).head<3>(),
		term.j3dTarget.col(3).head<3>() - term.j3dTarget.col(1).head<3>())
		* (CalcAxes(m_joints.col(2) - m_joints.col(1), m_joints.col(3) - m_joints.col(1)).inverse());
	const Eigen::AngleAxisf angleAxis(mat);
	param.GetPose().head(3) = angleAxis.angle() * angleAxis.axis();
}


void SkelSolver::SolvePose(const Term& term, SkelParam& param, const int& maxIterTime, const bool& hierarchy, const float& updateThresh)
{
	const SkelDef& def = GetSkelDef(m_type);

	const Eigen::Matrix3Xf jBlend = CalcJBlend(param);
	const int hierSize = def.hierarchyMap.maxCoeff();
	int hier = hierarchy ? 0 : hierSize;
	for (int jCut = 0; hier <= hierSize; hier++) {
		while (jCut < def.jointSize && def.hierarchyMap[jCut] <= hier)
			jCut++;
		for (int iterTime = 0; iterTime < maxIterTime; iterTime++) {
			// calc status
			const Eigen::Matrix4Xf nodeWarps = CalcNodeWarps(param, jBlend.leftCols(jCut));
			const Eigen::Matrix4Xf chainWarps = CalcChainWarps(nodeWarps);
			const Eigen::Matrix3Xf jFinal = CalcJFinal(chainWarps);
			Eigen::MatrixXf jointJacobi = Eigen::MatrixXf::Zero(3 * jCut, 3 + 3 * jCut);
			Eigen::MatrixXf ATA = Eigen::MatrixXf::Zero(3 + 3 * jCut, 3 + 3 * jCut);
			Eigen::VectorXf ATb = Eigen::VectorXf::Zero(3 + 3 * jCut);

			Eigen::MatrixXf nodeWarpsJacobi(9, 3 * jCut);
			for (int jIdx = 0; jIdx < jCut; jIdx++)
				nodeWarpsJacobi.middleCols(3 * jIdx, 3) = MathUtil::RodriguesJacobi<float>(param.GetPose().segment<3>(3 * jIdx)).transpose();

			for (int djIdx = 0; djIdx < jCut; djIdx++) {
				jointJacobi.block<3, 3>(3 * djIdx, 0).setIdentity();
				for (int dAxis = 0; dAxis < 3; dAxis++) {
					Eigen::Matrix4Xf dChainWarps = Eigen::Matrix4Xf::Zero(4, 4 * jCut);
					Eigen::VectorXi valid = Eigen::VectorXi::Zero(jCut);
					valid[djIdx] = 1;
					dChainWarps.block<3, 3>(0, 4 * djIdx) = Eigen::Map<Eigen::Matrix3f>(nodeWarpsJacobi.col(3 * djIdx + dAxis).data());
					if (djIdx != 0)
						dChainWarps.middleCols(4 * djIdx, 4) = chainWarps.middleCols(4 * def.parent[djIdx], 4) * dChainWarps.middleCols(4 * djIdx, 4);

					for (int jIdx = djIdx + 1; jIdx < jCut; jIdx++) {
						const int prtIdx = def.parent[jIdx];
						valid[jIdx] = valid[prtIdx];
						if (valid[jIdx]) {
							dChainWarps.middleCols(4 * jIdx, 4) = dChainWarps.middleCols(4 * prtIdx, 4) * nodeWarps.middleCols(4 * jIdx, 4);
							jointJacobi.block<3, 1>(jIdx * 3, 3 + djIdx * 3 + dAxis) = dChainWarps.block<3, 1>(0, 4 * jIdx + 3);
						}
					}
				}
			}

			// calc terms
			if (term.wJ3d > FLT_EPSILON) {
				for (int jIdx = 0; jIdx < jCut; jIdx++) {
					if (term.j3dTarget(3, jIdx) > FLT_EPSILON) {
						const float w = term.wJ3d * term.j3dTarget(3, jIdx);
						auto jacobi = jointJacobi.middleRows(3 * jIdx, 3);
						ATA += w * jacobi.transpose() * jacobi;
						ATb += w * jacobi.transpose()*(term.j3dTarget.block<3, 1>(0, jIdx) - jFinal.col(jIdx));
					}
				}
			}

			if (term.wJ2d > FLT_EPSILON) {
				for (int view = 0; view < term.projs.cols() / 4; view++) {
					const auto j2dTarget = term.j2dTarget.middleCols(view*def.jointSize, def.jointSize);
					if ((j2dTarget.row(2).array() > FLT_EPSILON).count() > 0) {
						const auto proj = term.projs.middleCols(view * 4, 4);
						for (int jIdx = 0; jIdx < jCut; jIdx++) {
							if (j2dTarget(2, jIdx) > FLT_EPSILON) {
								const Eigen::Vector3f abc = proj * (jFinal.col(jIdx).homogeneous());
								Eigen::Matrix<float, 2, 3> projJacobi;
								projJacobi << 1.0f / abc.z(), 0.0f, -abc.x() / (abc.z()*abc.z()),
									0.0f, 1.0f / abc.z(), -abc.y() / (abc.z()*abc.z());
								projJacobi = projJacobi * proj.leftCols(3);

								const float w = term.wJ2d * j2dTarget(2, jIdx);
								auto jacobi = projJacobi * jointJacobi.middleRows(3 * jIdx, 3);
								ATA += w * jacobi.transpose() * jacobi;
								ATb += w * jacobi.transpose()*(j2dTarget.block<2, 1>(0, jIdx) - abc.hnormalized());
							}
						}
					}
				}
			}

			if (term.wTemporalTrans > FLT_EPSILON) {
				ATA.topLeftCorner(3, 3) += term.wTemporalTrans * Eigen::Matrix3f::Identity();
				ATb.head(3) += term.wTemporalTrans * (term.paramPrev.GetTrans() - param.GetTrans());
			}

			if (term.wTemporalPose > FLT_EPSILON) {
				ATA.bottomRightCorner(3 * jCut, 3 * jCut) += term.wTemporalPose * Eigen::MatrixXf::Identity(3 * jCut, 3 * jCut);
				ATb.tail(3 * jCut) += term.wTemporalPose * (term.paramPrev.GetPose().head(3 * jCut)
					- param.GetPose().head(3 * jCut));
			}

			if (term.wRegularPose > FLT_EPSILON) {
				ATA += term.wRegularPose * Eigen::MatrixXf::Identity(3 + 3 * jCut, 3 + 3 * jCut);;
			}

			const Eigen::VectorXf delta = ATA.ldlt().solve(ATb);
			param.GetTransPose().head(3 + 3 * jCut) += delta;

			// debug
			// printf("iter: %d, update: %f\n", iterTime, delta.norm());

			if (delta.norm() < updateThresh)
				break;
		}
	}
}


void SkelSolver::SolveShape(const Term& term, SkelParam& param, const int& maxIterTime, const float& updateThresh) const
{
	const SkelDef& def = GetSkelDef(m_type);

	for (int iterTime = 0; iterTime < maxIterTime; iterTime++) {
		// calc status
		Eigen::Matrix3Xf jBlend = CalcJBlend(param);
		Eigen::MatrixXf ATA = Eigen::MatrixXf::Zero(def.shapeSize, def.shapeSize);
		Eigen::VectorXf ATb = Eigen::VectorXf::Zero(def.shapeSize);

		if (term.wBone3d > FLT_EPSILON) {
			for (int jIdx = 1; jIdx < def.jointSize; jIdx++) {
				if (term.bone3dTarget(1, jIdx - 1) > FLT_EPSILON) {
					const float w = term.wBone3d * term.bone3dTarget(1, jIdx - 1);
					const int prtIdx = def.parent(jIdx);
					const Eigen::Vector3f dir = jBlend.col(jIdx) - jBlend.col(prtIdx);
					const auto jacobi = m_boneShapeBlend.middleRows(3 * (jIdx - 1), 3);
					ATA += w * jacobi.transpose() * jacobi;
					ATb += w * jacobi.transpose() *(term.bone3dTarget(0, jIdx - 1)*dir.normalized() - dir);
				}
			}
		}

		if (term.wJ3d > FLT_EPSILON || term.wJ2d > FLT_EPSILON) {
			const Eigen::Matrix4Xf chainWarps = CalcChainWarps(CalcNodeWarps(param, jBlend));
			const Eigen::Matrix3Xf jFinal = CalcJFinal(chainWarps);
			Eigen::MatrixXf jointJacobi = Eigen::MatrixXf::Zero(3 * def.jointSize, def.shapeSize);
			for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
				if (jIdx == 0)
					jointJacobi.middleRows(3 * jIdx, 3) = m_jShapeBlend.middleRows(3 * jIdx, 3);
				else {
					const int prtIdx = def.parent[jIdx];
					jointJacobi.middleRows(3 * jIdx, 3) = jointJacobi.middleRows(3 * prtIdx, 3)
						+ chainWarps.block<3, 3>(0, 4 * prtIdx) * (m_jShapeBlend.middleRows(3 * jIdx, 3) - m_jShapeBlend.middleRows(3 * prtIdx, 3));
				}
			}

			if (term.wJ3d > FLT_EPSILON) {
				for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
					if (term.j3dTarget(3, jIdx) > FLT_EPSILON) {
						const float w = term.wJ3d * term.j3dTarget(3, jIdx);
						auto jacobi = jointJacobi.middleRows(3 * jIdx, 3);
						ATA += w * jacobi.transpose() * jacobi;
						ATb += w * jacobi.transpose()*(term.j3dTarget.block<3, 1>(0, jIdx) - jFinal.col(jIdx));
					}
				}
			}

			if (term.wJ2d > FLT_EPSILON) {
				for (int view = 0; view < term.projs.cols() / 4; view++) {
					const auto j2dTarget = term.j2dTarget.middleCols(view*def.jointSize, def.jointSize);
					if ((j2dTarget.row(2).array() > FLT_EPSILON).count() > 0) {
						const auto proj = term.projs.middleCols(view * 4, 4);
						for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
							if (j2dTarget(2, jIdx) > FLT_EPSILON) {
								const Eigen::Vector3f abc = proj * (jFinal.col(jIdx).homogeneous());
								Eigen::Matrix<float, 2, 3> projJacobi;
								projJacobi << 1.0f / abc.z(), 0.0f, -abc.x() / (abc.z()*abc.z()),
									0.0f, 1.0f / abc.z(), -abc.y() / (abc.z()*abc.z());
								projJacobi = projJacobi * proj.leftCols(3);

								const float w = term.wJ2d * j2dTarget(2, jIdx);
								auto jacobi = projJacobi * jointJacobi.middleRows(3 * jIdx, 3);
								ATA += w * jacobi.transpose() * jacobi;
								ATb += w * jacobi.transpose()*(j2dTarget.block<2, 1>(0, jIdx) - abc.hnormalized());
							}
						}
					}
				}
			}
		}

		if (term.wTemporalShape > FLT_EPSILON) {

			ATA += term.wTemporalShape * Eigen::MatrixXf::Identity(def.shapeSize, def.shapeSize);
			ATb += term.wTemporalShape * (term.paramPrev.GetShape() - param.GetShape());
		}

		if (term.wSquareShape > FLT_EPSILON) {
			ATA += term.wSquareShape * Eigen::MatrixXf::Identity(def.shapeSize, def.shapeSize);
			ATb -= term.wSquareShape * param.GetShape();
		}

		if (term.wRegularShape > FLT_EPSILON) {
			ATA += term.wRegularShape * Eigen::MatrixXf::Identity(def.shapeSize, def.shapeSize);
		}

		const Eigen::VectorXf delta = ATA.ldlt().solve(ATb);
		param.GetShape() += delta;

		// debug
		// printf("iter: %d, update: %f\n", iterTime, delta.norm());

		if (delta.norm() < updateThresh)
			break;
	}
}