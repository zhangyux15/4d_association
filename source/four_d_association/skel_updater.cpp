#include <iostream>
#include "skel_updater.h"
#include "color_util.h"
#include "math_util.h"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>


Eigen::Matrix4Xf SkelTriangulateUpdater::TriangulatePerson(const Eigen::Matrix3Xf& skel2d, const Eigen::Matrix3Xf& projs)
{
	const SkelDef& def = GetSkelDef(m_type);
	Eigen::Matrix4Xf skel = Eigen::Matrix4Xf::Zero(4, def.jointSize);

	Triangulator triangulator;
	triangulator.projs = projs;
	triangulator.points.resize(3, projs.cols() / 4);
	for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
		for (int view = 0; view < projs.cols() / 4; view++) {
			const auto& skel = skel2d.middleCols(view*def.jointSize, def.jointSize);
			triangulator.points.col(view) = skel.col(jIdx);
		}
		triangulator.Solve();
		if (triangulator.loss < m_triangulateThresh)
			skel.col(jIdx) = triangulator.pos.homogeneous();
	}
	return skel;
}


void SkelTriangulateUpdater::Update(const std::map<int, Eigen::Matrix3Xf>& skels2d, const Eigen::Matrix3Xf& projs)
{
	const SkelDef& def = GetSkelDef(m_type);
	
	const int prevCnt = int(m_skels.size());
	auto skelIter = m_skels.begin();
	int pIdx = 0;
	for (auto corrIter = skels2d.begin(); corrIter != skels2d.end(); corrIter++, pIdx++) {
		const Eigen::Matrix4Xf skel = TriangulatePerson(corrIter->second, projs);
		const bool active = (skel.row(3).array() >= FLT_EPSILON).count() >= m_minTrackJCnt;
		if (pIdx < prevCnt) {
			if (active) {
				skelIter->second = skel;
				skelIter++;
			}
			else
				skelIter = m_skels.erase(skelIter);
		}
		else if (active)
			m_skels.insert(std::make_pair(corrIter->first, skel));
	}
}


void SkelFittingUpdater::SkelInfo::PushPrevBones(const Eigen::Matrix4Xf& skel)
{
	const SkelDef& def = GetSkelDef(type);
	for (int jIdx = 1; jIdx < def.jointSize; jIdx++) {
		const int prtIdx = def.parent(jIdx);
		if (skel(3, jIdx) > FLT_EPSILON && skel(3, prtIdx) > FLT_EPSILON) {
			const float len = (skel.col(jIdx).head(3) - skel.col(prtIdx).head(3)).norm();
			boneLen[jIdx - 1] = (float(boneCnt[jIdx - 1]) *  boneLen[jIdx - 1] + len) / float(boneCnt[jIdx - 1] + 1);
			boneCnt[jIdx - 1]++;
		}
	}
}


void SkelFittingUpdater::Update(const std::map<int, Eigen::Matrix3Xf>& skels2d, const Eigen::Matrix3Xf& projs)
{
	const SkelDef& def = GetSkelDef(m_type);
	const int prevCnt = int(m_skels.size());
	// update tracked person
	auto skelIter = m_skels.begin();
	auto infoIter = m_skelInfos.begin();
	int pIdx = 0;
	for (auto corrIter = skels2d.begin(); corrIter != skels2d.end(); corrIter++, pIdx++) {
		if (pIdx < prevCnt) {
			SkelInfo& info = infoIter->second;
			const float active = std::min(info.active + m_activeRate * (2.f * MathUtil::Welsch(
				float(m_minTrackJCnt), float((corrIter->second.row(2).array() > FLT_EPSILON).count())) - 1.f), 1.f);
			if (active > 0.f) {
				const Eigen::Matrix4Xf skel = TriangulatePerson(corrIter->second, projs);
				if (info.boneCnt.minCoeff() < m_boneCapacity) {
					// align shape
					info.PushPrevBones(skel);
					SkelSolver::Term shapeTerm;
					shapeTerm.bone3dTarget = info.boneLen.transpose().colwise().homogeneous();
					shapeTerm.wBone3d = m_wBone3d;
					shapeTerm.wSquareShape = m_wSquareShape;
					m_solver.SolveShape(shapeTerm, info, m_shapeMaxIter);
				}

				// align pose
				SkelSolver::Term poseTerm;
				poseTerm.wJ2d = m_wJ2d;
				poseTerm.projs = projs;
				poseTerm.j2dTarget = corrIter->second;
				poseTerm.wRegularPose = m_wRegularPose;
				poseTerm.paramPrev = info;
				poseTerm.wTemporalTrans = m_wTemporalTrans;
				poseTerm.wTemporalPose = m_wTemporalPose;
				m_solver.SolvePose(poseTerm, info, m_poseMaxIter, true);
				skelIter->second.topRows(3) = m_solver.CalcJFinal(info);

				// update active
				info.active = active;
				skelIter++;
				infoIter++;
			}
			else {
				skelIter = m_skels.erase(skelIter);
				infoIter = m_skelInfos.erase(infoIter);
			}
		}
		else {
			Eigen::Matrix4Xf skel = TriangulatePerson(corrIter->second, projs);
			// alloc new person
			if (skel.row(3).minCoeff() > FLT_EPSILON) {
				SkelInfo info(m_type);

				// align shape
				info.PushPrevBones(skel);
				SkelSolver::Term shapeTerm;
				shapeTerm.wBone3d = m_wBone3d;
				shapeTerm.bone3dTarget = info.boneLen.transpose().colwise().homogeneous();
				shapeTerm.wSquareShape = m_wSquareShape;
				m_solver.SolveShape(shapeTerm, info, m_shapeMaxIter);

				// align pose
				SkelSolver::Term poseTerm;
				poseTerm.wJ3d = m_wJ3d;
				poseTerm.j3dTarget = skel;
				poseTerm.wRegularPose = m_wRegularPose;
				m_solver.SolvePose(poseTerm, info, m_poseMaxIter, true);
				
				skel.topRows(3) = m_solver.CalcJFinal(info);
				info.active = m_initActive;
				skel.row(3).setConstant(info.active);

				m_skels.insert(std::make_pair(corrIter->first, skel));
				m_skelInfos.insert(std::make_pair(corrIter->first, info));
			}
		}
	}
}

