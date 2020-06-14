#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>
#include <memory>
#include "skel.h"
#include "camera.h"
#include "skel_solver.h"


class SkelUpdater
{
public:
	SkelUpdater(const SkelType& _type) { m_type = _type; }
	virtual void Update(const std::map<int, Eigen::Matrix3Xf>& skels2d, const Eigen::Matrix3Xf& projs) = 0;
	const std::map<int, Eigen::Matrix4Xf>& GetSkel3d() const { return m_skels; }

protected:
	SkelType m_type;
	std::map<int, Eigen::Matrix4Xf> m_skels;
};


class SkelTriangulateUpdater : public SkelUpdater
{
public:
	SkelTriangulateUpdater(const SkelType& type) :SkelUpdater(type) {}
	virtual void Update(const std::map<int, Eigen::Matrix3Xf>& skels2d, const Eigen::Matrix3Xf& projs) override;

protected:
	Eigen::Matrix4Xf TriangulatePerson(const Eigen::Matrix3Xf& skel2d, const Eigen::Matrix3Xf& projs);
	float m_triangulateThresh = 0.05f;
	int m_minTrackJCnt = 20;
};


class SkelFittingUpdater : public SkelTriangulateUpdater
{
public:
	SkelFittingUpdater(const SkelType& type,const std::string& modelPath)
		: SkelTriangulateUpdater(type), m_solver(type, modelPath) {}

	virtual void Update(const std::map<int, Eigen::Matrix3Xf>& skels2d, const Eigen::Matrix3Xf& projs) override;

private:
	struct SkelInfo : public SkelParam
	{
		SkelInfo() {}
		SkelInfo(const SkelType& type) :SkelParam(type) {
			boneLen.setZero(GetSkelDef(type).jointSize - 1);
			boneCnt.setZero(GetSkelDef(type).jointSize - 1);
			active = 0.f;
		}

		Eigen::VectorXf boneLen;
		Eigen::VectorXi boneCnt;
		float active;
		void PushPrevBones(const Eigen::Matrix4Xf& skel);
	};

	SkelSolver m_solver;
	std::map<int, SkelInfo> m_skelInfos;

	int m_boneCapacity = 30;
	float m_wBone3d = 1.f;
	float m_wSquareShape = 1e-3f;
	int m_shapeMaxIter = 5;

	float m_wRegularPose = 1e-4f;
	float m_wTemporalTrans = 1e-2f;
	float m_wTemporalPose = 1e-3f;
	float m_wJ2d = 1e-5f;
	float m_wJ3d = 1.f;
	int m_poseMaxIter = 5;

	float m_initActive = 0.9f;
	float m_activeRate = 0.5f;
};

