#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>
#include <memory>
#include "skel.h"


std::vector<std::map<int, Eigen::Matrix4Xf>> ParseSkels(const std::string& filename);
void SerializeSkels(const std::vector<std::map<int, Eigen::Matrix4Xf>>& skels, const std::string& filename);


struct SkelParam
{
	SkelType type;
	Eigen::VectorXf data;

	SkelParam() { type = SKEL_TYPE_NONE; }
	SkelParam(const SkelType& _type) {
		type = _type;
		data.setZero(3 + GetSkelDef(type).jointSize * 3 + GetSkelDef(type).shapeSize);
	}

	auto GetTrans() { return data.segment<3>(0); }
	auto GetTrans() const { return data.segment<3>(0); }

	auto GetPose() { return data.segment(3, GetSkelDef(type).jointSize * 3); }
	auto GetPose() const { return data.segment(3, GetSkelDef(type).jointSize * 3); }

	auto GetTransPose() { return data.head(3 + GetSkelDef(type).jointSize * 3); }
	auto GetTransPose() const { return data.head(3 + GetSkelDef(type).jointSize * 3); }

	auto GetShape() { return data.tail(GetSkelDef(type).shapeSize); }
	auto GetShape() const { return data.tail(GetSkelDef(type).shapeSize); }
};


class SkelDriver
{
public:
	SkelDriver(const SkelType& _type, const std::string& modelPath);
	virtual ~SkelDriver() = default;

	const Eigen::Matrix3Xf& GetJoints() const { return m_joints; }
	Eigen::Matrix4Xf CalcNodeWarps(const SkelParam& param, const Eigen::Matrix3Xf& jBlend) const;
	Eigen::Matrix3Xf CalcJBlend(const SkelParam& param) const;
	Eigen::Matrix4Xf CalcChainWarps(const Eigen::Matrix4Xf& nodeWarps) const;
	Eigen::Matrix3Xf CalcJFinal(const Eigen::Matrix4Xf& chainWarps) const;
	Eigen::Matrix3Xf CalcJFinal(const SkelParam& param, const int& _jCut = -1) const;
	const SkelType& GetType() const { return m_type; }

protected:
	SkelType m_type;
	Eigen::Matrix3Xf m_joints;
	Eigen::MatrixXf m_jShapeBlend;
};


