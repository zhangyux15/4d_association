#pragma once
#include <string>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <json/json.h>
#include "math_util.h"


struct Camera
{
	cv::Size imgSize;
	// opencv
	cv::Matx33f cvK, cvKi, cvR, cvRt, cvRtKi;
	cv::Vec3f cvT, cvPos;
	cv::Matx<float, 3, 4> cvProj;

	// eigen
	Eigen::Matrix3f eiK, eiKi, eiR, eiRt, eiRtKi;
	Eigen::Vector3f eiT, eiPos;
	Eigen::Matrix<float, 3, 4> eiProj;

	// calibration param
	cv::Mat_<float> distCoeff = cv::Mat_<float>::zeros(5, 1);
	double rectifyAlpha = 0;
	cv::Matx33f originK;
	cv::Mat rectifyMapX, rectifyMapY;

	Camera() {}
	Camera(const Json::Value& json) { Parse(json); }
	void Parse(const Json::Value& json);
	Json::Value Serialize() const;

	void CV2Eigen();
	void Eigen2CV();
	void Rectify();
	void LookAt(const Eigen::Vector3f& eye, const Eigen::Vector3f& center, const Eigen::Vector3f& up);
	Eigen::Matrix3f CalcFundamental(const Camera& camera) const;
	Eigen::Vector3f CalcRay(const Eigen::Vector2f& uv) const;
};


std::map<std::string, Camera> ParseCameras(const std::string& jsonFile);
void SerializeCameras(const std::map<std::string, Camera>& cameras, const std::string& jsonFile);


struct Triangulator
{
	Eigen::Matrix3Xf points;
	Eigen::Matrix3Xf projs;
	bool convergent;
	float loss;
	Eigen::Vector3f pos;
	void Solve(const int& maxIterTime = 20, const float& updateTolerance = 1e-4f, const float& regularTerm = 1e-4f);
};