#pragma once
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <memory>
#include "skel.h"
#include "color_util.h"


struct SkelPainter
{
	SkelType type;
	int jointRadius = 3;
	int pafThickness = 2;
	float textScale = 1.f;
	float rate = 1.f;

	SkelPainter(const SkelType& _type) { type = _type; }
	static std::vector<cv::Rect> MergeImgs(const std::vector<cv::Mat>& imgs, cv::Mat& mergedImg, 
		const int& cols = 3, const cv::Size& _size = cv::Size(0, 0));
	void DrawDetect(const std::vector<Eigen::Matrix3Xf>& joints, const std::vector<Eigen::MatrixXf>& pafs, cv::Mat img) const;
	void DrawAssoc(const Eigen::Matrix3Xf& skel2d, cv::Mat img, const int& identity = 0) const;
	void DrawReproj(const Eigen::Matrix4Xf& skel3d, const Eigen::Matrix<float, 3, 4>& proj, cv::Mat img, const int& identity = 0) const;
};
