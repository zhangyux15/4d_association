#include "skel_painter.h"
#include "color_util.h"
#include "math_util.h"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>


std::vector<cv::Rect> SkelPainter::MergeImgs(const std::vector<cv::Mat>& imgs, cv::Mat& mergedImg, const int& cols, const cv::Size& _size)
{
	const cv::Size size = _size.width * _size.height == 0 ? cv::Size(imgs.begin()->cols, imgs.begin()->rows) : _size;
	const int rows = int((imgs.size() + cols - 1) / cols);
	mergedImg.create(rows*size.height, cols*size.width, imgs.begin()->type());
	std::vector<cv::Rect> rois;
	for (int view = 0; view < imgs.size(); view++) {
		cv::Rect roi(cv::Point2i(view%cols * size.width, view / cols * size.height), size);
		cv::resize(imgs[view], mergedImg(roi), size);
		rois.emplace_back(roi);
	}
	return rois;
}


void SkelPainter::DrawDetect(const std::vector<Eigen::Matrix3Xf>& joints, const std::vector<Eigen::MatrixXf>& pafs, cv::Mat img) const
{
	const SkelDef& def = GetSkelDef(type);
	const cv::Size size(img.cols, img.rows);

	// draw paf
	if (pafThickness > 0) {
		for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
			const int jaIdx = def.pafDict(0, pafIdx);
			const int jbIdx = def.pafDict(1, pafIdx);
			for (int jaCandiIdx = 0; jaCandiIdx < joints[jaIdx].cols(); jaCandiIdx++) {
				for (int jbCandiIdx = 0; jbCandiIdx < joints[jbIdx].cols(); jbCandiIdx++) {
					if (pafs[pafIdx](jaCandiIdx, jbCandiIdx) < FLT_EPSILON)
						continue;
					cv::line(img, cv::Point(joints[jaIdx](0,jaCandiIdx), joints[jaIdx](1, jaCandiIdx)),
						cv::Point(joints[jbIdx](0, jbCandiIdx), joints[jbIdx](1, jbCandiIdx)),
						ColorUtil::GetColor(-1), pafThickness);
				}
			}
		}
	}

	// draw candidate
	if (jointRadius > 0 || textScale > 0.f) {
		for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
			for (int candiIdx = 0; candiIdx < joints[jIdx].cols(); candiIdx++) {
				const cv::Point jPos(joints[jIdx](0, candiIdx), joints[jIdx](1, candiIdx));
				if (jointRadius > 0)
					cv::circle(img, jPos, jointRadius, ColorUtil::GetColor(-1), 1);
				if (textScale > 0.f)
					cv::putText(img, std::to_string(jIdx), jPos, cv::FONT_HERSHEY_PLAIN, textScale, ColorUtil::GetColor(-1));
			}
		}
	}
}


void SkelPainter::DrawAssoc(const Eigen::Matrix3Xf& skel2d, cv::Mat img, const int& identity) const
{
	const SkelDef& def = GetSkelDef(type);
	const cv::Size size(img.cols, img.rows);

	for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
		if (skel2d(2, jIdx) < FLT_EPSILON)
			continue;

		cv::Point jPos(skel2d(0, jIdx), skel2d(1, jIdx));
		if (jointRadius > 0)
			cv::circle(img, jPos, jointRadius, ColorUtil::GetColor(identity), 1);
		if (textScale > 0.f)
			cv::putText(img, std::to_string(jIdx), jPos, cv::FONT_HERSHEY_PLAIN, textScale, ColorUtil::GetColor(identity));
	}

	if (pafThickness > 0) {
		for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
			const int jaIdx = def.pafDict(0, pafIdx);
			const int jbIdx = def.pafDict(1, pafIdx);
			if (skel2d(2, jaIdx) < FLT_EPSILON || skel2d(2, jbIdx) < FLT_EPSILON)
				continue;

			cv::line(img, cv::Point(skel2d(0, jaIdx), skel2d(1, jaIdx)),
				cv::Point(skel2d(0, jbIdx), skel2d(1, jbIdx)),
				ColorUtil::GetColor(identity), pafThickness);
		}
	}
}



void SkelPainter::DrawReproj(const Eigen::Matrix4Xf& skel3d, const Eigen::Matrix<float, 3, 4>& proj, cv::Mat img, const int& identity) const
{
	const SkelDef& def = GetSkelDef(type);
	const cv::Size size(img.cols, img.rows);

	Eigen::Matrix3Xf skel2d(3, def.jointSize);
	skel2d.topRows(2) = (proj * skel3d.topRows(3).colwise().homogeneous()).colwise().hnormalized();
	skel2d.row(2) = skel3d.row(3);

	for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
		if (skel2d(2, jIdx) < FLT_EPSILON)
			continue;

		cv::Point jPos(skel2d(0, jIdx), skel2d(1, jIdx));
		if (jointRadius > 0)
			cv::circle(img, jPos, jointRadius, ColorUtil::GetColor(identity), 1);
		if (textScale > 0.f)
			cv::putText(img, std::to_string(jIdx), jPos, cv::FONT_HERSHEY_PLAIN, textScale, ColorUtil::GetColor(identity));
	}

	if (pafThickness > 0) {
		for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
			const int jaIdx = def.pafDict(0, pafIdx);
			const int jbIdx = def.pafDict(1, pafIdx);
			if (skel2d(2, jaIdx) < FLT_EPSILON || skel2d(2, jbIdx) < FLT_EPSILON)
				continue;

			cv::line(img, cv::Point(skel2d(0, jaIdx), skel2d(1, jaIdx)),
				cv::Point(skel2d(0, jbIdx), skel2d(1, jbIdx)),
				ColorUtil::GetColor(identity), pafThickness);
		}
	}
}


