#include <iostream>
#include <sstream>
#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "camera.h"
#include "math_util.h"


void Camera::Parse(const Json::Value& json)
{
	Json::Value var = json["K"];
	for (int row = 0; row < 3; row++)
		for (int col = 0; col < 3; col++)
			originK(row, col) = var[row * 3 + col].asFloat();

	if (json.isMember("R")) {
		var = json["R"];
		if (var.size() == 3) {
			cv::Vec3f r;
			for (int i = 0; i < 3; i++)
				r[i] = var[i].asFloat();
			cv::Rodrigues(r, cvR);
		}
		else if (var.size() == 9) {
			for (int row = 0; row < 3; row++)
				for (int col = 0; col < 3; col++)
					cvR(row, col) = var[row * 3 + col].asFloat();
		}
		else {
			std::cerr << "Unknown rotation format" << std::endl;
		}
	}

	if (json.isMember("T")) {
		var = json["T"];
		for (int i = 0; i < 3; i++)
			cvT[i] = var[i].asFloat();
	}

	if (json.isMember("RT")) {
		var = json["RT"];
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++)
				cvR(row, col) = var[row * 4 + col].asFloat();
			cvT[row] = var[row * 4 + 3].asFloat();
		}
	}

	var = json["imgSize"];
	imgSize = cv::Size(var[0].asFloat(), var[1].asFloat());

	if (json.isMember("distCoeff")) {
		var = json["distCoeff"];
		distCoeff.resize(var.size(), 1);
		for (int i = 0; i < int(var.size()); i++)
			distCoeff(i) = var[i].asFloat();
	}
	else
		distCoeff = cv::Mat_<float>::zeros(5, 1);

	if (json.isMember("rectifyAlpha"))
		rectifyAlpha = json["rectifyAlpha"].asDouble();
	else
		rectifyAlpha = 0;

	Rectify();
	CV2Eigen();
}


void Camera::Rectify()
{
	cvK = cv::getOptimalNewCameraMatrix(
		originK, distCoeff, imgSize, rectifyAlpha);
	cv::initUndistortRectifyMap(
		originK, distCoeff, cv::Mat(), cvK, imgSize, CV_32FC1, rectifyMapX, rectifyMapY);
}


void Camera::CV2Eigen()
{
	cvKi = cvK.inv();
	cvRt = cvR.t();
	cvRtKi = cvRt * cvKi;
	cvPos = -cvRt * cvT;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 4; j++)
			cvProj(i, j) = j < 3 ? cvR(i, j) : cvT[i];
	cvProj = cvK * cvProj;

	cv::cv2eigen(cvK, eiK);
	cv::cv2eigen(cvKi, eiKi);
	cv::cv2eigen(cvR, eiR);
	cv::cv2eigen(cvRt, eiRt);
	cv::cv2eigen(cvRtKi, eiRtKi);
	cv::cv2eigen(cvT, eiT);
	cv::cv2eigen(cvPos, eiPos);
	cv::cv2eigen(cvProj, eiProj);
}


void Camera::Eigen2CV()
{
	eiKi = eiK.inverse();
	eiRt = eiR.transpose();
	eiRtKi = eiRt * eiKi;
	eiPos = -eiRt * eiT;
	eiProj.leftCols<3>() = eiR;
	eiProj.col(3) = eiT;
	eiProj = eiK * eiProj;

	cv::eigen2cv(eiK, cvK);
	cv::eigen2cv(eiKi, cvKi);
	cv::eigen2cv(eiR, cvR);
	cv::eigen2cv(eiRt, cvRt);
	cv::eigen2cv(eiRtKi, cvRtKi);
	cv::eigen2cv(eiT, cvT);
	cv::eigen2cv(eiPos, cvPos);
	cv::eigen2cv(eiProj, cvProj);
}


void Camera::LookAt(const Eigen::Vector3f& eye, const Eigen::Vector3f& center, const Eigen::Vector3f& up)
{
	eiR.row(2) = (center - eye).transpose().normalized();
	eiR.row(0) = (up.cross(eiR.row(2).transpose())).transpose().normalized();
	eiR.row(1) = (eiR.row(2).transpose().cross(eiR.row(0).transpose())).transpose().normalized();
	eiT = -eiR * eye;
	Eigen2CV();
}


Json::Value Camera::Serialize() const
{
	Json::Value json;
	json["K"].resize(0);
	for (int row = 0; row < 3; row++)
		for (int col = 0; col < 3; col++)
			json["K"].append(Json::Value(originK(row, col)));

	json["R"].resize(0);
	for (int row = 0; row < 3; row++)
		for (int col = 0; col < 3; col++)
			json["R"].append(Json::Value(cvR(row, col)));

	json["T"].resize(0);
	for (int i = 0; i < 3; i++)
		json["T"].append(Json::Value(cvT(i)));

	json["imgSize"].resize(0);
	json["imgSize"].append(Json::Value(imgSize.width));
	json["imgSize"].append(Json::Value(imgSize.height));

	json["distCoeff"].resize(0);
	for (int i = 0; i < distCoeff.rows; i++)
		json["distCoeff"].append(Json::Value(distCoeff(i)));
	json["rectifyAlpha"]=Json::Value(rectifyAlpha);
	return json;
}


Eigen::Matrix3f Camera::CalcFundamental(const Camera& camera) const
{
	const Eigen::Matrix3f relaR = eiR * camera.eiRt;
	const Eigen::Vector3f relaT = eiT - relaR * camera.eiT;
	return (eiKi.transpose())*MathUtil::Skew(relaT)*relaR*(camera.eiKi);
}


Eigen::Vector3f Camera::CalcRay(const Eigen::Vector2f& uv) const
{
	return (-eiRtKi * uv.homogeneous()).normalized();
}


std::map<std::string, Camera> ParseCameras(const std::string& jsonFile)
{
	Json::Value json;
	std::ifstream fs(jsonFile);
	if (!fs.is_open()) {
		std::cerr << "json file not exist: " << jsonFile << std::endl;
		std::abort();
	}

	std::string errs;
	Json::parseFromStream(Json::CharReaderBuilder(), fs, &json, &errs);
	fs.close();

	if (errs != "") {
		std::cerr << "json read file error: " << errs << std::endl;
		std::abort();
	}

	std::map<std::string, Camera> cameras;
	for (auto camIter = json.begin(); camIter != json.end(); camIter++) 
		cameras.insert(std::make_pair(camIter.key().asString(), Camera(*camIter)));
	return cameras;
}


void SerializeCameras(const std::map<std::string, Camera>& cameras, const std::string& jsonFile)
{
	Json::Value json;
	for (const auto& camIter : cameras) 
		json[camIter.first] = camIter.second.Serialize();

	std::ofstream ofs(jsonFile);
	std::unique_ptr<Json::StreamWriter> sw_t(Json::StreamWriterBuilder().newStreamWriter());
	sw_t->write(json, &ofs);
	ofs.close();
}


void Triangulator::Solve(const int& maxIterTime, const float& updateTolerance, const float& regularTerm) {
	convergent = false;
	loss = FLT_MAX;
	pos.setZero();

	if ((points.row(2).array() > FLT_EPSILON).count() < 2) 
		return;

	for (int iterTime = 0; iterTime < maxIterTime && !convergent; iterTime++) {
		Eigen::Matrix3f ATA = regularTerm * Eigen::Matrix3f::Identity();
		Eigen::Vector3f ATb = Eigen::Vector3f::Zero();
		for (int view = 0; view < points.cols(); view++) {
			if (points(2, view) > FLT_EPSILON) {
				auto proj = projs.middleCols(4 * view, 4);
				const Eigen::Vector3f xyz = proj * pos.homogeneous();
				Eigen::Matrix<float, 2, 3> jacobi;
				jacobi << 1.0f / xyz.z(), 0.0f, -xyz.x() / (xyz.z()*xyz.z()),
					0.0f, 1.0f / xyz.z(), -xyz.y() / (xyz.z()*xyz.z());
				jacobi = jacobi * proj.leftCols(3);
				const float w = points(2, view);
				ATA += w * jacobi.transpose() * jacobi;
				ATb += w * jacobi.transpose()*(points.col(view).head(2) - xyz.hnormalized());
			}
		}
		const Eigen::Vector3f delta = ATA.ldlt().solve(ATb);
		loss = delta.norm();
		if (delta.norm() < updateTolerance)
			convergent = true;
		else
			pos += delta;
	}
}
