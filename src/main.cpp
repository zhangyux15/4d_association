#include "kruskal_associater.h"
#include "skel_updater.h"
#include "skel_painter.h"
#include "openpose.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <json/json.h>


int main()
{
	std::map<std::string, Camera> cameras = ParseCameras("../data/shelf/calibration.json");
	Eigen::Matrix3Xf projs(3, cameras.size() * 4);
	std::vector<cv::Mat> rawImgs(cameras.size());
	std::vector<cv::VideoCapture> videos(cameras.size());
	std::vector<std::vector<OpenposeDetection>> seqDetections(cameras.size());
	const SkelDef& skelDef = GetSkelDef(SKEL19);

#pragma omp parallel for
	for (int i = 0; i < cameras.size(); i++) {
		auto iter = std::next(cameras.begin(), i);
		videos[i] = cv::VideoCapture("../data/shelf/video/" + iter->first + ".mpeg");
		videos[i].set(cv::CAP_PROP_POS_FRAMES, 0);

		projs.middleCols(4 * i, 4) = iter->second.eiProj;
		seqDetections[i] = ParseDetections("../data/shelf/detection/" + iter->first + ".txt");
		cv::Size imgSize(int(videos[i].get(cv::CAP_PROP_FRAME_WIDTH)), int(videos[i].get(cv::CAP_PROP_FRAME_HEIGHT)));
		for (auto&&detection : seqDetections[i]) {
			for (auto&& joints : detection.joints) {
				joints.row(0) *= imgSize.width;
				joints.row(1) *= imgSize.height;
			}
		}
		rawImgs[i] = cv::Mat();
	}

	KruskalAssociater associater(SKEL19, cameras);
	associater.SetMaxTempDist(0.3f);
	associater.SetMaxEpiDist(0.15f);
	associater.SetPlaneThetaWelsh(5e-3f);
	associater.SetEpiWeight(1.f);
	associater.SetTempWeight(2.f);
	associater.SetViewWeight(1.f);
	associater.SetPafWeight(1.f);
	associater.SetHierWeight(0.f);
	associater.SetViewCntWelsh(1.5);
	associater.SetMinCheckCnt(2);
	associater.SetNodeMultiplex(true);

	SkelFittingUpdater skelUpdater(SKEL19, "../data/skel/SKEL19");
	SkelPainter skelPainter(SKEL19);

	for (int frameIdx = 0; ; frameIdx++) {
		for (int view = 0; view < cameras.size(); view++) {
			videos[view] >> rawImgs[view];
			associater.SetDetection(view, seqDetections[view][frameIdx].Mapping(SKEL19));
		}

		associater.SetSkels3dPrev(skelUpdater.GetSkel3d());
		associater.Associate();
		skelUpdater.Update(associater.GetSkels2d(), projs);

		
		// save
		const int layoutCols = 3;
		cv::Mat detectImg, assocImg, reprojImg;
		std::vector<cv::Rect> rois = SkelPainter::MergeImgs(rawImgs, detectImg, layoutCols,
			{ rawImgs.begin()->cols, rawImgs.begin()->rows});
		detectImg.copyTo(assocImg);
		detectImg.copyTo(reprojImg);

#pragma omp parallel for
		for (int view = 0; view < cameras.size(); view++) {
			const OpenposeDetection detection = seqDetections[view][frameIdx].Mapping(SKEL19);
			skelPainter.DrawDetect(detection.joints, detection.pafs, detectImg(rois[view]));
			for (const auto& skel2d : associater.GetSkels2d())
				skelPainter.DrawAssoc(skel2d.second.middleCols(view * skelDef.jointSize, skelDef.jointSize), assocImg(rois[view]), skel2d.first);

			for(const auto& skel3d : skelUpdater.GetSkel3d())
				skelPainter.DrawReproj(skel3d.second, projs.middleCols(4 * view, 4), reprojImg(rois[view]), skel3d.first);
		}

		cv::imwrite("../output/detect/" + std::to_string(frameIdx) + ".jpg", detectImg);
		cv::imwrite("../output/assoc/" + std::to_string(frameIdx) + ".jpg", assocImg);
		cv::imwrite("../output/reproj/" + std::to_string(frameIdx) + ".jpg", reprojImg);
		std::cout << std::to_string(frameIdx) << std::endl;
	}
	return 0;
}
