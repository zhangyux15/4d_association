#include "../src/kruskal_associater.h"
#include "../src/skel_updater.h"
#include "../src/skel_painter.h"
#include "../src/hungarian_algorithm.h"
#include <opencv2/opencv.hpp>
#include <json/json.h>


// #define SAVE_RESULT
#define RUN_OLD_VERSION


Eigen::Matrix4Xf MappingToShelf(const Eigen::Matrix4Xf& skel19)
{
	Eigen::Matrix4Xf shelf15(4, 15);
	const std::vector<int> mapping = { 13, 7, 2, 3, 8, 14, 15, 11, 5, 6, 12, 16, 1, 4, 0 };
	for (int jIdx = 0; jIdx < mapping.size(); jIdx++)
		shelf15.col(jIdx) = skel19.col(mapping[jIdx]);

	// interp head
	const Eigen::Vector3f faceDir = (shelf15.block<3, 1>(0, 12) - shelf15.block<3, 1>(0, 14)).cross(
		(shelf15.block<3, 1>(0, 8) - shelf15.block<3, 1>(0, 9))).normalized();
	const Eigen::Vector3f zDir(0.f, 0.f, 1.f);
	const Eigen::Vector3f shoulderCenter = (skel19.block<3, 1>(0, 5) + skel19.block<3, 1>(0, 6)) / 2.f;
	const Eigen::Vector3f headCenter = (skel19.block<3, 1>(0, 9) + skel19.block<3, 1>(0, 10)) / 2.f;
	
	shelf15.block<3, 1>(0, 12) = shoulderCenter + (headCenter - shoulderCenter)*0.5;
	shelf15.block<3, 1>(0, 13) = shelf15.block<3, 1>(0, 12) + faceDir * 0.125 + zDir * 0.145;
	return shelf15;
}


Eigen::VectorXi Evaluate(const Eigen::Matrix4Xf& shelfSkel, const Eigen::Matrix4Xf& gt)
{
	const SkelDef& def = GetSkelDef(SHELF15);
	Eigen::VectorXi c = Eigen::VectorXi::Zero(def.pafSize);
	for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
		const int jaIdx = def.pafDict(0, pafIdx);
		const int jbIdx = def.pafDict(1, pafIdx);
		float da = (shelfSkel.col(jaIdx) - gt.col(jaIdx)).head(3).norm();
		float db = (shelfSkel.col(jbIdx) - gt.col(jbIdx)).head(3).norm();
		float l = (gt.col(jaIdx) - gt.col(jbIdx)).head(3).norm();
		if (da + db < l)
			c[pafIdx] = 1;
	}
	return c;
}


void PrintEvaluation(const std::vector<Eigen::VectorXi>& correctJCnt) {
	Eigen::VectorXi sum = Eigen::VectorXi::Zero(correctJCnt.begin()->size());
	for (const auto& c : correctJCnt)
		sum += c;

	std::vector<std::string> pafName = {
		"Left Upper Arm", "Right Upper Arm", "Left Lower Arm", "Right Lower Arm",
		"Left Upper Leg", "Right Upper Leg", "Left Lower Leg", "Right Lower Leg",
		"Head", "Torso" };

	Eigen::VectorXf rate = sum.cast<float>() / float(correctJCnt.size());
	for (int i = 0; i < sum.size(); i++)
		std::cout << pafName[i] << ": " << sum[i] << "/" << correctJCnt.size() << " " << rate[i] << std::endl;

	std::cout << "Average:" << rate.sum() / rate.size() << std::endl;
}


int main()
{
	// init
	std::map<std::string, Camera> cams = ParseCameras("../data/shelf/calibration.json");
	Eigen::Matrix3Xf projs(3, cams.size() * 4);
	std::vector<cv::VideoCapture> videos(cams.size());
	std::vector<cv::Mat> rawImgs(cams.size());
	std::vector<std::vector<OpenposeDetection>> seqDetections(cams.size());
	std::vector<std::map<int, Eigen::Matrix4Xf>> gt = ParseSkels("../data/shelf/gt.txt");
	std::vector<std::map<int, Eigen::Matrix4Xf>> skels;

#pragma omp parallel for
	for (int i = 0; i < cams.size(); i++) {
		auto iter = std::next(cams.begin(), i);
		videos[i] = cv::VideoCapture("../data/shelf/video/" + iter->first + ".mp4");
		projs.middleCols(4 * i, 4) = iter->second.eiProj;
		seqDetections[i] = ParseDetections("../data/shelf/detection/" + iter->first + ".txt");
		cv::Size imgSize(int(videos[i].get(cv::CAP_PROP_FRAME_WIDTH)), int(videos[i].get(cv::CAP_PROP_FRAME_HEIGHT)));
		for (auto&&detection : seqDetections[i]) {
			for (auto&& joints : detection.joints) {
				joints.row(0) *= (imgSize.width - 1);
				joints.row(1) *= (imgSize.height - 1);
			}
		}
		rawImgs[i].create(imgSize, CV_8UC3);
	}

	KruskalAssociater associater(SKEL19, cams);
	associater.SetMaxTempDist(0.2f);
	associater.SetMaxEpiDist(0.15f);
	associater.SetEpiWeight(2.f);
	associater.SetTempWeight(2.f);
	associater.SetViewWeight(2.f);
	associater.SetPafWeight(1.f);
	associater.SetHierWeight(.5f);
	associater.SetViewCntWelsh(1.5f);
	associater.SetMinCheckCnt(1);
	associater.SetNodeMultiplex(true);
	associater.SetNormalizeEdge(true);			// new feature

#ifdef RUN_OLD_VERSION
	SkelFittingUpdater skelUpdater(SKEL19, "../data/skel/SKEL19_old");
#else
	SkelFittingUpdater skelUpdater(SKEL19, "../data/skel/SKEL19");
#endif
	skelUpdater.SetTriangulateThresh(0.05f);
	skelUpdater.SetMinTrackCnt(5);
	skelUpdater.SetBoneCapacity(100);
	skelUpdater.SetSquareShapeTerm(1e-2f);
	skelUpdater.SetRegularPoseTerm(1e-3f);
	skelUpdater.SetTemporalTransTerm(1e-1f);
	skelUpdater.SetTemporalPoseTerm(1e-2f);
	skelUpdater.SetShapeMaxIter(5);
	skelUpdater.SetPoseMaxIter(20);
	skelUpdater.SetInitActive(0.9f);
	skelUpdater.SetActiveRate(0.1f);

	SkelPainter skelPainter(SKEL19);
	skelPainter.jointRadius = 3;
	skelPainter.pafThickness = 2;
	skelPainter.rate = 0.5f;
	SkelPainter shelfPainter(SHELF15);
	shelfPainter.jointRadius = 3;
	shelfPainter.pafThickness = 2;
	shelfPainter.rate = 0.5f;

	std::map<int, std::vector<Eigen::VectorXi>> correctJCnt;
	// process sequence
	for (int frameIdx = 0; frameIdx < seqDetections.begin()->size(); frameIdx++) {
		for (int view = 0; view < cams.size(); view++) {
			videos[view] >> rawImgs[view];
			associater.SetDetection(view, seqDetections[view][frameIdx].Mapping(SKEL19));
		}

		associater.SetSkels3dPrev(skelUpdater.GetSkel3d());
		associater.Associate();
		skelUpdater.Update(associater.GetSkels2d(), projs);
		skels.emplace_back(skelUpdater.GetSkel3d());

		std::cout << std::to_string(frameIdx) << std::endl;

		// evaluate
		std::map<int, Eigen::Matrix4Xf> shelfSkels;
		for (const auto& skel : skelUpdater.GetSkel3d())
			shelfSkels.insert(std::make_pair(-skel.first, MappingToShelf(skel.second)));

		Eigen::MatrixXf hungarianMat(shelfSkels.size(), gt[frameIdx].size());
		for (int i = 0; i < shelfSkels.size(); i++)
			for (int j = 0; j < gt[frameIdx].size(); j++)
				hungarianMat(i, j) = (std::next(shelfSkels.begin(), i)->second -
					std::next(gt[frameIdx].begin(), j)->second).topRows(3).colwise().norm().sum();
		for (const auto& matchPair : HungarianAlgorithm(hungarianMat)) {
			const auto shelfIter = std::next(shelfSkels.begin(), matchPair.second.x());
			const auto gtIter = std::next(gt[frameIdx].begin(), matchPair.second.y());
			const Eigen::VectorXi c = Evaluate(shelfIter->second, gtIter->second);
			const int identity = gtIter->first;
			auto iter = correctJCnt.find(identity);
			if (iter == correctJCnt.end())
				iter = correctJCnt.insert(std::make_pair(identity, std::vector<Eigen::VectorXi>())).first;
			iter->second.emplace_back(c);
		}

#ifdef SAVE_RESULT
		skels.emplace_back(skelUpdater.GetSkel3d());
		const int layoutCols = 3;
		cv::Mat detectImg, assocImg, reprojImg, gtImg;
		std::vector<cv::Rect> rois = SkelPainter::MergeImgs(rawImgs, detectImg, layoutCols,
			{ int(skelPainter.rate * rawImgs.begin()->cols), int(skelPainter.rate * rawImgs.begin()->rows) });
		detectImg.copyTo(assocImg);
		detectImg.copyTo(reprojImg);
		detectImg.copyTo(gtImg);

#pragma omp parallel for
		for (int view = 0; view < cams.size(); view++) {
			const OpenposeDetection detection = seqDetections[view][frameIdx].Mapping(SKEL19);
			skelPainter.DrawDetect(detection.joints, detection.pafs, detectImg(rois[view]));
			for (const auto& skel2d : associater.GetSkels2d())
				skelPainter.DrawAssoc(skel2d.second.middleCols(view * detection.joints.size(), detection.joints.size()), assocImg(rois[view]), skel2d.first);

			for (const auto& skel3d : skelUpdater.GetSkel3d())
				skelPainter.DrawReproj(skel3d.second, projs.middleCols(4 * view, 4), reprojImg(rois[view]), skel3d.first);

			for (const auto& skel3d : shelfSkels)
				shelfPainter.DrawReproj(skel3d.second, projs.middleCols(4 * view, 4), gtImg(rois[view]), 0);

			for (const auto& skel3d : gt[frameIdx])
				shelfPainter.DrawReproj(skel3d.second, projs.middleCols(4 * view, 4), gtImg(rois[view]), 1);

		}

		cv::imwrite("../output/detect/" + std::to_string(frameIdx) + ".jpg", detectImg);
		cv::imwrite("../output/assoc/" + std::to_string(frameIdx) + ".jpg", assocImg);
		cv::imwrite("../output/reproj/" + std::to_string(frameIdx) + ".jpg", reprojImg);
		cv::imwrite("../output/gt/" + std::to_string(frameIdx) + ".jpg", gtImg);
#endif

	}
	SerializeSkels(skels, "../data/shelf/skel.txt");
	for (const auto& pair : correctJCnt) {
		std::cout << "identity: " << pair.first << std::endl;
		PrintEvaluation(pair.second);
	}
	system("pause");
	return 0;
}


