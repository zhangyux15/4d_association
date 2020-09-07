#include <opencv2/opencv.hpp>
#include "skel_painter.h"
#include "openpose.h"


OpenposeDetection::OpenposeDetection(const SkelType& _type)
{
	type = _type;
	joints.resize(GetSkelDef(type).jointSize);
	pafs.resize(GetSkelDef(type).pafSize);
}


OpenposeDetection OpenposeDetection::Mapping(const SkelType& tarType)
{
	OpenposeDetection detection(tarType);
	for (int jIdx = 0; jIdx < GetSkelDef(type).jointSize; jIdx++) {
		const int _jIdx = GetSkelMapping(type, tarType).jointMapping[jIdx];
		if (_jIdx != -1)
			detection.joints[_jIdx] = joints[jIdx];
	}

	for (int pafIdx = 0; pafIdx < GetSkelDef(type).pafSize; pafIdx++) {
		const int _pafIdx = GetSkelMapping(type, tarType).pafMapping[pafIdx];
		if (_pafIdx != -1)
			detection.pafs[_pafIdx] = pafs[pafIdx];
	}
	return detection;
}


std::vector<Eigen::Matrix3Xf> OpenposeDetection::Associate(const int& jcntThresh)
{
	const SkelDef& def = GetSkelDef(type);
	// generate valid pafs
	std::vector<std::tuple<float, int, int, int>> pafSet;
	for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
		const int jaIdx = def.pafDict(0, pafIdx);
		const int jbIdx = def.pafDict(1, pafIdx);
		for (int jaCandiIdx = 0; jaCandiIdx < joints[jaIdx].cols(); jaCandiIdx++) {
			for (int jbCandiIdx = 0; jbCandiIdx < joints[jbIdx].cols(); jbCandiIdx++) {
				const float jaScore = joints[jaIdx](2, jaCandiIdx);
				const float jbScore = joints[jbIdx](2, jbCandiIdx);
				const float pafScore = pafs[pafIdx](jaCandiIdx, jbCandiIdx);
				if (jaScore > 0.f && jbScore > 0.f && pafScore > 0.f)
					pafSet.emplace_back(std::make_tuple(pafScore, pafIdx, jaCandiIdx, jbCandiIdx));
			}
		}
	}
	std::sort(pafSet.rbegin(), pafSet.rend());

	// construct bodies use minimal spanning tree
	std::vector<Eigen::VectorXi> personsMap;
	std::vector<Eigen::VectorXi> assignMap(def.jointSize);
	for (int jIdx = 0; jIdx < assignMap.size(); jIdx++)
		assignMap[jIdx].setConstant(joints[jIdx].cols(), -1);

	for (const auto& paf : pafSet) {
		const float pafScore = std::get<0>(paf);
		const int pafIdx = std::get<1>(paf);
		const int jaCandiIdx = std::get<2>(paf);
		const int jbCandiIdx = std::get<3>(paf);
		const int jaIdx = def.pafDict(0, pafIdx);
		const int jbIdx = def.pafDict(1, pafIdx);

		int& aAssign = assignMap[jaIdx][jaCandiIdx];
		int& bAssign = assignMap[jbIdx][jbCandiIdx];

		// 1. A & B not assigned yet: Create new person
		if (aAssign == -1 && bAssign == -1) {
			Eigen::VectorXi personMap = Eigen::VectorXi::Constant(def.jointSize, -1);
			personMap[jaIdx] = jaCandiIdx;
			personMap[jbIdx] = jbCandiIdx;
			aAssign = bAssign = int(personsMap.size());
			personsMap.emplace_back(personMap);
		}

		// 2. A assigned but not B: Add B to person with A (if no another B there) 
		// 3. B assigned but not A: Add A to person with B (if no another A there)
		else if ((aAssign >= 0 && bAssign == -1) || (aAssign == -1 && bAssign >= 0)) {
			const int assigned = aAssign >= 0 ? aAssign : bAssign;
			int& unassigned = aAssign >= 0 ? bAssign : aAssign;
			const int unassignedIdx = aAssign >= 0 ? jbIdx : jaIdx;
			const int unassignedCandiIdx = aAssign >= 0 ? jbCandiIdx : jaCandiIdx;

			Eigen::VectorXi& personMap = personsMap[assigned];
			if (personMap[unassignedIdx] == -1) {
				personMap[unassignedIdx] = unassignedCandiIdx;
				unassigned = assigned;
			}
		}

		// 4. A & B already assigned to same person (circular/redundant PAF)
		else if (aAssign == bAssign) {}

		// 5. A & B already assigned to different people: Merge people if key point intersection is null
		else {
			const int assignFst = aAssign < bAssign ? aAssign : bAssign;
			const int assignSec = aAssign < bAssign ? bAssign : aAssign;
			Eigen::VectorXi& personFst = personsMap[assignFst];
			const Eigen::VectorXi& personSec = personsMap[assignSec];

			bool conflict = false;
			for (int jIdx = 0; jIdx < def.jointSize && !conflict; jIdx++)
				conflict |= (personFst[jIdx] != -1 && personSec[jIdx] != -1);

			if (!conflict) {
				for (int jIdx = 0; jIdx < def.jointSize; jIdx++)
					if (personSec[jIdx] != -1)
						personFst[jIdx] = personSec[jIdx];

				personsMap.erase(personsMap.begin() + assignSec);
				for (Eigen::VectorXi& tmp : assignMap) {
					for (int i = 0; i < tmp.size(); i++) {
						if (tmp[i] == assignSec)
							tmp[i] = assignFst;
						else if (tmp[i] > assignSec)
							tmp[i]--;
					}
				}
			}
		}
	}

	// filter
	for (auto personMap = personsMap.begin(); personMap != personsMap.end();) {
		if ((personMap->array() >= 0).count() < jcntThresh) {
			const int personIdx = int(personMap - personsMap.begin());
			for (Eigen::VectorXi& tmp : assignMap) {
				for (int i = 0; i < tmp.size(); i++) {
					if (tmp[i] == personIdx)
						tmp[i] = -1;
					else if (tmp[i] > personIdx)
						tmp[i]--;
				}
			}
			personMap = personsMap.erase(personMap);
		}
		else
			personMap++;
	}

	std::vector<Eigen::Matrix3Xf> skels;
	for (const auto& personMap : personsMap) {
		Eigen::Matrix3Xf skel = Eigen::Matrix3Xf::Zero(3, def.jointSize);
		for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
			const int index = personMap[jIdx];
			if (index != -1)
				skel.col(jIdx) = joints[jIdx].col(index);
		}
		skels.emplace_back(skel);
	}
	return skels;
}


std::vector<OpenposeDetection> ParseDetections(const std::string& filename)
{
	std::ifstream fs(filename);
	if (!fs.is_open()) {
		std::cerr << "file not exist: " << filename << std::endl;
		std::abort();
	}

	int skelType, frameSize;
	fs >> skelType >> frameSize;

	const SkelDef& def = GetSkelDef(SkelType(skelType));
	std::vector<OpenposeDetection> detections(frameSize, OpenposeDetection(SkelType(skelType)));
	for (OpenposeDetection& detection : detections) {
		for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
			int jSize;
			fs >> jSize;
			detection.joints[jIdx].resize(3, jSize);
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < jSize; j++)
					fs >> detection.joints[jIdx](i, j);
		}
		for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
			const int jAIdx = def.pafDict(0, pafIdx);
			const int jBIdx = def.pafDict(1, pafIdx);
			detection.pafs[pafIdx].resize(detection.joints[jAIdx].cols(), detection.joints[jBIdx].cols());
			for (int i = 0; i < detection.pafs[pafIdx].rows(); i++)
				for (int j = 0; j < detection.pafs[pafIdx].cols(); j++)
					fs >> detection.pafs[pafIdx](i, j);

			detection.pafs[pafIdx] = detection.pafs[pafIdx].array().pow(0.2f);
		}
	}
	fs.close();
	return detections;
}


void SerializeDetections(const std::vector<OpenposeDetection>& detections, const std::string& filename)
{
	const SkelDef& def = GetSkelDef(detections.begin()->type);
	std::ofstream fs(filename);
	fs << detections.begin()->type << "\t" << detections.size() << std::endl;

	for (const OpenposeDetection& detection : detections) {
		for (int jIdx = 0; jIdx < def.jointSize; jIdx++)
			fs << detection.joints[jIdx].cols() << std::endl << detection.joints[jIdx] << std::endl;

		for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++)
			fs << detection.pafs[pafIdx] << std::endl;;
	}
	fs.close();
}

