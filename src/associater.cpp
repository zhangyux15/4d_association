#include "associater.h"
#include "math_util.h"
#include <Eigen/Eigen>


Associater::Associater(const SkelType& type, const std::map<std::string, Camera>& cams)
{
	m_type = type;
	m_cams = cams;
	const SkelDef& def = GetSkelDef(m_type);

	m_detections.resize(m_cams.size());
	m_assignMap.resize(m_cams.size(), std::vector<Eigen::VectorXi>(def.jointSize));
	m_jointRays.resize(m_cams.size(), std::vector<Eigen::Matrix3Xf>(def.jointSize));
	m_epiEdges.resize(def.jointSize, std::vector<std::vector<Eigen::MatrixXf>>(m_cams.size(), std::vector<Eigen::MatrixXf>(m_cams.size())));
	m_tempEdges.resize(def.jointSize, std::vector<Eigen::MatrixXf>(m_cams.size()));
}


float Associater::Point2LineDist(const Eigen::Vector3f& pA, const Eigen::Vector3f& pB, const Eigen::Vector3f& ray)
{
	return ((pA - pB).cross(ray)).norm();
}


float Associater::Line2LineDist(const Eigen::Vector3f& pA, const Eigen::Vector3f& rayA, const Eigen::Vector3f& pB, const Eigen::Vector3f& rayB)
{
	if (std::abs(rayA.dot(rayB)) < 1e-5f)
		return Point2LineDist(pA, pB, rayA);
	else
		return std::abs((pA - pB).dot((rayA.cross(rayB)).normalized()));
}


void Associater::Initialize()
{
	const SkelDef& def = GetSkelDef(m_type);
#pragma omp parallel for
	for (int jIdx = 0; jIdx < def.jointSize; jIdx++)
		for (int view = 0; view < m_cams.size(); view++)
			m_assignMap[view][jIdx].setConstant(m_detections[view].joints[jIdx].cols(), -1);

	m_personsMap.clear();
	for (int i = 0; i < m_skels3dPrev.size(); i++)
		m_personsMap.insert(std::make_pair(i, Eigen::MatrixXi::Constant(GetSkelDef(m_type).jointSize, m_cams.size(), -1)));
}


void Associater::CalcJointRays()
{
	const SkelDef& def = GetSkelDef(m_type);
#pragma omp parallel for
	for (int view = 0; view < m_cams.size(); view++) {
		const Camera& cam = std::next(m_cams.begin(), view)->second;
		for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
			const Eigen::Matrix3Xf& joints = m_detections[view].joints[jIdx];
			m_jointRays[view][jIdx].resize(3, joints.cols());
			for (int jCandiIdx = 0; jCandiIdx < joints.cols(); jCandiIdx++)
				m_jointRays[view][jIdx].col(jCandiIdx) = cam.CalcRay(joints.block<2, 1>(0, jCandiIdx));
		}
	}
}


void Associater::CalcPafEdges()
{
	const SkelDef& def = GetSkelDef(m_type);
	if (m_normalizeEdges) {
#pragma omp parallel for
		for (int pafIdx = 0; pafIdx < def.pafSize; pafIdx++) {
			const Eigen::Vector2i jIdxPair = def.pafDict.col(pafIdx).transpose();
			for (auto&& detection : m_detections) {
				auto&& pafs = detection.pafs[pafIdx];
				if (pafs.size() > 0) {
					Eigen::VectorXf rowFactor = pafs.rowwise().sum().transpose().cwiseMax(1.f);
					Eigen::VectorXf colFactor = pafs.colwise().sum().cwiseMax(1.f);
					for (int i = 0; i < rowFactor.size(); i++)
						pafs.row(i) /= rowFactor[i];
					for (int i = 0; i < colFactor.size(); i++)
						pafs.col(i) /= colFactor[i];
				}
			}
		}
	}
}


void Associater::CalcEpiEdges()
{
	const SkelDef& def = GetSkelDef(m_type);
#pragma omp parallel for
	for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
		auto camAIter = m_cams.begin();
		for (int viewA = 0; viewA < m_cams.size() - 1; viewA++, camAIter++) {
			auto camBIter = std::next(camAIter);
			for (int viewB = viewA + 1; viewB < m_cams.size(); viewB++, camBIter++) {
				Eigen::MatrixXf& epi = m_epiEdges[jIdx][viewA][viewB];
				const Eigen::Matrix3Xf& jointsA = m_detections[viewA].joints[jIdx];
				const Eigen::Matrix3Xf& jointsB = m_detections[viewB].joints[jIdx];
				const Eigen::Matrix3Xf& raysA = m_jointRays[viewA][jIdx];
				const Eigen::Matrix3Xf& raysB = m_jointRays[viewB][jIdx];
				if (jointsA.cols() > 0 && jointsB.cols() > 0) {
					epi.setConstant(jointsA.cols(), jointsB.cols(), -1.f);
					for (int jaCandiIdx = 0; jaCandiIdx < epi.rows(); jaCandiIdx++) {
						for (int jbCandiIdx = 0; jbCandiIdx < epi.cols(); jbCandiIdx++) {
							const float dist = Line2LineDist(
								camAIter->second.eiPos, raysA.col(jaCandiIdx), camBIter->second.eiPos, raysB.col(jbCandiIdx));
							if (dist < m_maxEpiDist)
								epi(jaCandiIdx, jbCandiIdx) = 1.f - dist / m_maxEpiDist;
						}
					}

					if (m_normalizeEdges) {
						Eigen::VectorXf rowFactor = epi.rowwise().sum().transpose().cwiseMax(1.f);
						Eigen::VectorXf colFactor = epi.colwise().sum().cwiseMax(1.f);
						for (int i = 0; i < rowFactor.size(); i++)
							epi.row(i) /= rowFactor[i];

						for (int i = 0; i < colFactor.size(); i++)
							epi.col(i) /= colFactor[i];
					}
					m_epiEdges[jIdx][viewB][viewA] = epi.transpose();

				}
			}
		}
	}
}


void Associater::CalcTempEdges()
{
	const SkelDef& def = GetSkelDef(m_type);
#pragma omp parallel for
	for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
		auto camIter = m_cams.begin();
		for (int view = 0; view < m_cams.size(); view++, camIter++) {
			Eigen::MatrixXf& temp = m_tempEdges[jIdx][view];
			const Eigen::Matrix3Xf& rays = m_jointRays[view][jIdx];
			if (m_skels3dPrev.size() > 0 && rays.cols() > 0) {
				temp.setConstant(m_skels3dPrev.size(), rays.cols(), -1.f);
				int pIdx = 0;
				for (auto skelIter = m_skels3dPrev.begin(); skelIter != m_skels3dPrev.end(); skelIter++, pIdx++) {
					if (skelIter->second(3, jIdx) > FLT_EPSILON) {
						for (int jCandiIdx = 0; jCandiIdx < temp.cols(); jCandiIdx++) {
							const float dist = Point2LineDist(skelIter->second.col(jIdx).head(3), camIter->second.eiPos, rays.col(jCandiIdx));
							if (dist < m_maxTempDist)
								temp(pIdx, jCandiIdx) = 1.f - dist / m_maxTempDist;
						}
					}
				}

				if (m_normalizeEdges) {
					Eigen::VectorXf rowFactor = temp.rowwise().sum().transpose().cwiseMax(1.f);
					Eigen::VectorXf colFactor = temp.colwise().sum().cwiseMax(1.f);
					for (int i = 0; i < rowFactor.size(); i++)
						temp.row(i) /= rowFactor[i];

					for (int i = 0; i < colFactor.size(); i++)
						temp.col(i) /= colFactor[i];
				}
			}
		}
	}
}


void Associater::CalcSkels2d()
{
	const SkelDef& def = GetSkelDef(m_type);

	// filter person map
	for (auto personIter = std::next(m_personsMap.begin(), m_skels3dPrev.size()); personIter != m_personsMap.end(); ) {
		if ((personIter->second.array() >= 0).count() >= m_minAsgnCnt)
			personIter++;
		else {
			// erase
			const Eigen::MatrixXi& person = personIter->second;
			for (int view = 0; view < m_cams.size(); view++)
				for (int jIdx = 0; jIdx < def.jointSize; jIdx++)
					if (person(jIdx, view) != -1)
						m_assignMap[view][jIdx][person(jIdx, view)] = -1;
			personIter = m_personsMap.erase(personIter);
		}
	}

	// set identity
	m_skels2d.clear();
	for (const auto& person : m_personsMap) {
		const int identity = person.first < m_skels3dPrev.size() ? std::next(m_skels3dPrev.begin(), person.first)->first
			: m_skels2d.empty() ? 0 : m_skels2d.rbegin()->first + 1;
		Eigen::Matrix3Xf skel2d = Eigen::Matrix3Xf::Zero(3, m_cams.size() * def.jointSize);
		for (int view = 0; view < m_cams.size(); view++) {
			for (int jIdx = 0; jIdx < def.jointSize; jIdx++) {
				const int index = person.second(jIdx, view); {
					if (index != -1)
						skel2d.col(view * def.jointSize + jIdx) = m_detections[view].joints[jIdx].col(index);
				}
			}
		}
		m_skels2d.insert(std::make_pair(identity, skel2d));
	}
}

