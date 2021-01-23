#pragma once
#include "associater.h"


class KruskalAssociater : public Associater
{
public:
	KruskalAssociater(const SkelType& type, const std::map<std::string, Camera>& cams);
	virtual void Associate() override;

	void SetEpiWeight(const float& _wEpi) { m_wEpi = _wEpi; }
	void SetTempWeight(const float& _wTemp) { m_wTemp = _wTemp; }
	void SetViewWeight(const float& _wView) { m_wView = _wView; }
	void SetPafWeight(const float& _wPaf) { m_wPaf = _wPaf; }
	void SetHierWeight(const float& _wHier) { m_wHier = _wHier; }
	void SetViewCntWelsh(const float& _cViewCnt) { m_cViewCnt = _cViewCnt; }
	void SetMinCheckCnt(const int& _minCheckCnt) { m_minCheckCnt = _minCheckCnt; }
	void SetNodeMultiplex(const bool& _nodeMultiplex) { m_nodeMultiplex = _nodeMultiplex; }

private:
	struct BoneClique
	{
		float score;
		int pafIdx;
		Eigen::VectorXi proposal;
		bool operator < (const BoneClique &b) const { return score < b.score; }
	};

	struct Voting
	{
		Eigen::Vector2i fst, sec, fstCnt, secCnt;
		std::map<int, Eigen::Vector2i> vote;
		void Parse();
	};

	std::vector<std::vector<int>> m_joint2paf;
	Eigen::VectorXi m_pafHier;
	int m_pafHierSize;
	std::vector<std::vector<std::vector<Eigen::Vector2i>>> m_boneNodes;		// m_nodes[pafIdx][camIdx][boneIdx] = (jaCandiIdx, jbCandiIdx)
	std::vector<std::vector<std::vector<Eigen::MatrixXf>>> m_boneEpiEdges;	// m_boneEpiEdges[pafIdx][viewA][viewB](boneAIdx, boneBIdx)
	std::vector<std::vector<Eigen::MatrixXf>> m_boneTempEdges;				// m_boneTempEdge[pafIdx][view](pIdx, boneIdx)
	
	float m_wEpi = 1.f;
	float m_wTemp = 3.f;
	float m_wView = 1.f;
	float m_wPaf = 1.f;
	float m_wHier = 0.5f;
	float m_cViewCnt = 2.f;
	int m_minCheckCnt = 2;
	bool m_nodeMultiplex = false;

	void CalcBoneNodes();
	void CalcBoneEpiEdges();
	void CalcBoneTempEdges();
	void EnumCliques(std::vector<BoneClique>& cliques);
	void PushClique(const int& pafIdx, const Eigen::VectorXi& proposal, std::vector<BoneClique>& cliques);
	void CalcCliqueScore(BoneClique& clique);
	int CheckJointCompatibility(const int& view, const int& jIdx, const int& candiIdx, const int& personIdx);
	int CheckPersonCompatibility(const int& masterIdx, const int& slaveIdx, const int& view);
	int CheckPersonCompatibility(const int& masterIdx, const int& slaveIdx);
	void MergePerson(const int& masterIdx, const int& slaveIdx);
	void AssignTopClique(std::vector<BoneClique>& cliques);
	void SpanTree();
	void DismemberPersons(std::vector<BoneClique>& cliques);
	void Clique2Voting(const BoneClique& clique, Voting& voting);
};


