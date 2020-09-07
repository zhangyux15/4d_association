#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>
#include <map>


enum SkelType
{
	SKEL_TYPE_NONE = -1,
	SKEL19,
	SKEL17,
	SKEL15,
	COCO18,
	BODY25,
	OPTITRACK21,
	SHELF15,
	MPIIHAND21,
	SKEL_TYPE_SIZE
};


struct SkelDef
{
	int jointSize;
	int pafSize = 0;
	int shapeSize = 0;
	Eigen::Matrix2Xi pafDict;
	Eigen::VectorXi parent;
	Eigen::VectorXi hierarchyMap;
	Eigen::Matrix2Xi drawBone;
};


struct SkelMapping
{
	Eigen::VectorXi jointMapping;
	Eigen::VectorXi pafMapping;
};


inline const SkelDef& GetSkelDef(const SkelType& type)
{
	static const std::vector<SkelDef> skelDefs = [] {
		std::vector<SkelDef> _skelDefs(SKEL_TYPE_SIZE);
		// SKEL19
		_skelDefs[SKEL19].jointSize = 19;
		_skelDefs[SKEL19].pafSize = 18;
		_skelDefs[SKEL19].shapeSize = 10;
		_skelDefs[SKEL19].pafDict.resize(2, 18);
		_skelDefs[SKEL19].pafDict << 1, 2, 7, 0, 0, 3, 8, 1, 5, 11, 5, 1, 6, 12, 6, 1, 14, 13,
			0, 7, 13, 2, 3, 8, 14, 5, 11, 15, 9, 6, 12, 16, 10, 4, 17, 18;
		_skelDefs[SKEL19].parent.resize(19);
		_skelDefs[SKEL19].parent << -1, 0, 0, 0, 1, 1, 1, 2, 3, 4, 4, 5, 6, 7, 8, 11, 12, 14, 13;
		_skelDefs[SKEL19].hierarchyMap.resize(19);
		_skelDefs[SKEL19].hierarchyMap << 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3;
		_skelDefs[SKEL19].drawBone.resize(2, 19);
		_skelDefs[SKEL19].drawBone << 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 11, 12, 13, 14,
			2, 3, 4, 5, 6, 5, 7, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18, 17;

		// SKEL17
		_skelDefs[SKEL17].jointSize = 17;
		_skelDefs[SKEL17].pafSize = 16;
		_skelDefs[SKEL17].shapeSize = 10;
		_skelDefs[SKEL17].pafDict.resize(2, 16);
		_skelDefs[SKEL17].pafDict <<
			1, 2, 7, 0, 0, 3, 8, 1, 5, 9, 1, 6, 10, 1, 12, 11,
			0, 7, 11, 2, 3, 8, 12, 5, 9, 13, 6, 10, 14, 4, 15, 16;
		_skelDefs[SKEL17].parent.resize(17);
		_skelDefs[SKEL17].parent << -1, 0, 0, 0, 1, 1, 1, 2, 3, 5, 6, 7, 8, 9, 10, 12, 11;
		_skelDefs[SKEL17].hierarchyMap.resize(17);
		_skelDefs[SKEL17].hierarchyMap << 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3;
		_skelDefs[SKEL17].drawBone.resize(2, 17);
		_skelDefs[SKEL17].drawBone << 0, 0, 1, 1, 1, 2, 2, 3, 3, 5, 6, 7, 8, 9, 10, 11, 12,
			2, 3, 4, 5, 6, 5, 7, 6, 8, 9, 10, 11, 12, 13, 14, 16, 15;


		// SKEL15
		_skelDefs[SKEL15].jointSize = 15;
		_skelDefs[SKEL15].pafSize = 14;
		_skelDefs[SKEL15].shapeSize = 10;
		_skelDefs[SKEL15].pafDict.resize(2, 14);
		_skelDefs[SKEL15].pafDict <<
			1, 2, 7, 0, 0, 3, 8, 1, 5, 9, 1, 6, 10, 1,
			0, 7, 11, 2, 3, 8, 12, 5, 9, 13, 6, 10, 14, 4;
		_skelDefs[SKEL15].parent.resize(15);
		_skelDefs[SKEL15].parent << -1, 0, 0, 0, 1, 1, 1, 2, 3, 5, 6, 7, 8, 9, 10;
		_skelDefs[SKEL15].hierarchyMap.resize(15);
		_skelDefs[SKEL15].hierarchyMap << 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3;
		_skelDefs[SKEL15].drawBone.resize(2, 15);
		_skelDefs[SKEL15].drawBone << 0, 0, 1, 1, 1, 2, 2, 3, 3, 5, 6, 7, 8, 9, 10,
			2, 3, 4, 5, 6, 5, 7, 6, 8, 9, 10, 11, 12, 13, 14;


		// COCO18
		_skelDefs[COCO18].jointSize = 18;
		_skelDefs[COCO18].pafSize = 19;
		_skelDefs[COCO18].shapeSize = 10;
		_skelDefs[COCO18].pafDict.resize(2, 19);
		_skelDefs[COCO18].pafDict << 1, 8, 9, 1, 11, 12, 1, 2, 3, 2, 1, 5, 6, 5, 1, 0, 0, 14, 15,
			8, 9, 10, 11, 12, 13, 2, 3, 4, 16, 5, 6, 7, 17, 0, 14, 15, 16, 17;
		_skelDefs[COCO18].drawBone.resize(2, 18);
		_skelDefs[COCO18].drawBone << 0, 0, 0, 1, 1, 2, 2, 3, 5, 5, 6, 8, 8, 9, 11, 12, 14, 15,
			1, 14, 15, 2, 5, 3, 8, 4, 6, 11, 7, 9, 11, 10, 12, 13, 16, 17;

		// BODY25
		_skelDefs[BODY25].jointSize = 25;
		_skelDefs[BODY25].pafSize = 26;
		_skelDefs[BODY25].shapeSize = 10;
		_skelDefs[BODY25].pafDict.resize(2, 26);
		_skelDefs[BODY25].pafDict << 1, 9, 10, 8, 8, 12, 13, 1, 2, 3, 2, 1, 5, 6, 5, 1, 0, 0, 15, 16, 14, 19, 14, 11, 22, 11,
			8, 10, 11, 9, 12, 13, 14, 2, 3, 4, 17, 5, 6, 7, 18, 0, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24;
		_skelDefs[BODY25].drawBone.resize(2, 25);
		_skelDefs[BODY25].drawBone << 0, 0, 0, 1, 1, 2, 2, 3, 5, 5, 6, 8, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 16, 19, 22,
			1, 15, 16, 2, 5, 3, 9, 4, 6, 12, 7, 9, 12, 10, 11, 22, 24, 13, 14, 19, 21, 17, 18, 20, 23;

		// OPTITRACK21
		_skelDefs[OPTITRACK21].jointSize = 21;
		_skelDefs[OPTITRACK21].drawBone.resize(2, 20);
		_skelDefs[OPTITRACK21].drawBone << 0, 0, 0, 1, 2, 2, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 16, 17, 18,
			1, 13, 16, 2, 3, 5, 9, 4, 6, 7, 8, 10, 11, 12, 14, 15, 19, 17, 18, 20;

		// SHELF15
		_skelDefs[SHELF15].jointSize = 15;
		_skelDefs[SHELF15].pafSize = 10;
		_skelDefs[SHELF15].pafDict.resize(2, 10);
		_skelDefs[SHELF15].pafDict << 9, 8, 10, 7, 3, 2, 4, 1, 12, 12,
			10, 7, 11, 6, 4, 1, 5, 0, 13, 14;
		_skelDefs[SHELF15].drawBone.resize(2, 16);
		_skelDefs[SHELF15].drawBone << 0, 1, 2, 2, 3, 3, 3, 4, 6, 7, 8, 8, 9, 9, 10, 12,
			1, 2, 8, 14, 4, 9, 14, 5, 7, 8, 9, 12, 10, 12, 11, 13;

		// MPIIHAND21
		_skelDefs[MPIIHAND21].jointSize = 21;
		_skelDefs[MPIIHAND21].drawBone.resize(2, 20);
		_skelDefs[MPIIHAND21].drawBone << 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 0, 13, 14, 15, 0, 17, 18, 19,
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20;

		return _skelDefs;
	}();
	return skelDefs[type];
}


inline const SkelMapping& GetSkelMapping(const SkelType& srcType, const SkelType& tarType)
{
	static const std::vector<std::vector<SkelMapping>> skelMappings = [] {
		std::vector<std::vector<SkelMapping>> _skelMappings(SKEL_TYPE_SIZE, std::vector<SkelMapping>(SKEL_TYPE_SIZE));

		// BODY25 <-> SKEL19
		_skelMappings[BODY25][SKEL19].jointMapping.resize(25);
		_skelMappings[BODY25][SKEL19].jointMapping << 4, 1, 5, 11, 15, 6, 12, 16, 0, 2, 7, 13, 3, 8, 14, -1, -1, 9, 10, 17, -1, -1, 18, -1, -1;
		_skelMappings[BODY25][SKEL19].pafMapping.resize(26);
		_skelMappings[BODY25][SKEL19].pafMapping << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, 16, -1, -1, 17, -1, -1;

		// BODY25 <-> SKEL17
		_skelMappings[BODY25][SKEL17].jointMapping.resize(25);
		_skelMappings[BODY25][SKEL17].jointMapping << 4, 1, 5, 9, 13, 6, 10, 14, 0, 2, 7, 11, 3, 8, 12, -1, -1, -1, -1, 15, -1, -1, 16, -1, -1;
		_skelMappings[BODY25][SKEL17].pafMapping.resize(26);
		_skelMappings[BODY25][SKEL17].pafMapping << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, -1, 10, 11, 12, -1, 13, -1, -1, -1, -1, 14, -1, -1, 15, -1, -1;

		// BODY25 <-> SKEL15
		_skelMappings[BODY25][SKEL15].jointMapping.resize(25);
		_skelMappings[BODY25][SKEL15].jointMapping << 4, 1, 5, 9, 13, 6, 10, 14, 0, 2, 7, 11, 3, 8, 12, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1;
		_skelMappings[BODY25][SKEL15].pafMapping.resize(26);
		_skelMappings[BODY25][SKEL15].pafMapping << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, -1, 10, 11, 12, -1, 13, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1;
		
		return _skelMappings;
	}();

	assert(skelMappings[srcType][tarType].jointMapping.size() > 0);
	return skelMappings[srcType][tarType];
}

