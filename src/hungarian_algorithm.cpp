#include "hungarian_algorithm.h"
#include <vector>
#include <algorithm>
#include <Eigen/Eigen>
#include <iostream>
#include <set>
#include <string>


// refer https://blog.csdn.net/Wonz5130/article/details/80678410
std::vector<std::pair<float, Eigen::Vector2i>> HungarianAlgorithm(const Eigen::MatrixXf& _hungarianMat)
{
	int rowSize = _hungarianMat.rows();
	int colSize = _hungarianMat.cols();
	int matSize = rowSize > colSize ? rowSize : colSize;

	if (matSize == 0)
		return std::vector<std::pair<float, Eigen::Vector2i>>();

	Eigen::MatrixXf hungarianMat = Eigen::MatrixXf::Zero(matSize, matSize);
	hungarianMat.block(0, 0, rowSize, colSize) = _hungarianMat;

	// step1
	hungarianMat.colwise() -= hungarianMat.rowwise().minCoeff();

	// step2
	hungarianMat.rowwise() -= hungarianMat.colwise().minCoeff();

	while (true)
	{
		// step 3.1
		Eigen::MatrixXi zeroMap = Eigen::MatrixXi::Zero(matSize, matSize);

		// generate zeroMask
		std::vector<int> zeroCount(2 * matSize, 0);		// first zero count of row and then zero count of col
		for (int row = 0; row < matSize; row++)
			for (int col = 0; col < matSize; col++)
				if (fabs(hungarianMat(row, col)) < FLT_EPSILON) {
					zeroMap(row, col) = 1;
					zeroCount[row]++;
					zeroCount[matSize + col]++;
				}

		for (auto& count : zeroCount)
			count = count == 0 ? matSize + 1 : count;

		// find optimum result
		Eigen::MatrixXi markMap = zeroMap;
		std::vector<std::pair<int, int>> keyElem;

		while (true) {
			auto iter = std::min_element(zeroCount.begin(), zeroCount.end());
			if (*iter == matSize + 1)
				break;

			int index = std::distance(zeroCount.begin(), iter);
			int keyRow, keyCol;
			if (index < matSize) {	// row
				keyRow = index;
				for (keyCol = 0; markMap(keyRow, keyCol) != 1; keyCol++) {}
			}
			else { // col
				keyCol = index - matSize;
				for (keyRow = 0; markMap(keyRow, keyCol) != 1; keyRow++) {}
			}

			// mark
			keyElem.emplace_back(std::make_pair(keyRow, keyCol));
			markMap(keyRow, keyCol) = 2;
			zeroCount[keyRow] = matSize + 1;
			zeroCount[matSize + keyCol] = matSize + 1;

			for (int i = 0; i < matSize; i++) {
				if ((markMap(keyRow, i) == 1)) {
					markMap(keyRow, i) = -1;
					zeroCount[i + matSize] = zeroCount[i + matSize] == 1 ? matSize + 1 : zeroCount[i + matSize] - 1;
				}

				if ((markMap(i, keyCol) == 1)) {
					markMap(i, keyCol) = -1;
					zeroCount[i] = zeroCount[i] == 1 ? matSize + 1 : zeroCount[i] - 1;
				}
			}
		}

		if (keyElem.size() != matSize)
		{
			std::set<int> keyRows, keyCols;
			for (int row = 0; row < matSize; row++)
				if (markMap.row(row).maxCoeff() != 2)
					keyRows.insert(row);

			while (true) {
				bool updateFlag = false;
				for (const int keyRow : keyRows)
					for (int keyCol = 0; keyCol < matSize && (!updateFlag); keyCol++)
						if (zeroMap(keyRow, keyCol) == 1)
							updateFlag |= keyCols.insert(keyCol).second;

				for (const int keyCol : keyCols)
					for (int keyRow = 0; keyRow < matSize; keyRow++)
						if (markMap(keyRow, keyCol) == 2)
							updateFlag |= keyRows.insert(keyRow).second;

				if (!updateFlag)
					break;
			}

			std::vector<bool> linedRows(matSize, true), linedCols(matSize, false);
			for (auto& keyRow : keyRows)
				linedRows[keyRow] = false;
			for (auto& keyCol : keyCols)
				linedCols[keyCol] = true;

			std::vector<float> uncoveredValues;
			for (int row = 0; row < matSize; row++)
				for (int col = 0; col < matSize; col++)
					if ((!linedRows[row]) && (!linedCols[col]))
						uncoveredValues.emplace_back(hungarianMat(row, col));

			float minValue = *std::min_element(uncoveredValues.begin(), uncoveredValues.end());

			Eigen::VectorXf minValueFlat(matSize);
			minValueFlat.setConstant(minValue);

			for (auto& keyRow : keyRows)
				hungarianMat.row(keyRow) -= minValueFlat;

			for (auto& keyCol : keyCols)
				hungarianMat.col(keyCol) += minValueFlat;
		}
		else {
			// success
			// std::cout << hungarianMat << std::endl << std::endl;
			std::vector<std::pair<float, Eigen::Vector2i>> matchPairs;
			for (const auto& elem : keyElem)
				if ((elem.first < rowSize) && (elem.second < colSize))
					matchPairs.emplace_back(std::make_pair(_hungarianMat(elem.first, elem.second), Eigen::Vector2i(elem.first, elem.second)));
			return matchPairs;
		}
	}
}
