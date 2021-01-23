#pragma once
#include <cmath>
#include <fstream>
#include <iostream>
#include <type_traits>
#include <Eigen/Core>
#include <opencv2/core.hpp>


namespace MathUtil {
	// Linear Algebra
	template<typename T>
	inline Eigen::Matrix<T, 3, 3> Skew(const Eigen::Matrix<T, 3, 1>& vec)
	{
		Eigen::Matrix<T, 3, 3> skew;
		skew << 0, -vec.z(), vec.y(),
			vec.z(), 0, -vec.x(),
			-vec.y(), vec.x(), 0;
		return skew;
	}

	template<typename T>
	inline Eigen::Matrix<T, 3, 3> Rodrigues(const Eigen::Matrix<T, 3, 1>& vec)
	{
		const T theta = vec.norm();
		const Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();

		if (abs(theta) < 1e-5f)
			return I;
		else {
			const T c = std::cos(theta);
			const T s = std::sin(theta);
			const T itheta = 1 / theta;
			const Eigen::Matrix<T, 3, 1> r = vec / theta;
			return c * I + (1 - c) * r * r.transpose() + s * Skew(r);
		}
	}

	template<typename T>
	inline Eigen::Matrix<T, 3, 9> RodriguesJacobi(const Eigen::Matrix<T, 3, 1>& vec)
	{
		const T theta = vec.norm();
		Eigen::Matrix<T, 3, 9> dSkew;
		dSkew.setZero();
		dSkew(0, 5) = dSkew(1, 6) = dSkew(2, 1) = -1;
		dSkew(0, 7) = dSkew(1, 2) = dSkew(2, 3) = 1;
		if (abs(theta) < 1e-5f) {
			return -dSkew;
		}
		else {
			const T c = std::cos(theta);
			const T s = std::sin(theta);
			const T c1 = 1 - c;
			const T itheta = 1 / theta;
			const Eigen::Matrix<T, 3, 1> r = vec / theta;
			const Eigen::Matrix<T, 3, 3> rrt = r * r.transpose();
			const Eigen::Matrix<T, 3, 3> skew = Skew(r);
			const Eigen::Matrix<T, 3, 3> I = Eigen::Matrix3f::Identity();
			Eigen::Matrix<T, 3, 9> drrt;
			drrt << r.x() + r.x(), r.y(), r.z(), r.y(), 0, 0, r.z(), 0, 0,
				0, r.x(), 0, r.x(), r.y() + r.y(), r.z(), 0, r.z(), 0,
				0, 0, r.x(), 0, 0, r.y(), r.x(), r.y(), r.z() + r.z();
			Eigen::Matrix<T, 3, 9> jaocbi;
			Eigen::Matrix<T, 5, 1> a;
			for (int i = 0; i < 3; i++) {
				a << -s * r[i], (s - 2 * c1*itheta)*r[i], c1 * itheta, (c - s * itheta)*r[i], s * itheta;
				for (int j = 0; j < 3; j++)
					for (int k = 0; k < 3; k++)
						jaocbi(i, k + k + k + j) = (a[0] * I(j, k) + a[1] * rrt(j, k) +
							a[2] * drrt(i, j + j + j + k) + a[3] * skew(j, k) +
							a[4] * dSkew(i, j + j + j + k));
			}
			return jaocbi;
		}
	}


	// robust function
	template<typename T>
	inline T Welsch(const T& c, const T& _x)
	{
		const T x = _x / c;
		return 1 - exp(-x * x / 2);
	}

	template <typename T>
	Eigen::Matrix<T, -1, -1> LoadMat(const std::string& filename) {
		std::ifstream ifs(filename);
		if (!ifs.is_open()) {
			std::cerr << "can not open fie: " << filename;
			std::abort();
		}
		int rows, cols;
		ifs >> rows >> cols;
		Eigen::Matrix<T, -1, -1> mat(rows, cols);
		for (int i = 0; i < rows; i++)
			for (int j = 0; j < cols; j++)
				ifs >> mat(i, j);
		return mat;
	}

	template <typename T>
	inline void SaveMat(const Eigen::Matrix<T, -1, -1>& mat, const std::string& filename) {
		std::ofstream ofs(filename);
		ofs << mat.rows() << " " << mat.cols() << std::endl;
		ofs << mat << std::endl;
		ofs.close();
	}
}

