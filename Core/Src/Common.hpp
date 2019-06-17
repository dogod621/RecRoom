#pragma once

#include "Common.h"

namespace RecRoom
{
	template<>
	inline Scanner Convert<Scanner, std::string>(const std::string& v)
	{
		if (v == "BLK360") return Scanner::BLK360;
		else return Scanner::Scaner_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, Scanner>(const Scanner& v)
	{
		switch (v)
		{
		case Scanner::BLK360: return std::string("BLK360"); break;
		default: return std::string("UNKNOWN"); break;
		}
	}

	template<>
	inline LinearSolver Convert<LinearSolver, std::string>(const std::string& v)
	{
		if (v == "EIGEN_QR") return LinearSolver::EIGEN_QR;
		else if (v == "EIGEN_SVD") return LinearSolver::EIGEN_SVD;
		else if (v == "EIGEN_NE") return LinearSolver::EIGEN_NE;
		else return LinearSolver::LinearSolver_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, LinearSolver>(const LinearSolver& v)
	{
		switch (v)
		{
		case LinearSolver::EIGEN_QR: return std::string("EIGEN_QR"); break;
		case LinearSolver::EIGEN_SVD: return std::string("EIGEN_SVD"); break;
		case LinearSolver::EIGEN_NE: return std::string("EIGEN_NE"); break;
		default: return std::string("UNKNOWN"); break;
		}
	}
}