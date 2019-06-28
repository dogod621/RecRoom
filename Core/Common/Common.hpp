#pragma once

#include "Common.h"

namespace RecRoom
{
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

	template<>
	inline Color Convert<Color, uint32_t>(const uint32_t& v)
	{
		// 4-byte Integer Hashing: http://burtleburtle.net/bob/hash/integer.html
		Color c;
		c.rgba = (v ^ 61) ^ (v >> 16);
		c.rgba = c.rgba + (c.rgba << 3);
		c.rgba = c.rgba ^ (c.rgba >> 4);
		c.rgba = c.rgba * 0x27d4eb2d;
		c.rgba = c.rgba ^ (c.rgba >> 15);
		return c;
	}
}