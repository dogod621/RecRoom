#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include <string>
#include <exception>
#include <Eigen/Core>

namespace RecRoom
{
	class exception : public std::exception
	{
	public:
		exception(const std::string &msg, const char *file, int line) 
			: std::exception()
		{
			std::ostringstream o;
			o << file << ":" << line << "-" << msg;
			message = o.str();
		}
		~exception() {}
		const char *what() const { return message.c_str(); }

	protected:
		std::string message;
	};

#define THROW_EXCEPTION(message) throw RecRoom::exception(message, __FILE__, __LINE__);

	//
	using Flag = unsigned int;

	//
	enum Scanner : Flag
	{
		Scaner_UNKNOWN = 0,

		BLK360 = 1,
	};

	enum LinearSolver : Flag
	{
		LinearSolver_UNKNOWN = 0,

		EIGEN_QR = 1,
		EIGEN_SVD = 2,
		EIGEN_NE = 3,
	};

	//
	template<class outType, class inType >
	inline outType Convert(inType v);
}

#include "Common.hpp"