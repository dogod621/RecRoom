#pragma once

#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <string>
#include <exception>
#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/console/print.h>

namespace RecRoom
{
#define INFO_MESSAGE(file, line, func, message) ("File:" + std::string(file) + ", Line:" + std::to_string(line) + ", Func:" + std::string(func)  + ", What:" + std::string(message) + ".").c_str()

	class exception : public std::exception
	{
	public:
		exception(const char *file, int line, const char *func, const std::string &message_)
			: std::exception()
		{
			message = INFO_MESSAGE(file, line, func, message_);
		}
		~exception() {}
		const char *what() const { return message.c_str(); }

	protected:
		std::string message;
	};

#define THROW_EXCEPTION(message) throw RecRoom::exception(__FILE__, __LINE__, __func__, message);
#define PRINT_ERROR(message) PCL_ERROR(INFO_MESSAGE(__FILE__, __LINE__, __func__, message));
#define PRINT_WARNING(message) PCL_WARN(INFO_MESSAGE(__FILE__, __LINE__, __func__, message));
#define PRINT_INFO(message) PCL_INFO(INFO_MESSAGE(__FILE__, __LINE__, __func__, message));

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
	template<class outType, class inType>
	inline outType Convert(const inType& v);

#define PTR(T) boost::shared_ptr<T> // std::shared_ptr<T>
#define CONST_PTR(T) boost::shared_ptr<const T> // std::shared_ptr<const T>
}

#include "Common.hpp"