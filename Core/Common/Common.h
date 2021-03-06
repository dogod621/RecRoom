#pragma once

#define _USE_MATH_DEFINES

#ifdef _OPENMP
#include <omp.h>
#endif

#include <iostream>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include <exception>
#include <Eigen/Core>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/console/print.h>
#include <pcl/point_cloud.h>

#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>

#include <pcl/search/organized.h>
#include <pcl/search/impl/organized.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/PolygonMesh.h>

#include "nlohmann/json.hpp"

namespace RecRoom
{
#define PTR(T) boost::shared_ptr<T> // std::shared_ptr<T>
#define CONST_PTR(T) boost::shared_ptr<const T> // std::shared_ptr<const T>

	struct Color
	{
		union
		{
			struct
			{
				unsigned char r;
				unsigned char g;
				unsigned char b;
				unsigned char a;
			};
			uint32_t rgba;
		};

		Color() : r(0), g(0), b(0), a(0) {}
	};

	struct ColorHDR
	{
		union
		{
			struct
			{
				float r;
				float g;
				float b;
			};
			float data[3];
		};

		ColorHDR() : r(0), g(0), b(0) {}
	};

	class Common
	{
	public:
		static std::mutex gLock;

		static bool GenFrame(const Eigen::Vector3d& notmal, Eigen::Vector3d& tangent, Eigen::Vector3d& bitangent);
		static bool GenFrame(const Eigen::Vector3f& notmal, Eigen::Vector3f& tangent, Eigen::Vector3f& bitangent);

	protected:
		static Eigen::Vector3d tempVec1_d;
		static Eigen::Vector3d tempVec2_d;
		static Eigen::Vector3d tempVec3_d;
		static Eigen::Vector3d tempVec4_d;

		static Eigen::Vector3f tempVec1_f;
		static Eigen::Vector3f tempVec2_f;
		static Eigen::Vector3f tempVec3_f;
		static Eigen::Vector3f tempVec4_f;
	};

#define FG_R std::string("\033[31m") // red
#define FG_G std::string("\033[32m") // green
#define FG_Y std::string("\033[33m") // yellow
#define FG_B std::string("\033[34m") // blue
#define FG_M std::string("\033[35m") // magenta
#define FG_C std::string("\033[36m") // cyan
#define FG_W std::string("\033[37m") // white
#define FG_N std::string("\033[39m")
#define BG_R std::string("\033[41m")
#define BG_G std::string("\033[42m")
#define BG_Y std::string("\033[43m")
#define BG_B std::string("\033[44m")
#define BG_M std::string("\033[45m")
#define BG_C std::string("\033[46m") 
#define BG_W std::string("\033[47m") 
#define BG_N std::string("\033[49m")

#define INFO_MESSAGE(file, line, func, message) (FG_W + std::string(message) + FG_N + " " + FG_G + std::string(file) + FG_N + " " + FG_B + std::to_string(line) + FG_N + " " + FG_C + std::string(func) + FG_N + ".").c_str()
#define WARNING_MESSAGE(file, line, func, message) (BG_Y + FG_W + std::string(message) + FG_N + " " + FG_G + std::string(file) + FG_N + " " + FG_B + std::to_string(line) + FG_N + " " + FG_C + std::string(func) + FG_N + "." + BG_N).c_str()
#define ERROR_MESSAGE(file, line, func, message) (BG_R + FG_W + std::string(message) + FG_N + " " + FG_G + std::string(file) + FG_N + " " + FG_B + std::to_string(line) + FG_N + " " + FG_C + std::string(func) + FG_N + "." + BG_N).c_str()

	class exception : public std::exception
	{
	public:
		exception(const char* file, int line, const char* func, const std::string& message_)
			: std::exception()
		{
			message = ERROR_MESSAGE(file, line, func, message_);
		}
		~exception() {}
		const char* what() const { return message.c_str(); }

	protected:
		std::string message;
	};

#define THROW_EXCEPTION(message) {throw RecRoom::exception(__FILE__, __LINE__, __FUNCTION__, message);}
#define PRINT_ERROR(message) {std::lock_guard<std::mutex> guard(RecRoom::Common::gLock);std::cout << ERROR_MESSAGE(__FILE__, __LINE__, __FUNCTION__, message) << std::endl;}
#define PRINT_WARNING(message) {std::lock_guard<std::mutex> guard(RecRoom::Common::gLock);std::cout << WARNING_MESSAGE(__FILE__, __LINE__, __FUNCTION__, message) << std::endl;}
#define PRINT_INFO(message) {std::lock_guard<std::mutex> guard(RecRoom::Common::gLock);std::cerr << INFO_MESSAGE(__FILE__, __LINE__, __FUNCTION__, message) << std::endl;}

	//
	using Flag = unsigned int;

	//
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

	template<class PointType>
	using Pc = pcl::PointCloud<PointType>;

	template<class PointType>
	using Acc = pcl::search::Search<PointType>;

	template<class PointType>
	using KDTree = pcl::search::KdTree<PointType>;

	using Mesh = pcl::PolygonMesh;

	using PcIndex = std::vector<int>;

	class ThreadAble
	{
	public:
		ThreadAble()
		{
			SetNumberOfThreads();
		}

		void SetNumberOfThreads(unsigned int numThreads = 0);
		unsigned int getNumThreads() const { return numThreads; }

	protected:
		unsigned int numThreads;
	};

	class DumpAble
	{
	public:
		DumpAble(const std::string& className, const boost::filesystem::path& filePath);

	public:
		boost::filesystem::path getFilePath() const { return filePath; }

	protected:
		std::string className;
		boost::filesystem::path filePath;

		virtual void Load();

		virtual void Dump() const;

		virtual void Load(const nlohmann::json& j) = 0;
		virtual void Dump(nlohmann::json& j) const = 0;

		virtual bool CheckExist() const;
	};

	class AsyncAble
	{
	public:
		AsyncAble(std::size_t asyncSize) : asyncSize(asyncSize) {}

	public:
		std::size_t getAsyncSize() const { return asyncSize; }

		void setAsyncSize(std::size_t asyncSize_) { asyncSize = asyncSize_; }

	protected:
		std::size_t asyncSize;
	};
}

#include "Common.hpp"