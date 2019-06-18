#pragma once

#include <Eigen/Geometry> 
#include <pcl/point_cloud.h>

#include "nlohmann/json.hpp"

#include "Common.h"
#include "Coordinate.h"
#include "PointType.h"

namespace RecRoom
{
	//
	class Data
	{
	public:
		Data() {}
		virtual void LoadFile(const boost::filesystem::path& filePath) { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void SaveFile(const boost::filesystem::path& filePath) { THROW_EXCEPTION("Interface is not implemented"); }
		virtual nlohmann::json DumpToJson() { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void LoadFromJson(const nlohmann::json& j) { THROW_EXCEPTION("Interface is not implemented"); }
	};

	class Data3D : public Data
	{
	public:
		Data3D() : transform(Eigen::Matrix4d::Identity()), orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)), position(Eigen::Vector3d(0.0, 0.0, 0.0)) {}
	
	public:
		inline Eigen::Matrix4d getTransform() const { return transform; }
		inline Eigen::Quaterniond getOrientation() const { return orientation; }
		inline Eigen::Vector3d getPosition() const { return position; }

	protected:
		Eigen::Matrix4d transform;
		Eigen::Quaterniond orientation; // cache from transform
		Eigen::Vector3d position; // cache from transform
	};

	template<class PointType>
	class PointCloudData : public Data3D
	{
	public:
		PointCloudData() : Data3D(), numPoints(0), numValidPoints(0) {}

		virtual void ToPointCloud(pcl::PointCloud<PointType>& scanCloud) { THROW_EXCEPTION("Interface is not implemented"); }

	public:
		inline std::size_t getNumPoints() const { return numPoints; }
		inline std::size_t getNumValidPoints() const { return numValidPoints; }

	protected:
		std::size_t numPoints;
		std::size_t numValidPoints;
	};

	template<class PointType, Scanner scanner>
	class ScanData : public PointCloudData<PointType>
	{
	public:
		ScanData() : PointCloudData(), serialNumber(-1) {}

	public:
		inline Scanner getScanner() const { return scanner; }
		inline int getSerialNumber() const { return serialNumber; }

	protected:
		int serialNumber;
	};
}