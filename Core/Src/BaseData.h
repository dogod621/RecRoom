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
		using Self = Data;
		using Ptr = boost::shared_ptr<Self>;
		using ConstPtr = boost::shared_ptr<const Self>;

	public:
		Data() {}

	public:
		virtual void FromFile(const boost::filesystem::path& filePath) { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ToFile(const boost::filesystem::path& filePath) const { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void FromJson(const nlohmann::json& j) { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ToJson(nlohmann::json& j) const { THROW_EXCEPTION("Interface is not implemented"); }
	};

	class Data3D : public Data
	{
	public:
		using Base = Data;
		using Self = Data3D;
		using Ptr = boost::shared_ptr<Self>;
		using ConstPtr = boost::shared_ptr<const Self>;
		
	public:
		Data3D() : Base(), transform(Eigen::Matrix4d::Identity()), orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)), position(Eigen::Vector3d(0.0, 0.0, 0.0)) {}
	
	public:
		virtual void FromJson(const nlohmann::json& j);
		 void ToJson(nlohmann::json& j) const;

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
		using Base = Data3D;
		using Self = PointCloudData<PointType>;
		using Ptr = boost::shared_ptr<Self>;
		using ConstPtr = boost::shared_ptr<const Self>;
		
	public:
		PointCloudData() : Base(), numPoints(0) {}

	public:
		virtual void FromJson(const nlohmann::json& j);
		virtual void ToJson(nlohmann::json& j) const;

		virtual void FromPointCloud(const pcl::PointCloud<PointType>& scanCloud) { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ToPointCloud(pcl::PointCloud<PointType>& scanCloud) const { THROW_EXCEPTION("Interface is not implemented"); }

	public:
		inline std::size_t getNumPoints() const { return numPoints; }

	protected:
		std::size_t numPoints;
	};

	template<class PointType>
	class ScanData : public PointCloudData<PointType>
	{
	public:
		using Base = PointCloudData<PointType>;
		using Self = ScanData;
		using Ptr = boost::shared_ptr<Self>;
		using ConstPtr = boost::shared_ptr<const Self>;
		
	public:
		ScanData() : Base(), scanner(Scanner::Scaner_UNKNOWN), serialNumber(-1) {}

	public:
		virtual void FromJson(const nlohmann::json& j);
		virtual void ToJson(nlohmann::json& j) const;

	public:
		inline Scanner getScanner() const { return scanner; }
		inline int getSerialNumber() const { return serialNumber; }

	protected:
		Scanner scanner;
		int serialNumber;
	};
}

#include "BaseData.hpp"