#pragma once

#include <Eigen/Geometry> 
#include <pcl/point_cloud.h>

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
	};

	class Data3D : public Data
	{
	public:
		Data3D(CoodSys coodSys) 
			:coodSys(coodSys), transform(Eigen::Matrix4d::Identity()), orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)), position(Eigen::Vector3d(0.0, 0.0, 0.0)) {}

		inline CoodSys getCoodSys() const { return coodSys; }
		inline Eigen::Matrix4d getTransform() const { return transform; }
		inline Eigen::Quaterniond getOrientation() const { return orientation; }
		inline Eigen::Vector3d getPosition() const { return position; }

	protected:
		CoodSys coodSys;

		Eigen::Matrix4d transform;
		Eigen::Quaterniond orientation; // cache from transform
		Eigen::Vector3d position; // cache from transform
	};

	template<class PointType>
	class PointCloudData : public Data3D
	{
	public:
		PointCloudData(CoodSys coodSys): Data3D(coodSys) {}

		virtual void ToPointCloud(pcl::PointCloud<PointType>& scanCloud) { THROW_EXCEPTION("Interface is not implemented"); }

	};

	template<class PointType>
	class ScanData : public PointCloudData<PointType>
	{
	public:
		ScanData(CoodSys coodSys, Scanner scanner)
			: PointCloudData(coodSys), scanner(scanner) {}

		inline Scanner getScanner() const { return scanner; }

	protected:
		CoodSys coodSys;
		Scanner scanner;
	};
}