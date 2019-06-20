#pragma once

#include "Data.h"

namespace RecRoom
{
	template<class PointType>
	void DataPointCloud<PointType>::FromJson(const nlohmann::json& j)
	{
		Data3D::FromJson(j);
		numPoints = j["numPoints"];
	}

	template<class PointType>
	void DataPointCloud<PointType>::ToJson(nlohmann::json& j) const
	{
		Data3D::ToJson(j);
		j["numPoints"] = numPoints;
	}

	template<class PointType>
	void DataScan<PointType>::FromJson(const nlohmann::json& j)
	{
		DataPointCloud<PointType>::FromJson(j);
		scanner = Convert<Scanner, std::string>(j["scanner"].get<std::string>());
		serialNumber = j["serialNumber"];
	}

	template<class PointType>
	void DataScan<PointType>::ToJson(nlohmann::json& j) const
	{
		DataPointCloud<PointType>::ToJson(j);
		j["scanner"] = Convert<std::string, Scanner>(scanner);
		j["serialNumber"] = serialNumber;
	}
}