#pragma once

#include "BaseData.h"

namespace RecRoom
{
	template<class PointType>
	void PointCloudData<PointType>::FromJson(const nlohmann::json& j)
	{
		Data3D::FromJson(j);
		numPoints = j["numPoints"];
	}

	template<class PointType>
	void PointCloudData<PointType>::ToJson(nlohmann::json& j) const
	{
		Data3D::ToJson(j);
		j["numPoints"] = numPoints;
	}

	template<class PointType>
	void ScanData<PointType>::FromJson(const nlohmann::json& j)
	{
		PointCloudData<PointType>::FromJson(j);
		scanner = Convert<Scanner, std::string>(j["scanner"].get<std::string>());
		serialNumber = j["serialNumber"];
	}

	template<class PointType>
	void ScanData<PointType>::ToJson(nlohmann::json& j) const
	{
		PointCloudData<PointType>::ToJson(j);
		j["scanner"] = Convert<std::string, Scanner>(scanner);
		j["serialNumber"] = serialNumber;
	}
}