#pragma once

#include "BaseData.h"

namespace RecRoom
{
	template<class PointType>
	void PointCloudData<PointType>::FromJson(const nlohmann::json& j)
	{
		Base::FromJson(j);
		numPoints = j["numPoints"];
	}

	template<class PointType>
	void PointCloudData<PointType>::ToJson(nlohmann::json& j) const
	{
		Base::ToJson(j);
		j["numPoints"] = numPoints;
	}

	template<class PointType>
	void ScanData<PointType>::FromJson(const nlohmann::json& j)
	{
		Base::FromJson(j);
		scanner = Convert<Scanner, std::string>(j["scanner"].get<std::string>());
		serialNumber = j["serialNumber"];
	}

	template<class PointType>
	void ScanData<PointType>::ToJson(nlohmann::json& j) const
	{
		Base::ToJson(j);
		j["scanner"] = Convert<std::string, Scanner>(scanner);
		j["serialNumber"] = serialNumber;
	}
}