#pragma once

#include "nlohmann/json.hpp"

#include "ReconstructorPc.h"

namespace RecRoom
{
	template<>
	inline ReconstructStatus Convert<ReconstructStatus, std::string>(const std::string& v)
	{
		if (v == "POINT_CLOUD") return ReconstructStatus::POINT_CLOUD;
		else if (v == "PC_MATERIAL") return ReconstructStatus::PC_MATERIAL;
		else if (v == "PC_SEGMENT") return ReconstructStatus::PC_SEGMENT;
		else if (v == "SEG_MATERIAL") return ReconstructStatus::SEG_MATERIAL;
		else if (v == "MESH") return ReconstructStatus::MESH;
		else return ReconstructStatus::ReconstructStatus_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, ReconstructStatus>(const ReconstructStatus& v)
	{
		switch (v)
		{
		case ReconstructStatus::POINT_CLOUD: return std::string("POINT_CLOUD"); break;
		case ReconstructStatus::PC_MATERIAL: return std::string("PC_MATERIAL"); break;
		case ReconstructStatus::PC_SEGMENT: return std::string("PC_SEGMENT"); break;
		case ReconstructStatus::SEG_MATERIAL: return std::string("SEG_MATERIAL"); break;
		case ReconstructStatus::MESH: return std::string("MESH"); break;
		default: return std::string("UNKNOWN"); break;
		}
	}

	template<>
	inline ReconstructStatus Convert<ReconstructStatus, nlohmann::json>(const nlohmann::json& v)
	{
		ReconstructStatus r = ReconstructStatus::ReconstructStatus_UNKNOWN;
		for (nlohmann::json::const_iterator it = v.begin(); it != v.end(); it++)
			r = (ReconstructStatus)(r | Convert<ReconstructStatus, std::string>(*it));
		return r;
	}

	template<>
	inline nlohmann::json Convert<nlohmann::json, ReconstructStatus>(const ReconstructStatus& v)
	{
		nlohmann::json j;
		if (v & ReconstructStatus::POINT_CLOUD) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::POINT_CLOUD));
		if (v & ReconstructStatus::PC_MATERIAL) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::PC_MATERIAL));
		if (v & ReconstructStatus::PC_SEGMENT) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::PC_SEGMENT));
		if (v & ReconstructStatus::SEG_MATERIAL) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::SEG_MATERIAL));
		if (v & ReconstructStatus::MESH) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::MESH));
		return j;
	}
}