#pragma once

#include "nlohmann/json.hpp"

#include "ReconstructorPc.h"

namespace RecRoom
{
	template<>
	inline ReconstructStatus Convert<ReconstructStatus, std::string>(const std::string& v)
	{
		if (v == "POINT_CLOUD") return ReconstructStatus::POINT_CLOUD;
		else if (v == "PC_NORMAL") return ReconstructStatus::PC_NORMAL;
		else if (v == "PC_ALBEDO") return ReconstructStatus::PC_ALBEDO;
		else if (v == "PC_SHARPNESS") return ReconstructStatus::PC_SHARPNESS;
		else if (v == "PC_SEGMENT") return ReconstructStatus::PC_SEGMENT;
		else if (v == "SEG_MATERIAL") return ReconstructStatus::SEG_MATERIAL;
		else if (v == "MESH_PREPROCESS") return ReconstructStatus::MESH_PREPROCESS;
		else if (v == "MESH") return ReconstructStatus::MESH;
		else if (v == "MESH_POSTPROCESS") return ReconstructStatus::MESH_POSTPROCESS;
		else return ReconstructStatus::ReconstructStatus_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, ReconstructStatus>(const ReconstructStatus& v)
	{
		switch (v)
		{
		case ReconstructStatus::POINT_CLOUD: return std::string("POINT_CLOUD"); break;
		case ReconstructStatus::PC_NORMAL: return std::string("PC_NORMAL"); break;
		case ReconstructStatus::PC_ALBEDO: return std::string("PC_ALBEDO"); break;
		case ReconstructStatus::PC_SHARPNESS: return std::string("PC_SHARPNESS"); break;
		case ReconstructStatus::PC_SEGMENT: return std::string("PC_SEGMENT"); break;
		case ReconstructStatus::SEG_MATERIAL: return std::string("SEG_MATERIAL"); break;
		case ReconstructStatus::MESH_PREPROCESS: return std::string("MESH_PREPROCESS"); break;
		case ReconstructStatus::MESH: return std::string("MESH"); break;
		case ReconstructStatus::MESH_POSTPROCESS: return std::string("MESH_POSTPROCESS"); break;
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
		if (v & ReconstructStatus::PC_NORMAL) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::PC_NORMAL));
		if (v & ReconstructStatus::PC_ALBEDO) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::PC_ALBEDO));
		if (v & ReconstructStatus::PC_SHARPNESS) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::PC_SHARPNESS));
		if (v & ReconstructStatus::PC_SEGMENT) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::PC_SEGMENT));
		if (v & ReconstructStatus::SEG_MATERIAL) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::SEG_MATERIAL));
		if (v & ReconstructStatus::MESH_PREPROCESS) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::MESH_PREPROCESS));
		if (v & ReconstructStatus::MESH) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::MESH));
		if (v & ReconstructStatus::MESH_POSTPROCESS) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::MESH_POSTPROCESS));
		return j;
	}
}