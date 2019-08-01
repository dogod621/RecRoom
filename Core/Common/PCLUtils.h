#pragma once

#include <string>

#include <pcl/point_types.h>

#include "Common.h"

// 
namespace RecRoom
{
	int SaveAsPLY(const std::string& fileName, const Mesh& mesh, unsigned precision = 5, bool binary = true);

	void SyncMeshNormal(Mesh& mesh);
};

#include "PCLUtils.hpp"
