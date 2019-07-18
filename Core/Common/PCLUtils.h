#pragma once

#include <string>
#include <pcl/PolygonMesh.h>

#include "Common.h"

// 
namespace RecRoom
{
	int SaveAsPLY(const std::string &fileName, const pcl::PolygonMesh &mesh, unsigned precision = 5, bool binary = true);
};

#include "PCLUtils.hpp"
