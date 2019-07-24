#pragma once

#include <string>

#include "Common.h"

// 
namespace RecRoom
{
	int SaveAsPLY(const std::string &fileName, const Mesh &mesh, unsigned precision = 5, bool binary = true);
};

#include "PCLUtils.hpp"
