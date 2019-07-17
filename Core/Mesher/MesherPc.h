#pragma once

#include <pcl/PolygonMesh.h>

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class MesherPc
	{
	public:
		MesherPc() {}

	public:
		virtual void Process(PTR(PcMED)& inV, pcl::PolygonMesh& out) const;

	protected:
		virtual void ToMesh(PTR(PcREC)& inV, PTR(KDTreeREC)& tree, pcl::PolygonMesh& out) const = 0;
	};
}

#include "MesherPc.hpp"