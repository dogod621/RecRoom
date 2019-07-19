#pragma once

#include <pcl/PolygonMesh.h>

#include "Common/Common.h"
#include "Common/Point.h"
#include "Sampler/SamplerPc.h"

namespace RecRoom
{
	class MesherPc
	{
	public:
		MesherPc(CONST_PTR(ResamplerPcMED) resampler = nullptr) : resampler(resampler){}

	public:
		virtual void Process(PTR(PcMED)& inV, pcl::PolygonMesh& out) const;

	protected:
		virtual void ToMesh(PTR(Pc<pcl::PointNormal>)& inV, PTR(KDTree<pcl::PointNormal>)& tree, pcl::PolygonMesh& out) const = 0;

	protected:
		CONST_PTR(ResamplerPcMED) resampler;
	};
}

#include "MesherPc.hpp"