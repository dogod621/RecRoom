#pragma once

#include <pcl/surface/processing.h>

#include "Common.h"
#include "Data.h"

namespace RecRoom
{
	template<class PointTypeIn, class PointTypeOut>
	class SurfaceProcesser : public Data, public pcl::CloudSurfaceProcessing<PointTypeIn, PointTypeOut>
	{
	public:
		using Self = SurfaceProcesser<PointTypeIn, PointTypeOut>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

	public:
		virtual Ptr Clone() const { THROW_EXCEPTION("Interface is not implemented"); }
		virtual double SearchRadius() const { return 0.0; }
	};
}

#include "SurfaceProcesser.hpp"