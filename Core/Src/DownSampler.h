#pragma once

#include <pcl/filters/filter.h>

#include "Common.h"
#include "Data.h"

namespace RecRoom
{
	template<class PointType>
	class DownSampler : public Data, public pcl::Filter<PointType>
	{
	public:
		using Self = DownSampler<PointType>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

	public:
		virtual Ptr Clone() const { THROW_EXCEPTION("Interface is not implemented"); }
	};
}

#include "DownSampler.hpp"