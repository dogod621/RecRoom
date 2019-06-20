#pragma once

#include <pcl/features/feature.h>

#include "Common.h"
#include "Data.h"

namespace RecRoom
{
	template<class PointTypeIn, class PointTypeOut>
	class AlbedoEstimater : public Data, public pcl::Feature<PointTypeIn, PointTypeOut>
	{
	public:
		using Self = AlbedoEstimater<PointTypeIn, PointTypeOut>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

	public:
		virtual Ptr Clone() const { THROW_EXCEPTION("Interface is not implemented"); }
	};
}

#include "AlbedoEstimater.hpp"