#pragma once

#include <pcl/filters/filter_indices.h>

#include "Common.h"
#include "Data.h"

namespace RecRoom
{
	template<class PointType>
	class OutlierRemover : public Data, public pcl::FilterIndices<PointType>
	{
	public:
		using Self = OutlierRemover<PointType>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

	public:
		virtual Ptr Clone() const { THROW_EXCEPTION("Interface is not implemented"); }
	};
}

#include "OutlierRemover.hpp"