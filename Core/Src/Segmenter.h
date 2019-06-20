#pragma once

#include <pcl/pcl_base.h>

#include "Common.h"
#include "Data.h"

namespace RecRoom
{
	template<class PointType>
	class Segmenter : public Data, public pcl::PCLBase<PointType>
	{
	public:
		using Self = Segmenter<PointType>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

	public:
		virtual Ptr Clone() const { THROW_EXCEPTION("Interface is not implemented"); }
	};
}

#include "Segmenter.hpp"