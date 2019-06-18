#pragma once

#include <pcl/pcl_base.h>

#include "Common.h"

namespace RecRoom
{
	template<class PointType>
	class Segmenter : public pcl::PCLBase<PointType>
	{
	public:
		using Ptr = boost::shared_ptr<Segmenter<PointType>>;
		using ConstPtr = boost::shared_ptr<const Segmenter<PointType>>;
	};
}

#include "BaseSegmenter.hpp"