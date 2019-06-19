#pragma once

#include <pcl/pcl_base.h>

#include "Common.h"

namespace RecRoom
{
	template<class PointType>
	class Segmenter : public pcl::PCLBase<PointType>
	{
	public:
		using Base = pcl::PCLBase<PointType>; 
		using Self = Segmenter;
		using Ptr = boost::shared_ptr<Self>;
		using ConstPtr = boost::shared_ptr<const Self>;
		
	};
}

#include "BaseSegmenter.hpp"