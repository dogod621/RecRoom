#pragma once

#include <pcl/pcl_base.h>

#include "Common.h"
#include "Data.h"

namespace RecRoom
{
	template<class PointTypeIn, class PointTypeOut>
	class Segmenter : public Data, public pcl::PCLBase<PointTypeIn>
	{
	public:
		using Self = Segmenter<PointTypeIn, PointTypeOut>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

	public:
		virtual Ptr Clone() const { THROW_EXCEPTION("Interface is not implemented"); }
		virtual double SearchRadius() const { return 0.0; }

		virtual void Extract(pcl::PointCloud<PointTypeOut>& out) { THROW_EXCEPTION("Interface is not implemented"); }
	};
}

#include "Segmenter.hpp"