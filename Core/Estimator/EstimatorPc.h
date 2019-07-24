#pragma once

#include "Common/Common.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class EstimatorPc 
		: public SearchAnySurfaceProcesserPc2Pc<InPointType, OutPointType>
	{
	public:
		EstimatorPc(double searchRadius)
			: SearchAnySurfaceProcesserPc2Pc<InPointType, OutPointType>(),
			searchRadius(searchRadius)
		{
			if (searchRadius <= 0.0)
				THROW_EXCEPTION("searchRadius is not valid");
		}

	public:
		double getSearchRadius() const { return searchRadius; }

		void setSearchRadius(double v) 
		{ 
			if (v <= 0.0)
			{
				THROW_EXCEPTION("searchRadius is not valid");
			}
			else
				searchRadius = v; 
		}

	protected:
		double searchRadius;
	};
}

#include "EstimatorPc.hpp"