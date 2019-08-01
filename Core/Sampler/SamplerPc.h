#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPc : public SearchInputSurfaceProcessorPc2Pc<PointType, PointType>
	{
	public:
		using Interpolator = InterpolatorPc<PointType, PointType>;
		using InterpolatorNearest = InterpolatorPcNearest<PointType, PointType>;

	public:
		SamplerPc(CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest)) 
			: SearchInputSurfaceProcessorPc2Pc<PointType, PointType>(), fieldInterpolator(fieldInterpolator)
		{
			name = "SamplerPc";

			if (!fieldInterpolator)
			{
				THROW_EXCEPTION("fieldInterpolator is not set");
			}
		}

	public:
		CONST_PTR(Interpolator) getFieldInterpolator() const { return fieldInterpolator; };

		void setFieldInterpolator(const CONST_PTR(Interpolator)& v)
		{
			if (!v)
			{
				THROW_EXCEPTION("fieldInterpolator is not set");
			}
			else
			{
				fieldInterpolator = v;
			}
		};

	protected:
		CONST_PTR(Interpolator) fieldInterpolator;
	};
}

#include "SamplerPc.hpp"