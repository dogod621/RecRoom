#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"
#include "Common/Point.h"
#include "Sampler/SamplerPc.h"
#include "Filter/FilterPc.h"
#include "Interpolator/InterpolatorPcNearest.h"

namespace RecRoom
{
	template<class PointType>
	class MesherPc : public SearchInputSurfaceProcessorPc<PointType, Mesh>
	{
	public:
		using Interpolator = InterpolatorPc<PointType, PointType>;
		using InterpolatorNearest = InterpolatorPcNearest<PointType, PointType>;

	public:
		MesherPc(
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: SearchInputSurfaceProcessorPc<PointType, Mesh>(),
			fieldInterpolator(fieldInterpolator) 
		{
			name = "MesherPc";

			if (!fieldInterpolator)
				THROW_EXCEPTION("fieldInterpolator is not set");
		}

	public:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Mesh& output) const;

	public:
		inline virtual bool InputPointValid(const PointType& p) const
		{
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z) &&
				pcl_isfinite(p.normal_x) &&
				pcl_isfinite(p.normal_y) &&
				pcl_isfinite(p.normal_z);
		}

	protected:
		virtual void ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const = 0;

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

#include "MesherPc.hpp"