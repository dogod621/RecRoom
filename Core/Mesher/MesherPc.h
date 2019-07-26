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
		using Sampler = SamplerPc<PointType>;
		using Filter = FilterPc<PointType>;
		using Interpolator = InterpolatorPc<PointType, PointType>;
		using InterpolatorNearest = InterpolatorPcNearest<PointType, PointType>;

	public:
		MesherPc(
			CONST_PTR(Sampler) preprocessSampler = nullptr,
			CONST_PTR(Filter) preprocessFilter = nullptr,
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: SearchInputSurfaceProcessorPc<PointType, Mesh>(),
			preprocessSampler(preprocessSampler), preprocessFilter(preprocessFilter), fieldInterpolator(fieldInterpolator) 
		{
			name = "MesherPc";

			if (!fieldInterpolator)
				THROW_EXCEPTION("fieldInterpolator is not set");
		}

	public:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Mesh& output) const
		{
			return true;
		}

		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Mesh& output) const;

	protected:
		virtual void ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const = 0;

	public:
		CONST_PTR(Sampler) getPreprocessSampler() const { return preprocessSampler; };
		CONST_PTR(Filter) getPreprocessFilter() const { return preprocessFilter; };
		CONST_PTR(Interpolator) getFieldInterpolator() const { return fieldInterpolator; };

		void setPreprocessSampler(const CONST_PTR(Sampler)& v) { preprocessSampler = v; };
		void setPreprocessFilter(const CONST_PTR(Filter)& v) { preprocessFilter = v; };
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
		CONST_PTR(Sampler) preprocessSampler;
		CONST_PTR(Filter) preprocessFilter;
		CONST_PTR(Interpolator) fieldInterpolator;
	};
}

#include "MesherPc.hpp"