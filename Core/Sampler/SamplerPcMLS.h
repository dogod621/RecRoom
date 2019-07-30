#pragma once

#include "Common/MovingLeastSquares.h"
#include "Interpolator/InterpolatorPcNearest.h"

#include "SamplerPc.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPcMLS : public SamplerPc<PointType>
	{
	public:
		using Sampler = SamplerPc<PointType>;
		using Interpolator = InterpolatorPc<PointType, PointType>;
		using InterpolatorNearest = InterpolatorPcNearest<PointType, PointType>;

	public:
		SamplerPcMLS(
			double searchRadius,
			int order = 2,
			MLSProjectionMethod projectionMethod = MLSProjectionMethod::SIMPLE,
			MLSUpsamplingMethod upsampleMethod = MLSUpsamplingMethod::MLSUpsamplingMethod_NONE,
			bool computeNormals = true,
			CONST_PTR(Sampler) distinctSampler = nullptr,
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: SamplerPc<PointType>() ,
			searchRadius(searchRadius), 
			order(order),
			projectionMethod(projectionMethod), 
			upsampleMethod(upsampleMethod),
			computeNormals(computeNormals), 
			distinctSampler(distinctSampler),
			fieldInterpolator(fieldInterpolator)
		{
			name = "SamplerPcMLS";

			if (!fieldInterpolator)
			{
				THROW_EXCEPTION("fieldInterpolator is not set");
			}
		}

	protected:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& output) const;

	public:
		double getSearchRadius() const { return searchRadius; }
		int getOrder() const { return order; }
		MLSProjectionMethod getProjectionMethod() const { return projectionMethod; }
		MLSUpsamplingMethod getUpsampleMethod() const { return upsampleMethod; }
		unsigned int getThreads() const { return threads; }
		bool getComputeNormals() const { return computeNormals; }
		CONST_PTR(Sampler) getDistinctSampler() const { return distinctSampler; };
		CONST_PTR(Interpolator) getFieldInterpolator() const { return fieldInterpolator; };

		void setSearchRadius(double v) { searchRadius = v; }
		void setOrder(int v) { order = v; }
		void setProjectionMethod(MLSProjectionMethod v) { projectionMethod = v; }
		void setUpsampleMethod(MLSUpsamplingMethod v) { upsampleMethod = v; }
		void setThreads(unsigned int v) { threads = v; }
		void setComputeNormals(bool v) { computeNormals = v; }
		void setDistinctSampler(const CONST_PTR(Sampler)& v) { distinctSampler = v; };
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
		double searchRadius;
		int order;
		MLSProjectionMethod projectionMethod;
		MLSUpsamplingMethod upsampleMethod;
		unsigned int threads;
		bool computeNormals;
		CONST_PTR(Sampler) distinctSampler;
		CONST_PTR(Interpolator) fieldInterpolator;
	};
}

#include "SamplerPcMLS.hpp"