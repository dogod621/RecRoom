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
		using Interpolator = SamplerPc<PointType>::Interpolator;
		using InterpolatorNearest = SamplerPc<PointType>::InterpolatorNearest;

	public:
		SamplerPcMLS(
			double searchRadius,
			int order = 2,
			MLSProjectionMethod projectionMethod = MLSProjectionMethod::SIMPLE,
			MLSUpsamplingMethod upsampleMethod = MLSUpsamplingMethod::MLSUpsamplingMethod_NONE,
			bool computeNormals = true,
			CONST_PTR(Pc<PointType>) distinctCloud = nullptr,
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: SamplerPc<PointType>(fieldInterpolator) ,
			searchRadius(searchRadius), 
			order(order),
			projectionMethod(projectionMethod), 
			upsampleMethod(upsampleMethod),
			computeNormals(computeNormals), 
			distinctCloud(distinctCloud),
		{
			name = "SamplerPcMLS";
		}

	protected:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& output) const;

		inline virtual bool InputPointValid(const PointType& p) const
		{
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z) &&
				pcl_isfinite(p.normal_x) &&
				pcl_isfinite(p.normal_y) &&
				pcl_isfinite(p.normal_z);
		}

		inline virtual bool OutPointValid(const PointType& p) const
		{
			if (computeNormals)
			{
				return pcl_isfinite(p.x) &&
					pcl_isfinite(p.y) &&
					pcl_isfinite(p.z) &&
					pcl_isfinite(p.normal_x) &&
					pcl_isfinite(p.normal_y) &&
					pcl_isfinite(p.normal_z) &&
					pcl_isfinite(p.curvature);
			}
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z);
		}

	public:
		double getSearchRadius() const { return searchRadius; }
		int getOrder() const { return order; }
		MLSProjectionMethod getProjectionMethod() const { return projectionMethod; }
		MLSUpsamplingMethod getUpsampleMethod() const { return upsampleMethod; }
		unsigned int getThreads() const { return threads; }
		bool getComputeNormals() const { return computeNormals; }
		CONST_PTR(Pc<PointType>) getDistinctSampler() const { return distinctCloud; };

		void setSearchRadius(double v) { searchRadius = v; }
		void setOrder(int v) { order = v; }
		void setProjectionMethod(MLSProjectionMethod v) { projectionMethod = v; }
		void setUpsampleMethod(MLSUpsamplingMethod v) { upsampleMethod = v; }
		void setThreads(unsigned int v) { threads = v; }
		void setComputeNormals(bool v) { computeNormals = v; }
		void setDistinctSampler(const CONST_PTR(Pc<PointType>)& v) { distinctCloud = v; };

	protected:
		double searchRadius;
		int order;
		MLSProjectionMethod projectionMethod;
		MLSUpsamplingMethod upsampleMethod;
		unsigned int threads;
		bool computeNormals;
		CONST_PTR(Pc<PointType>) distinctCloud;
	};
}

#include "SamplerPcMLS.hpp"