#pragma once

#include "Common/MovingLeastSquares.h"

#include "SamplerPc.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPcMLS : public ResamplerPc<PointType>
	{
	public:
		SamplerPcMLS(
			double searchRadius,
			int order = 2,
			MLSProjectionMethod projectionMethod = MLSProjectionMethod::SIMPLE,
			MLSUpsamplingMethod upsampleMethod = MLSUpsamplingMethod::MLSUpsamplingMethod_NONE,
			unsigned int threads = 1,
			bool computeNormals = true) 
			: searchRadius(searchRadius), order(order),
			projectionMethod(projectionMethod), upsampleMethod(upsampleMethod),
			threads(threads), computeNormals(computeNormals),
			ResamplerPc<PointType>() {}

	public:
		virtual void Process(
			const PTR(Pc<PointType>) & inV,
			Pc<PointType> & outV) const;

	public:
		double getSearchRadius() const { return searchRadius; }
		int getOrder() const { return order; }
		MLSProjectionMethod getProjectionMethod() const { return projectionMethod; }
		MLSUpsamplingMethod getUpsampleMethod() const { return upsampleMethod; }
		unsigned int getThreads() const { return threads; }
		bool getComputeNormals() const { return computeNormals; }

		void setSearchRadius(double v) { searchRadius = v; }
		void setOrder(int v) { order = v; }
		void setProjectionMethod(MLSProjectionMethod v) { projectionMethod = v; }
		void setUpsampleMethod(MLSUpsamplingMethod v) { upsampleMethod = v; }
		void setThreads(unsigned int v) { threads = v; }
		void setComputeNormals(bool v) { computeNormals = v; }

	protected:
		double searchRadius;
		int order;
		MLSProjectionMethod projectionMethod;
		MLSUpsamplingMethod upsampleMethod;
		unsigned int threads;
		bool computeNormals;
	};

	using SamplerPcMLSRAW = SamplerPcMLS<PointRAW>;
	using SamplerPcMLSMED = SamplerPcMLS<PointMED>;
	using SamplerPcMLSREC = SamplerPcMLS<PointREC>;
}

#include "SamplerPcMLS.hpp"