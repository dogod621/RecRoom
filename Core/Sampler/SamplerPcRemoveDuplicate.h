#pragma once

#include "SamplerPc.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPcRemoveDuplicate : public ResamplerPc<PointType>
	{
	public:
		SamplerPcRemoveDuplicate(double searchRadius, float minDistance)
			: ResamplerPc<PointType>(), searchRadius(searchRadius), minDistance(minDistance), sqrMinDistance(minDistance * minDistance) 
		{
			if (searchRadius <= minDistance)
				THROW_EXCEPTION("searchRadius <= minDistance");
		}

	public:
		virtual void Process(
			const PTR(Pc<PointType>) & inV,
			Pc<PointType> & outV) const;

	public:
		double getSearchRadius() const { return searchRadius; }
		float getMinDistance() const { return minDistance; }

		void setSearchRadius(double v) { searchRadius = v; }
		void setMinDistance(float v) { minDistance = v; sqrMinDistance = v * v; }
		
	protected:
		double searchRadius;
		float minDistance;
		float sqrMinDistance;
	};

	using SamplerPcRemoveDuplicateRAW = SamplerPcRemoveDuplicate<PointRAW>;
	using SamplerPcRemoveDuplicateMED = SamplerPcRemoveDuplicate<PointMED>;
	using SamplerPcRemoveDuplicateREC = SamplerPcRemoveDuplicate<PointREC>;
	using SamplerPcRemoveDuplicateNDF = SamplerPcRemoveDuplicate<PointNDF>;
	using SamplerPcRemoveDuplicateLF = SamplerPcRemoveDuplicate<PointLF>;
}

#include "SamplerPcRemoveDuplicate.hpp"