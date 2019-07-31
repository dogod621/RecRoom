#pragma once

#include "SamplerPcMLS.h"

namespace RecRoom
{
	template<class PointType>
	void SamplerPcMLS<PointType>::ImplementProcess(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<PointType> & output) const
	{
		MovingLeastSquares<PointType, PointType> mls(searchRadius, order, projectionMethod, upsampleMethod, computeNormals);

		if (upsampleMethod == MLSUpsamplingMethod::DISTINCT_CLOUD)
		{
			if (distinctSampler)
			{
				PTR(Pc<PointType>) distinctCloud (new Pc<PointType>);
				distinctSampler->Process(searchSurface, input, filter, *distinctCloud);
				mls.setDistinctCloud(distinctCloud);
			}
			else
			{
				THROW_EXCEPTION("Use DISTINCT_CLOUD but not set distinctSampler.")
			}
		}

		mls.setSearchMethod(boost::const_pointer_cast<Acc<PointType>>(searchSurface)); // trick, already ensure it won't be modified 
		mls.setInputCloud(input);
		mls.setIndices(filter);
		mls.process(output);

		// Interpolate missing field
		PTR(Pc<PointType>) tempOutput(new Pc<PointType>);
		(*tempOutput) = output;
		Pc<PointType> temp;
		fieldInterpolator->Process(searchSurface, tempOutput, nullptr, temp);

		for (std::size_t px = 0; px < temp.size(); ++px)
		{
			PointType& tarP = (*tempOutput)[px];
			PointType& srcP = temp[px];

			float tempX = tarP.x;
			float tempY = tarP.y;
			float tempZ = tarP.z;
			float tempNX = tarP.normal_x;
			float tempNY = tarP.normal_y;
			float tempNZ = tarP.normal_z;

			tarP = srcP;

			tarP.x = tempX;
			tarP.y = tempY;
			tarP.z = tempZ;
			if (computeNormals)
			{
				if ((tempNX * srcP.normal_x + tempNY * srcP.normal_y + tempNZ * srcP.normal_z) > 0)
				{
					tarP.normal_x = tempNX;
					tarP.normal_y = tempNY;
					tarP.normal_z = tempNZ;
				}
				else
				{
					tarP.normal_x = -tempNX;
					tarP.normal_y = -tempNY;
					tarP.normal_z = -tempNZ;
				}
			}
		}

		output = (*tempOutput);
	}
}