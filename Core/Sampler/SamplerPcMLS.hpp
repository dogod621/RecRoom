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
		PTR(Acc<PointType>) searchSurface2;
		PTR(Pc<PointType>) input2;
		PTR(PcIndex) filter2;
		{
			if (upsampleMethod == MLSUpsamplingMethod::DISTINCT_CLOUD)
			{
				if (distinctSampler)
				{
					searchSurface2 = PTR(Acc<PointType>)(new KDTree<PointType>);
					input2 = PTR(Pc<PointType>)(new Pc<PointType>);
					filter2 = nullptr;

					distinctSampler->Process(searchSurface, input, filter, *input2);

					searchSurface2->setInputCloud(input2);
				}
				else
				{
					THROW_EXCEPTION("Use DISTINCT_CLOUD but not set distinctSampler.")
				}
			}
			else
			{
				searchSurface2 = boost::const_pointer_cast<Acc<PointType>>(searchSurface);
				input2 = boost::const_pointer_cast<Pc<PointType>>(input);
				filter2 = boost::const_pointer_cast<PcIndex>(filter);
			}
		}

		MovingLeastSquares<PointType, PointType> mls(searchRadius, order, projectionMethod, upsampleMethod, computeNormals);

		mls.setSearchMethod(boost::const_pointer_cast<Acc<PointType>>(searchSurface)); // trick, already ensure it won't be modified 
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