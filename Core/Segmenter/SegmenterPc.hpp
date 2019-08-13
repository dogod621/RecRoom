#pragma once

#include "SegmenterPc.h"

namespace RecRoom
{
	template<class PointType>
	void SegmenterPc<PointType>::Process(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<PointType>& output,
		PcSoftLabel& cache) const
	{
		PTR(PcIndex) filter2 = GenFilter(input, filter);

		if (output.size() != filter2->size())
			output.resize(filter2->size());

		{
			std::stringstream ss;
			ss << name << " - Start - inSize: " << filter2->size();
			PRINT_INFO(ss.str());
		}

		if (ImplementCheck(searchSurface, input, filter, output))
			ImplementProcess(searchSurface, input, filter2, output, cache);
		else
			THROW_EXCEPTION(name + " - Not pass check");

		{
			std::stringstream ss;
			ss << name << " - End - inSize: " << filter2->size() << ", outSize: " << OutputSize(output);
			PRINT_INFO(ss.str());
		}

		//
		if (output.is_dense)
		{
			for (Pc<PointType>::iterator it = output.begin(); it != output.end(); ++it)
			{
				if (!OutputPointValid(*it))
				{
					output.is_dense = false;
					break;
				}
			}
		}
	}

	template<class PointType>
	void SegmenterPc<PointType>::ProcessInOut(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const PTR(Pc<PointType>)& inOut,
		const CONST_PTR(PcIndex)& filter,
		PcSoftLabel& cache) const
	{
		PTR(PcIndex) filter2 = GenFilter(inOut, filter);

		Pc<PointType> temp1;
		Pc<PointType> temp2;

		pcl::ExtractIndices<PointType> extract;
		extract.setInputCloud(inOut);
		extract.setIndices(filter2);
		extract.setNegative(false);
		extract.filter(temp1);

		temp2.resize(temp1.size());
		for (std::size_t px = 0; px < temp1.size(); ++px)
			temp2[px] = temp1[px];

		{
			std::stringstream ss;
			ss << name << " - Start - inSize: " << filter2->size();
			PRINT_INFO(ss.str());
		}

		if (ImplementCheck(searchSurface, inOut, filter, temp2))
			ImplementProcess(searchSurface, inOut, filter2, temp2, cache);
		else
			THROW_EXCEPTION(name + " - Not pass check");

		if (filter2->size() != temp2.size())
			THROW_EXCEPTION(name + " - INOUT size not match");

		{
			std::stringstream ss;
			ss << name << " - End - inSize: " << filter2->size() << ", outSize: " << OutputSize(temp2);
			PRINT_INFO(ss.str());
		}

		if (temp2.is_dense)
		{
			for (std::size_t idx = 0; idx < filter2->size(); ++idx)
			{
				(*inOut)[(*filter2)[idx]] = temp2[idx];

				if (!OutputPointValid(temp2[idx]))
					inOut->is_dense = false;
			}
		}
		else
		{
			for (std::size_t idx = 0; idx < filter2->size(); ++idx)
			{
				(*inOut)[(*filter2)[idx]] = temp2[idx];
			}
		}
	}
}