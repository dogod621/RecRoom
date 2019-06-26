#include <pcl/filters/extract_indices.h>

#include "EstimatorPc.h"

namespace RecRoom
{
	void EstimatorPc::Process(
		const PTR(AccMED)& searchMethod,
		const PTR(PcMED)& searchSurface,
		const PTR(PcMED)& inV,
		const PTR(PcIndex)& inIdx) const
	{
		if (inIdx)
		{
			PcMED temp;

			pcl::ExtractIndices<PointMED> extract;
			extract.setInputCloud(inV);
			extract.setIndices(inIdx);
			extract.setNegative(false);
			extract.filter(temp);

			this->Process(searchMethod, searchSurface, inV, inIdx, temp);

			//
			for (std::size_t px = 0; px < inIdx->size(); ++px)
			{
				(*inV)[(*inIdx)[px]] = temp[px];
			}
		}
		else
		{
			this->Process(searchMethod, searchSurface, inV, inIdx, *inV);
		}
	}
}
