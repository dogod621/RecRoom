#include "SamplerPc.h"

namespace RecRoom
{
	void SamplerPc::Process(
		const PTR(AccMED)& searchMethod,
		const PTR(PcMED)& searchSurface,
		const PTR(PcMED)& inV) const
	{
		PcIndex upIdx;
		this->Process(searchMethod, searchSurface, inV, upIdx);

		for (std::size_t px = 0; px < upIdx.size(); ++px)
		{
			if (upIdx[px] >= 0)
			{
				PointMED& tarP = (*inV)[px];
				PointMED& srcP = (*searchSurface)[upIdx[px]];

#ifdef POINT_MED_WITH_NORMAL
				tarP.normal_x = srcP.normal_x;
				tarP.normal_y = srcP.normal_y;
				tarP.normal_z = srcP.normal_z;
				tarP.curvature = srcP.curvature;
#endif

#ifdef POINT_MED_WITH_SEGLABEL
				tarP.segLabel = srcP.segLabel;
#endif		
			}
		}
	}
}