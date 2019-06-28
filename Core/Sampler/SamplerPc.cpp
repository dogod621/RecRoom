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
				float tempX = (*inV)[px].x;
				float tempY = (*inV)[px].y;
				float tempZ = (*inV)[px].z;
				uint32_t tempLable = (*inV)[px].label;
				(*inV)[px] = (*searchSurface)[upIdx[px]];
				(*inV)[px].x = tempX;
				(*inV)[px].y = tempY;
				(*inV)[px].z = tempZ;
				(*inV)[px].label = tempLable;
			}
		}
	}
}