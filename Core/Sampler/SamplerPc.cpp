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
			if(upIdx[px] >= 0)
				(*inV)[px] = (*searchSurface)[upIdx[px]];
		}
	}
}