#pragma once

#include "SamplerPcRemoveDuplicate.h"

namespace RecRoom
{
	struct PointStatus
	{
		bool valid;
		bool stamp;

		PointStatus() : valid(true), stamp(false) {}
	};

	template<class PointType>
	void SamplerPcRemoveDuplicate<PointType>::Process(
		const PTR(Pc<PointType>) & inV,
		Pc<PointType> & outV) const
	{
		std::vector<PointStatus> pcStatus;
		pcStatus.resize(inV->size());

		PTR(KDTree<PointType>) tree(new KDTree<PointType>);
		tree->setInputCloud(inV);

		std::vector<int> dfsStack;
		dfsStack.reserve(inV->size());
		outV.reserve(inV->size());
		
		for (int px1 = 0; px1 < inV->size(); ++px1)
		{
			PointStatus& ps1 = pcStatus[px1];
			if (ps1.valid && (!ps1.stamp))
			{
				dfsStack.push_back(px1);
				ps1.stamp = true;
			}

			while (!dfsStack.empty())
			{
				int px2 = dfsStack.back();
				dfsStack.pop_back();
				PointStatus& ps2 = pcStatus[px2];
				outV.push_back((*inV)[px2]);
				
				std::vector<int> ki;
				std::vector<float> kd;
				if (tree->radiusSearch(*inV, px2, searchRadius, ki, kd) > 1)
				{
					for (std::size_t i = 0; i < ki.size(); ++i)
					{
						int px3 = ki[i];
						PointStatus& ps3 = pcStatus[px3];
						if (kd[i] > sqrMinDistance)
						{
							if (ps3.valid && (!ps3.stamp))
							{
								dfsStack.push_back(px3);
								ps3.stamp = true;
							}
						}
						else
							ps3.valid = false;
					}
				}
			}
		}
	}
}