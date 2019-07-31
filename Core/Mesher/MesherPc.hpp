#pragma once

#include "MesherPc.h"

namespace RecRoom
{
	template<class PointType>
	void MesherPc<PointType>::ImplementProcess(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Mesh& output) const
	{
		//
		PTR(Pc<PointType>) pcVertex(new Pc<PointType>);
		pcVertex->reserve(filter->size());
		for (PcIndex::const_iterator it = filter->begin(); it != filter->end(); ++it)
			pcVertex->push_back((*input)[*it]);
		PTR(Acc<PointType>) accVertex(new KDTree<PointType>);
		accVertex->setInputCloud(pcVertex);
		ToMesh(accVertex, pcVertex, output);

		//
		{
			PTR(Pc<PointType>) pcVertex2(new Pc<PointType>);
			pcl::fromPCLPointCloud2(output.cloud, *pcVertex2);
			fieldInterpolator->ProcessInOut(accVertex, pcVertex2, nullptr);
			pcl::toPCLPointCloud2(*pcVertex2, output.cloud);
		}
	}
}