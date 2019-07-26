#pragma once

#include "Filter/FilterPcRemoveNonFinite.h"

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
		PTR(PcIndex) filterRmNAN (new PcIndex);
		FilterPcRemoveNonFinite<PointType> fnan;
		fnan.Process(searchSurface, input, filter, *filterRmNAN);

		//
		PTR(Pc<PointType>) pcVertex(new Pc<PointType>);
		{
			PTR(Acc<PointType>) searchSurface2;
			PTR(Pc<PointType>) input2;
			PTR(PcIndex) filter2;
			if (preprocessSampler)
			{
				searchSurface2 = PTR(Acc<PointType>)(new KDTree<PointType>);
				input2 = PTR(Pc<PointType>)(new Pc<PointType>);
				filter2 = nullptr;

				preprocessSampler->Process(searchSurface, input, filterRmNAN, *input2);

				searchSurface2->setInputCloud(input2);
			}
			else
			{
				searchSurface2 = boost::const_pointer_cast<Acc<PointType>>(searchSurface);
				input2 = boost::const_pointer_cast<Pc<PointType>>(input);
				filter2 = boost::const_pointer_cast<PcIndex>(filterRmNAN);
			}

			//
			PTR(Acc<PointType>) searchSurface3;
			PTR(PcIndex) filter3;
			if (preprocessFilter)
			{
				searchSurface3 = PTR(Acc<PointType>)(new KDTree<PointType>);
				filter3 = PTR(PcIndex)(new PcIndex);

				preprocessFilter->Process(searchSurface2, input2, filter2, *filter3);

				searchSurface3->setInputCloud(input2, filter3);
			}
			else
			{
				searchSurface3 = searchSurface2;
				filter3 = filter2;
			}

			//
			if (filter3)
			{
				pcVertex->reserve(filter3->size());
				for (PcIndex::const_iterator it = filter3->begin(); it != filter3->end(); ++it)
				{
					PointType srcP = (*input2)[*it];

					if (pcl::isFinite(srcP))
						pcVertex->push_back(srcP);
				}
			}
			else
			{
				pcVertex->reserve(input2->size());

				for (std::size_t px = 0; px < input2->size(); ++px)
				{
					PointType srcP = (*input2)[px];

					if (pcl::isFinite(srcP))
						pcVertex->push_back(srcP);
				}
			}
		}

		//
		PTR(Acc<PointType>) treeVertex(new KDTree<PointType>);
		treeVertex->setInputCloud(pcVertex);

		//
		ToMesh(treeVertex, pcVertex, output);

		//
		{
			PTR(Pc<PointType>) pcVertex2(new Pc<PointType>);
			pcl::fromPCLPointCloud2(output.cloud, *pcVertex2);

			fieldInterpolator->ProcessInOut(treeVertex, pcVertex2, nullptr);

			pcl::toPCLPointCloud2(*pcVertex2, output.cloud);
		}
	}
}