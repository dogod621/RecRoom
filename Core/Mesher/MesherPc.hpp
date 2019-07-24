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
		PTR(Pc<PointType>) pcVertex (new Pc<PointType>);
		{
			PTR(Acc<PointType>) searchSurface2; 
			PTR(Pc<PointType>) input2;
			PTR(PcIndex) filter2;
			if (preprocessSampler)
			{
				PRINT_INFO("Sampling - Start");

				searchSurface2 = PTR(Acc<PointType>)(new KDTree<PointType>);
				input2 = PTR(Pc<PointType>)(new Pc<PointType>);
				filter2 = nullptr;
				
				preprocessSampler->Process(searchSurface, input, filter, *input2);

				searchSurface2->setInputCloud(input2);

				std::stringstream ss;
				ss << "Sampling - End - orgPcSize: " << input->size() << ", pcSize: " << input2->size();
				PRINT_INFO(ss.str());
			}
			else
			{
				searchSurface2 = boost::const_pointer_cast<Acc<PointType>>(searchSurface);
				input2 = boost::const_pointer_cast<Pc<PointType>>(input);
				filter2 = boost::const_pointer_cast<PcIndex>(filter);
			}

			//
			PTR(Acc<PointType>) searchSurface3;
			PTR(PcIndex) filter3;
			if (preprocessFilter)
			{
				PRINT_INFO("Filtering - Start");

				searchSurface3 = PTR(Acc<PointType>)(new KDTree<PointType>);
				filter3 = PTR(PcIndex)(new PcIndex);

				preprocessFilter->Process(searchSurface2, input2, filter2, *filter3);

				searchSurface3->setInputCloud(input2, filter3);

				std::stringstream ss;
				ss << "Filtering - End - orgPcSize: " << input2->size() << ", pcSize: " << filter3->size();
				PRINT_INFO(ss.str());
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
					if ((*it) >= 0)
					{
						PointType srcP = (*input2)[*it];

						if (pcl::isFinite(srcP))
							pcVertex->push_back(srcP);
					}
				}
			}
			else
			{
				pcVertex->reserve(input2->size());

				for (std::size_t px = 0; px < input2->size() ; ++px)
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
		{
			PRINT_INFO("Reconstruct - Start");

			ToMesh(treeVertex, pcVertex, output);

			PRINT_INFO("Reconstruct - End");
		}

		//
		{
			PTR(Pc<PointType>) pcVertex2(new Pc<PointType>);
			pcl::fromPCLPointCloud2(output.cloud, *pcVertex2);

			PRINT_INFO("Interpolation Field - Start");

			fieldInterpolator->ProcessInOut(treeVertex, pcVertex2, nullptr);
			
			PRINT_INFO("Interpolation Field - End");

			pcl::toPCLPointCloud2(*pcVertex2, output.cloud);
		}
	}
}