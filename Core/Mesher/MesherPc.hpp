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

			fieldInterpolator->ProcessInOut(searchSurface, pcVertex2, nullptr);
			
			// Interpolate missing field
			Pc<PointType> temp;
			fieldInterpolator->Process(searchSurface, pcVertex2, nullptr, temp);

			for (std::size_t px = 0; px < temp.size(); ++px)
			{
				PointType& tarP = (*pcVertex2)[px];
				PointType& srcP = temp[px];

				float tempX = tarP.x;
				float tempY = tarP.y;
				float tempZ = tarP.z;
				float tempNX = tarP.normal_x;
				float tempNY = tarP.normal_y;
				float tempNZ = tarP.normal_z;
				float tempNC = tarP.curvature;

				tarP = srcP;

				tarP.x = tempX;
				tarP.y = tempY;
				tarP.z = tempZ;

				if (pcl_isfinite(tempNX) && pcl_isfinite(tempNY) && pcl_isfinite(tempNZ))
				{
					if (pcl_isfinite(srcP.normal_x) && pcl_isfinite(srcP.normal_y) && pcl_isfinite(srcP.normal_z))
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
					else
					{
						tarP.normal_x = tempNX;
						tarP.normal_y = tempNY;
						tarP.normal_z = tempNZ;
					}
				}

				if (pcl_isfinite(tempNC))
					tarP.curvature = tempNC;
			}

			pcl::toPCLPointCloud2(*pcVertex2, output.cloud);
		}
	}
}