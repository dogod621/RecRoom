#include <pcl/surface/mls.h>

#include "Sampler/SamplerPcNearest.h"

#include "MesherPc.h"

namespace RecRoom
{
	void MesherPc::Process(PTR(PcMED)& inV, pcl::PolygonMesh& out) const
	{
		PTR(Pc<pcl::PointNormal>) pcNT(new Pc<pcl::PointNormal>);
		PTR(PcREC) pcREC(new PcREC);
		
		pcNT->reserve(inV->size());
		pcREC->reserve(inV->size());
		for (PcMED::const_iterator it = inV->begin(); it != inV->end(); ++it)
		{
			if (pcl::isFinite(*it))
			{
				pcl::PointNormal p;
				p.x = it->x;
				p.y = it->y;
				p.z = it->z;
				float norm = std::sqrt(it->normal_x*it->normal_x + it->normal_y*it->normal_y + it->normal_z*it->normal_z);
				p.normal_x = it->normal_x / norm;
				p.normal_y = it->normal_y / norm;
				p.normal_z = it->normal_z / norm;
				p.curvature = it->curvature;
				pcNT->push_back(p);
				pcREC->push_back(*it);
			}
		}

		{
			PTR(KDTree<pcl::PointNormal>) treeNT(new KDTree<pcl::PointNormal>);
			ToMesh(pcNT, treeNT, out);
		}

		{
			PTR(KDTreeREC) treeREC(new KDTreeREC);
			treeREC->setInputCloud(pcREC);

			PTR(Pc<pcl::PointNormal>) pcNT(new Pc<pcl::PointNormal>);
			pcl::fromPCLPointCloud2(out.cloud, *pcNT);

			PTR(PcREC) temp(new PcREC);
			temp->resize(pcNT->size());
			for (std::size_t px = 0; px < pcNT->size(); ++px)
			{
				PointREC& tarP = (*temp)[px];
				pcl::PointNormal& srcP = (*pcNT)[px];
				tarP.x = srcP.x;
				tarP.y = srcP.y;
				tarP.z = srcP.z;
				tarP.normal_x = srcP.normal_x;
				tarP.normal_y = srcP.normal_y;
				tarP.normal_z = srcP.normal_z;
				tarP.curvature = srcP.curvature;
			}
			
			SamplerPcNearestREC upSampler;
			PcIndex upIdx;
			upSampler.Process(treeREC, pcREC, temp, upIdx);

			for (std::size_t px = 0; px < upIdx.size(); ++px)
			{
				if (upIdx[px] >= 0)
				{
					PointREC& tarP = (*temp)[px];
					PointREC& srcP = (*pcREC)[upIdx[px]];

#ifdef POINT_REC_WITH_NORMAL
					tarP.normal_x = srcP.normal_x;
					tarP.normal_y = srcP.normal_y;
					tarP.normal_z = srcP.normal_z;
					tarP.curvature = srcP.curvature;
#endif

#ifdef POINT_REC_WITH_RGB
					tarP.r = srcP.r;
					tarP.g = srcP.g;
					tarP.b = srcP.b;
#endif

#ifdef POINT_REC_WITH_INTENSITY
					tarP.intensity = srcP.intensity;
					tarP.intensity = srcP.intensity;
					tarP.intensity = srcP.intensity;
#endif
#ifdef POINT_REC_WITH_LABEL
					tarP.label = srcP.label;
#endif		
				}
			}

			pcl::toPCLPointCloud2(*temp, out.cloud);
		}
	}
}