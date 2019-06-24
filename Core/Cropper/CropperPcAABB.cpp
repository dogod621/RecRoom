#include <pcl/filters/crop_box.h>

#include "CropperPcAABB.h"

namespace RecRoom
{
	void CropperPcAABB::Process(const PTR(PcMED)& inV, PcIndex& outV) const
	{
		pcl::CropBox<PointMED> cb;
		cb.setMin(Eigen::Vector4f(minAABB.x(), minAABB.y(), minAABB.z(), 1.0));
		cb.setMax(Eigen::Vector4f(maxAABB.x(), maxAABB.y(), maxAABB.z(), 1.0));
		cb.setInputCloud(inV);
		cb.filter(outV);
	}
}