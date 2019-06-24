#include <pcl/filters/statistical_outlier_removal.h>

#include "CropperPcOutlier.h"

namespace RecRoom
{
	void CropperPcOutlier::Process(const PTR(PcMED)& inV, PcIndex& outV) const
	{
		pcl::StatisticalOutlierRemoval<PointMED> olr;
		olr.setMeanK(meanK);
		olr.setStddevMulThresh(stdMul);
		olr.setInputCloud(inV);
		olr.filter(outV);
	}
}
