#include "SegmenterPcSVC.h"
#include "Common/SupervoxelClustering.h"

namespace RecRoom
{
	void SegmenterPcSVC::Process(const PTR(PcMED)& pc) const
	{
#ifdef POINT_MED_WITH_SEGLABEL
		SupervoxelClustering super(
			voxelResolution, seedResolution,
			xyzImportance, normalImportance, rgbImportance, intensityImportance);
		super.setInputCloud(pc);
		super.Extract(*pc);
#endif
	}
}