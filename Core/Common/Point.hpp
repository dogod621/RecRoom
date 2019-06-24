#pragma once

#include "Point.h"

template <> inline bool pcl::isFinite<RecRoom::PointRAW>(const RecRoom::PointRAW &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		RAW_ISFINITE_NORMAL \
		RAW_ISFINITE_RGB \
		RAW_ISFINITE_INTENSITY \
		RAW_ISFINITE_LABEL;
}

template <> inline bool pcl::isFinite<RecRoom::PointREC>(const RecRoom::PointREC &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		REC_ISFINITE_NORMAL \
		REC_ISFINITE_RGB \
		REC_ISFINITE_INTENSITY \
		REC_ISFINITE_LABEL;
}

template <> inline bool pcl::isFinite<RecRoom::PointMED>(const RecRoom::PointMED &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		MED_ISFINITE_NORMAL \
		MED_ISFINITE_RGB \
		MED_ISFINITE_INTENSITY \
		MED_ISFINITE_LABEL \
		MED_ISFINITE_SEGLABEL;
}

