#pragma once

#include "PointType.h"

namespace RecRoom
{
}

template <> inline bool pcl::isFinite<RecRoom::PointE57>(const RecRoom::PointE57 &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		E57_ISFINITE_NORMAL \
		E57_ISFINITE_RGB \
		E57_ISFINITE_INTENSITY \
		E57_ISFINITE_LABEL;
}

template <> inline bool pcl::isFinite<RecRoom::PointPCD>(const RecRoom::PointPCD &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		PCD_ISFINITE_NORMAL \
		PCD_ISFINITE_RGB \
		PCD_ISFINITE_INTENSITY \
		PCD_ISFINITE_LABEL;
}

template <> inline bool pcl::isFinite<RecRoom::PointE57xPCD>(const RecRoom::PointE57xPCD &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		E57xPCD_ISFINITE_NORMAL \
		E57xPCD_ISFINITE_RGB \
		E57xPCD_ISFINITE_INTENSITY \
		E57xPCD_ISFINITE_LABEL \
		E57xPCD_ISFINITE_SEGLABEL;
}

