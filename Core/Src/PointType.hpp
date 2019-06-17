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

template <> inline bool pcl::isFinite<RecRoom::PointExchange>(const RecRoom::PointExchange &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		EXCHANGE_ISFINITE_NORMAL \
		EXCHANGE_ISFINITE_RGB \
		EXCHANGE_ISFINITE_INTENSITY \
		EXCHANGE_ISFINITE_LABEL \
		EXCHANGE_ISFINITE_SEGLABEL;
}

