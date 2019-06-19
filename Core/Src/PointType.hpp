#pragma once

#include "PointType.h"

namespace RecRoom
{
	inline PointE57::PointE57(const PointPCD& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef POINT_PCD_WITH_NORMAL
		E57_COPY_NORMAL;
#else
		E57_INIT_NORMAL;
#endif

#ifdef POINT_PCD_WITH_RGB
		E57_COPY_RGB;
#else
		E57_INIT_RGB;
#endif

#ifdef POINT_PCD_WITH_INTENSITY
		E57_COPY_INTENSITY;
#else
		E57_INIT_INTENSITY;
#endif

		E57_INIT_LABEL;
	}

	inline PointE57::PointE57(const PointE57xPCD& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		E57_COPY_NORMAL;
		E57_COPY_RGB;
		E57_COPY_INTENSITY;
		E57_COPY_LABEL;
	}

	inline PointPCD::PointPCD(const PointE57& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef POINT_E57_WITH_NORMAL
		PCD_COPY_NORMAL;
#else
		PCD_INIT_NORMAL;
#endif

#ifdef POINT_E57_WITH_RGB
		PCD_COPY_RGB;
#else
		PCD_INIT_RGB;
#endif

#ifdef POINT_E57_WITH_INTENSITY
		PCD_COPY_INTENSITY;
#else
		PCD_INIT_INTENSITY;
#endif

		PCD_INIT_LABEL;
	}

	inline PointPCD::PointPCD(const PointE57xPCD& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		PCD_COPY_NORMAL;
		PCD_COPY_RGB;
		PCD_COPY_INTENSITY;
		PCD_COPY_SEGLABEL;
	}

	inline PointE57xPCD::PointE57xPCD(const PointE57& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		E57xPCD_COPY_NORMAL;
		E57xPCD_COPY_RGB;
		E57xPCD_COPY_INTENSITY;
		E57xPCD_COPY_LABEL;
		E57xPCD_INIT_SEGLABEL;
	}

	inline PointE57xPCD::PointE57xPCD(const PointPCD& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		E57xPCD_COPY_NORMAL;
		E57xPCD_COPY_RGB;
		E57xPCD_COPY_INTENSITY;
		E57xPCD_INIT_LABEL;
#ifdef POINT_PCD_WITH_INTENSITY
		segLabel = p.label;
#else
		E57xPCD_INIT_SEGLABEL;
#endif
	}
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

