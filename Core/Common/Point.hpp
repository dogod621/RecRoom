#pragma once

#include "Point.h"

namespace RecRoom
{
	inline PointRAW::PointRAW(const PointREC& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef POINT_REC_WITH_NORMAL
		RAW_COPY_NORMAL;
#else
		RAW_INIT_NORMAL;
#endif

#ifdef POINT_REC_WITH_RGB
		RAW_COPY_RGB;
#else
		RAW_INIT_RGB;
#endif

#ifdef POINT_REC_WITH_INTENSITY
		RAW_COPY_INTENSITY;
#else
		RAW_INIT_INTENSITY;
#endif

		RAW_INIT_LABEL;
	}

	inline PointRAW::PointRAW(const PointMED& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		RAW_COPY_NORMAL;
		RAW_COPY_RGB;
		RAW_COPY_INTENSITY;
		RAW_COPY_LABEL;
	}

	inline PointREC::PointREC(const PointRAW& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef POINT_RAW_WITH_NORMAL
		REC_COPY_NORMAL;
#else
		REC_INIT_NORMAL;
#endif

#ifdef POINT_RAW_WITH_RGB
		REC_COPY_RGB;
#else
		REC_INIT_RGB;
#endif

#ifdef POINT_RAW_WITH_INTENSITY
		REC_COPY_INTENSITY;
#else
		REC_INIT_INTENSITY;
#endif

		REC_INIT_LABEL;
	}

	inline PointREC::PointREC(const PointMED& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		REC_COPY_NORMAL;
		REC_COPY_RGB;
		REC_COPY_INTENSITY;

#ifdef POINT_REC_WITH_LABEL
		label = p.segLabel;
#endif
	}

	inline PointMED::PointMED(const PointRAW& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef POINT_RAW_WITH_NORMAL
		MED_COPY_NORMAL;
#else
		MED_INIT_NORMAL;
#endif

#ifdef POINT_RAW_WITH_RGB
		MED_COPY_RGB;
#else
		MED_INIT_RGB;
#endif

#ifdef POINT_RAW_WITH_INTENSITY
		MED_COPY_INTENSITY;
#else
		MED_INIT_INTENSITY;
#endif

#ifdef POINT_RAW_WITH_LABEL
		MED_COPY_LABEL;
#else
		MED_INIT_LABEL;
#endif
		MED_INIT_SEGLABEL;
	}

	inline PointMED::PointMED(const PointREC& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef POINT_REC_WITH_NORMAL
		MED_COPY_NORMAL;
#else
		MED_INIT_NORMAL;
#endif

#ifdef POINT_REC_WITH_RGB
		MED_COPY_RGB;
#else
		MED_INIT_RGB;
#endif

#ifdef POINT_REC_WITH_INTENSITY
		MED_COPY_INTENSITY;
#else
		MED_INIT_INTENSITY;
#endif

		MED_INIT_LABEL;
#ifdef POINT_REC_WITH_LABEL
		segLabel = p.label;
#else
		MED_INIT_SEGLABEL;
#endif
	}
}

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

