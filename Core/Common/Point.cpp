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
		REC_COPY_SEGLABEL;
	}

	inline PointMED::PointMED(const PointRAW& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		MED_COPY_NORMAL;
		MED_COPY_RGB;
		MED_COPY_INTENSITY;
		MED_COPY_LABEL;
		MED_INIT_SEGLABEL;
	}

	inline PointMED::PointMED(const PointREC& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		MED_COPY_NORMAL;
		MED_COPY_RGB;
		MED_COPY_INTENSITY;
		MED_INIT_LABEL;
#ifdef POINT_REC_WITH_INTENSITY
		segLabel = p.label;
#else
		MED_INIT_SEGLABEL;
#endif
	}
}