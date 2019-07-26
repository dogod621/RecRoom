#pragma once

#include "Point.h"

namespace RecRoom
{
	inline PointRAW::PointRAW(const PointREC& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef OUTPUT_PERPOINT_RGB
		RAW_COPY_RGB;
#else
		RAW_INIT_RGB;
#endif

#ifdef OUTPUT_PERPOINT_INTENSITY
		RAW_COPY_INTENSITY;
#else
		RAW_INIT_INTENSITY;
#endif

#ifdef OUTPUT_PERPOINT_NORMAL
		RAW_COPY_NORMAL;
#else
		RAW_INIT_NORMAL;
#endif

		RAW_INIT_SERIAL_NUMBER;
	}

	inline PointRAW::PointRAW(const PointMED& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		RAW_COPY_RGB;
		RAW_COPY_INTENSITY;
		RAW_COPY_NORMAL;
		RAW_COPY_SERIAL_NUMBER;
	}

	inline PointRAW& PointRAW::operator = (const PointREC &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef OUTPUT_PERPOINT_RGB
		RAW_COPY_RGB;
#endif

#ifdef OUTPUT_PERPOINT_INTENSITY
		RAW_COPY_INTENSITY;
#endif

#ifdef OUTPUT_PERPOINT_NORMAL
		RAW_COPY_NORMAL;
#endif

		return *this;
	}

	inline PointRAW& PointRAW::operator = (const PointMED &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		RAW_COPY_RGB;
		RAW_COPY_INTENSITY;
		RAW_COPY_NORMAL;
		RAW_COPY_SERIAL_NUMBER;

		return *this;
	}

	inline PointREC::PointREC(const PointRAW& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef INPUT_PERPOINT_RGB
		REC_COPY_RGB;
#else
		REC_INIT_RGB;
#endif

#ifdef INPUT_PERPOINT_INTENSITY
		REC_COPY_INTENSITY;
#else
		REC_INIT_INTENSITY;
#endif

#ifdef INPUT_PERPOINT_NORMAL
		REC_COPY_NORMAL;
#else
		REC_INIT_NORMAL;
#endif

		REC_INIT_SHARPNESS;
		REC_INIT_LABEL;
	}

	inline PointREC::PointREC(const PointMED& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		REC_COPY_RGB;
		REC_COPY_INTENSITY;
		REC_COPY_NORMAL;
		REC_COPY_SHARPNESS;
		REC_COPY_LABEL;
	}

	inline PointREC& PointREC::operator = (const PointRAW &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef INPUT_PERPOINT_RGB
		REC_COPY_RGB;
#endif

#ifdef INPUT_PERPOINT_INTENSITY
		REC_COPY_INTENSITY;
#endif

#ifdef INPUT_PERPOINT_NORMAL
		REC_COPY_NORMAL;
#endif

		return *this;
	}

	inline PointREC& PointREC::operator = (const PointMED &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		REC_COPY_RGB;
		REC_COPY_INTENSITY;
		REC_COPY_NORMAL;
		REC_COPY_SHARPNESS;
		REC_COPY_LABEL;

		return *this;
	}

	inline PointMED::PointMED(const PointRAW& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef INPUT_PERPOINT_RGB
		MED_COPY_RGB;
#else
		MED_INIT_RGB;
#endif

#ifdef INPUT_PERPOINT_INTENSITY
		MED_COPY_INTENSITY;
#else
		MED_INIT_INTENSITY;
#endif

#ifdef INPUT_PERPOINT_NORMAL
		MED_COPY_NORMAL;
#else
		MED_INIT_NORMAL;
#endif

		MED_INIT_SHARPNESS;

#ifdef INPUT_PERPOINT_SERIAL_NUMBER
		MED_COPY_SERIAL_NUMBER;
#else
		MED_INIT_SERIAL_NUMBER;
#endif

		MED_INIT_LABEL;
	}

	inline PointMED::PointMED(const PointREC& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef OUTPUT_PERPOINT_RGB
		MED_COPY_RGB;
#else
		MED_INIT_RGB;
#endif

#ifdef OUTPUT_PERPOINT_INTENSITY
		MED_COPY_INTENSITY;
#else
		MED_INIT_INTENSITY;
#endif

#ifdef OUTPUT_PERPOINT_NORMAL
		MED_COPY_NORMAL;
#else
		MED_INIT_NORMAL;
#endif

#ifdef OUTPUT_PERPOINT_SHARPNESS
		MED_COPY_SHARPNESS;
#else
		MED_INIT_SHARPNESS;
#endif

		MED_INIT_SERIAL_NUMBER;

#ifdef OUTPUT_PERPOINT_LABEL
		MED_COPY_LABEL;
#else
		MED_INIT_LABEL;
#endif
	}

	inline PointMED& PointMED::operator = (const PointRAW &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef INPUT_PERPOINT_RGB
		MED_COPY_RGB;
#endif

#ifdef INPUT_PERPOINT_INTENSITY
		MED_COPY_INTENSITY;
#endif

#ifdef INPUT_PERPOINT_NORMAL
		MED_COPY_NORMAL;
#endif

#ifdef INPUT_PERPOINT_SERIAL_NUMBER
		MED_COPY_SERIAL_NUMBER;
#endif

		return *this;
	}

	inline PointMED& PointMED::operator = (const PointREC &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

#ifdef OUTPUT_PERPOINT_RGB
		MED_COPY_RGB;
#endif

#ifdef OUTPUT_PERPOINT_INTENSITY
		MED_COPY_INTENSITY;
#endif

#ifdef OUTPUT_PERPOINT_NORMAL
		MED_COPY_NORMAL;
#endif

#ifdef OUTPUT_PERPOINT_SHARPNESS
		MED_COPY_SHARPNESS;
#endif

#ifdef OUTPUT_PERPOINT_LABEL
		MED_COPY_LABEL;
#endif

		return *this;
	}
}

template <> inline bool pcl::isFinite<RecRoom::PointRAW>(const RecRoom::PointRAW &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		RAW_ISFINITE_RGB \
		RAW_ISFINITE_INTENSITY \
		RAW_ISFINITE_NORMAL \
		RAW_ISFINITE_SERIAL_NUMBER;
}

template <> inline bool pcl::isFinite<RecRoom::PointREC>(const RecRoom::PointREC &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		REC_ISFINITE_RGB \
		REC_ISFINITE_INTENSITY \
		REC_ISFINITE_NORMAL \
		REC_ISFINITE_SHARPNESS \
		REC_ISFINITE_LABEL;
}

template <> inline bool pcl::isFinite<RecRoom::PointMED>(const RecRoom::PointMED &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		MED_ISFINITE_RGB \
		MED_ISFINITE_INTENSITY \
		MED_ISFINITE_NORMAL \
		MED_ISFINITE_SHARPNESS \
		MED_ISFINITE_SERIAL_NUMBER \
		MED_ISFINITE_LABEL;
}

template <> inline bool pcl::isFinite<RecRoom::PointVNN>(const RecRoom::PointVNN& p)
{
	if (p.k > 0)
	{
		if (p.indices)
			return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z);
		else
			return false;
	}
	else
	{
		return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z);
	}
}

template <> inline bool pcl::isFinite<RecRoom::PointNDF>(const RecRoom::PointNDF& p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) &&
		pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z) &&
		pcl_isfinite(p.intensity);
}

template <> inline bool pcl::isFinite<RecRoom::PointLF>(const RecRoom::PointLF& p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) &&
		pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z) &&
		pcl_isfinite(p.intensity);
}



