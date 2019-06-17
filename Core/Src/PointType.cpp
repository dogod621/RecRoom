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

	inline PointE57::PointE57(const PointExchange& p)
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
	
	inline PointPCD::PointPCD(const PointExchange& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		PCD_COPY_NORMAL;
		PCD_COPY_RGB;
		PCD_COPY_INTENSITY;
		PCD_COPY_SEGLABEL;
	}
	
	inline PointExchange::PointExchange(const PointE57& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		EXCHANGE_COPY_NORMAL;
		EXCHANGE_COPY_RGB;
		EXCHANGE_COPY_INTENSITY;
		EXCHANGE_COPY_LABEL;
		EXCHANGE_INIT_SEGLABEL;
	}
	
	inline PointExchange::PointExchange(const PointPCD& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;

		EXCHANGE_COPY_NORMAL;
		EXCHANGE_COPY_RGB;
		EXCHANGE_COPY_INTENSITY;
		EXCHANGE_INIT_LABEL;
#ifdef POINT_PCD_WITH_INTENSITY
		segLabel = p.label;
#else
		EXCHANGE_INIT_SEGLABEL;
#endif
	}
}