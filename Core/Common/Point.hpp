#pragma once

#include "Point.h"

namespace RecRoom
{
	inline PointRAW::PointRAW(const PointREC& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		intensity = 0.0f;
		hasSerialNumber = -1;
	}

	inline PointRAW::PointRAW(const PointMED& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		intensity = p.intensity;
		serialNumber = p.serialNumber;
	}

	inline PointRAW& PointRAW::operator = (const PointREC &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		return *this;
	}

	inline PointRAW& PointRAW::operator = (const PointMED &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		intensity = p.intensity;
		serialNumber = p.serialNumber;
		return *this;
	}

	inline PointREC::PointREC(const PointRAW& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
		diffuseAlbedo = 0.0f;
		specularAlbedo = 0.f;
		specularSharpness = 0.0f;
	}

	inline PointREC::PointREC(const PointMED& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
		diffuseAlbedo = p.diffuseAlbedo;
		specularAlbedo = p.specularAlbedo;
		specularSharpness = p.specularSharpness;
	}

	inline PointREC& PointREC::operator = (const PointRAW &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		return *this;
	}

	inline PointREC& PointREC::operator = (const PointMED &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
		diffuseAlbedo = p.diffuseAlbedo;
		specularAlbedo = p.specularAlbedo;
		specularSharpness = p.specularSharpness;
		return *this;
	}

	inline PointMED::PointMED(const PointRAW& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		intensity = p.intensity;
		normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
		diffuseAlbedo = 0.0f;
		specularAlbedo = 0.f;
		specularSharpness = 0.0f;
		serialNumber = p.serialNumber;
		softLabelStart = 0;
		softLabelEnd = 0;
	}

	inline PointMED::PointMED(const PointREC& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		intensity = 0.0f;
		normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
		diffuseAlbedo = p.diffuseAlbedo;
		specularAlbedo = p.specularAlbedo;
		specularSharpness = p.specularSharpness;
		hasSerialNumber = -1;
		softLabelStart = 0;
		softLabelEnd = 0;
	}

	inline PointMED& PointMED::operator = (const PointRAW &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		intensity = p.intensity;
		serialNumber = p.serialNumber;
		return *this;
	}

	inline PointMED& PointMED::operator = (const PointREC &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rgba = p.rgba;
		normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
		diffuseAlbedo = p.diffuseAlbedo;
		specularAlbedo = p.specularAlbedo;
		specularSharpness = p.specularSharpness;
		return *this;
	}
}

template <> inline bool pcl::isFinite<RecRoom::PointRAW>(const RecRoom::PointRAW &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		&& pcl_isfinite(p.intensity);
}

template <> inline bool pcl::isFinite<RecRoom::PointREC>(const RecRoom::PointREC &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z) && pcl_isfinite(p.curvature) \
		&& pcl_isfinite(p.diffuseAlbedo) && pcl_isfinite(p.specularAlbedo) && pcl_isfinite(p.specularSharpness);
}
template <> inline bool pcl::isFinite<RecRoom::PointMED>(const RecRoom::PointMED &p)
{
	return pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z) \
		&& pcl_isfinite(p.intensity) \
		&& pcl_isfinite(p.normal_x) && pcl_isfinite(p.normal_y) && pcl_isfinite(p.normal_z) && pcl_isfinite(p.curvature) \
		&& pcl_isfinite(p.diffuseAlbedo) && pcl_isfinite(p.specularAlbedo) && pcl_isfinite(p.specularSharpness);
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



