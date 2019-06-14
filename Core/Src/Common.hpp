#pragma once

#include "Common.h"

namespace RecRoom
{
	template<> Eigen::Vector3d FlagConvert<Axis, Eigen::Vector3d>(Axis v)
	{
		switch (v)
		{
		case Axis::X: return Eigen::Vector3d(1.0, 0.0, 0.0); break;
		case Axis::Y: return Eigen::Vector3d(0.0, 1.0, 0.0); break;
		case Axis::Z: return Eigen::Vector3d(0.0, 0.0, 1.0); break;
		case Axis::NX: return Eigen::Vector3d(-1.0, 0.0, 0.0); break;
		case Axis::NY: return Eigen::Vector3d(0.0, -1.0, 0.0); break;
		case Axis::NZ: return Eigen::Vector3d(0.0, 0.0, -1.0); break;
		default: THROW_EXCEPTION("Axis is not support"); break;
		}
	}

	template<> _CoodMode FlagConvert<CoodSys, _CoodMode>(CoodSys v)
	{
		if (v & 0b00000000000000000000000000000001) return _CoodMode::_XYZ;
		else if (v & 0b00000000000000000000000111111110) return _CoodMode::_RAE;
		else return _CoodMode::_CoodMode_UNKNOWN;
	}

	template<> _RAEElevationMode FlagConvert<CoodSys, _RAEElevationMode>(CoodSys v)
	{
		return (_RAEElevationMode)(v & 0b00000000000000000000000110000000);
	}

	template<> _RAEAzimuthMode FlagConvert<CoodSys, _RAEAzimuthMode>(CoodSys v)
	{
		return (_RAEAzimuthMode)(v & 0b00000000000000000000000001111110);
	}

	template<> _CoodMode FlagConvert<Projection, _CoodMode>(Projection v)
	{
		return FlagConvert<CoodSys, _CoodMode>(FlagConvert<Projection, CoodSys>(v));
	}

	template<> CoodSys FlagConvert<Projection, CoodSys>(Projection v)
	{
		return (CoodSys)(v & 0b00000000000000001111111111111111);
	}

	template<> _UVMode FlagConvert<Projection, _UVMode>(Projection v)
	{
		return (_UVMode)(v & 0b11111111111111110000000000000000);
	}

	Eigen::Vector3d GetAzimuth0DegreeVector(_RAEAzimuthMode type)
	{
		return FlagConvert<Axis, Eigen::Vector3d>((Axis)(type & 0b00000000000000000000000001110000));
	}

	Eigen::Vector3d GetAzimuth90DegreeVector(_RAEAzimuthMode type)
	{
		return FlagConvert<Axis, Eigen::Vector3d>((Axis)(type & 0b00000000000000000000000000001110));
	}

	template<CoodSys inType, CoodSys outType >
	Eigen::Vector3d CoodConvert(const Eigen::Vector3d& coord)
	{
		switch (FlagConvert<CoodSys, _CoodMode>(inType))
		{
		case _CoodMode::_XYZ:
			switch (FlagConvert<CoodSys, _CoodMode>(outType))
			{
			case _CoodMode::_XYZ: return coord; break;

			case _CoodMode::_RAE:
			{
				Eigen::Vector3d rae;
				rae.x() = coord.norm();
				Eigen::Vector3d uv_xyz = coord / rae.x();
				Eigen::Vector3d uv_a0 = GetAzimuth0DegreeVector(type);
				Eigen::Vector3d uv_a90 = GetAzimuth90DegreeVector(type);
				Eigen::Vector3d uv_n = uv_a0.cross(uv_a90);

				double len_n = uv_xyz.dot(uv_n);
				switch (FlagConvert<_RAEElevationMode, CoodSys>(outType))
				{
				case _RAEElevationMode::_N:
					rae.z() = std::acos(len_n);
					break;
				case _RAEElevationMode::_S:
					rae.z() = M_PI - std::acos(len_n);
					break;
				case _RAEElevationMode::_E:
					rae.z() = std::asin(len_n);
					break;
				default:
					THROW_EXCEPTION("outType _RAEElevationMode is not support");
					break;
				}

				Eigen::Vector3d v_e = uv_xyz - len_n * uv_n;
				rae.y() = std::atan2(v_e.dot(uv_a90), v_e.dot(uv_a0));
				return rae;
			}
			break;

			default:
				THROW_EXCEPTION("outType is not support");
				break;
			}
			break;

		case _CoodMode::_RAE:
			switch (FlagConvert<CoodSys, _CoodMode>(outType))
			{
			case _CoodMode::_XYZ:
			{
				double r = coord.x();
				double s_e = std::sin(coord.z());
				double s_a = std::sin(coord.y());
				double c_e = std::cos(coord.z());
				double c_a = std::cos(coord.y());
				Eigen::Vector3d uv_a0 = GetAzimuth0DegreeVector(type);
				Eigen::Vector3d uv_a90 = GetAzimuth90DegreeVector(type);
				Eigen::Vector3d uv_n = uv_a0.cross(uv_a90);

				//
				double len_n;
				double len_e;
				switch (FlagConvert<_RAEElevationMode, CoodSys>(inType))
				{
				case ElevationMode::N:
					len_n = r * c_e;
					len_e = r * s_e;
					break;
				case ElevationMode::S:
					len_n = r * (-c_e);
					len_e = r * s_e;
					break;
				case ElevationMode::E:
					len_n = r * s_e;
					len_e = r * c_e;
					break;
				default:
					THROW_EXCEPTION("inType _RAEElevationMode is not support");
					break;
				}
				return uv_a0 * len_e * c_a + uv_a90 * len_e * s_a + uv_n * len_n;
			}
			break;

			case _CoodMode::_RAE: return coord; break;

			default:
				THROW_EXCEPTION("outType is not support");
				break;
			}
			break;

		default:
			THROW_EXCEPTION("inType is not support");
			break;
		}
	}

	template<Projection projection>
	Eigen::Vector2d ToUV(const Eigen::Vector3d& coord)
	{
		CoodSys coodSys = FlagConvert<Projection, CoodSys>(projection);
		_UVMode uvMode = FlagConvert<Projection, _UVMode>(projection);
		_CoodMode coodMode = FlagConvert<CoodSys, _CoodMode>(coodSys);

		switch (coodMode)
		{
		case _CoodMode::RAE:
		{
			_RAEElevationMode elevationMode = FlagConvert<CoodSys, _RAEElevationMode>(coodSys);

			switch (uvMode)
			{
			case _UVMode::_PANORAMA:
			{
				Eigen::Vector2d p2;

				
				p2.x() = 0.5 * (1.0 - rae.y() / (M_PI));
				
				switch (elevationMode)
				{
				case _RAEElevationMode::_E:
					p2.y() = 0.5 * (1.0 + rae.z() / (M_PI*0.5));
					break;
				case _RAEElevationMode::_N:
					p2.y() = (1.0 - rae.z() / M_PI);
					break;
				case _RAEElevationMode::_S:
					p2.y() = rae.z() / M_PI;
					break;
				default:
					THROW_EXCEPTION("projection _RAEElevationMode is not support");
					break;
				}

				p2.x() = std::min(std::max(0.0, p2.x()), 1.0);
				p2.y() = std::min(std::max(0.0, p2.y()), 1.0);
				return p2;
			}
			break;

			case _UVMode::_PANORAMA_EQUIRECTANGULAR: 
			{
				Eigen::Vector2d p2;


				p2.x() = 0.5 * (1.0 - rae.y() / (M_PI));

				switch (elevationMode)
				{
				case _RAEElevationMode::_E:
					p2.y() = 0.5 * (1.0 + std::sin(rae.z()));
					break;
				case _RAEElevationMode::_N:
					p2.y() = std::cos(rae.z());
					break;
				case _RAEElevationMode::_S:
					p2.y() = 1.0 - std::cos(rae.z());
					break;
				default:
					THROW_EXCEPTION("projection _RAEElevationMode is not support");
					break;
				}

				p2.x() = std::min(std::max(0.0, p2.x()), 1.0);
				p2.y() = std::min(std::max(0.0, p2.y()), 1.0);
				return p2;
			}
			break;

			default:
				THROW_EXCEPTION("projection _UVMode is not support");
				break;
			}
			break;
		}
		break;

		default:
			THROW_EXCEPTION("projection CoodSys is not support");
			break;
		}
	}
}