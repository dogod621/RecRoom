#pragma once

#include "Common.hpp"
#include "Coordinate.hpp"

namespace RecRoom
{
	struct ScanMeta
	{
		Eigen::Matrix4d transform;
		Eigen::Quaterniond orientation; // cache from transform
		Eigen::Vector3d position; // cache from transform
		std::size_t numPoints;
		Scanner scanner;
		int serialNumber;
		bool hasPointXYZ;
		bool hasPointNormal;
		bool hasPointRGB;
		bool hasPointI;
		CoordSys rawDataCoordSys;

		ScanMeta()
			: transform(Eigen::Matrix4d::Identity()), orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)), position(Eigen::Vector3d(0.0, 0.0, 0.0)),
			numPoints(0), scanner(Scanner::Scaner_UNKNOWN), serialNumber(-1),
			hasPointXYZ(false), hasPointNormal(false), hasPointRGB(false), hasPointI(false),
			rawDataCoordSys(CoordSys::CoordSys_UNKNOWN) {}
	};

	struct ScannLaser
	{
		Eigen::Vector3d incidentDirection;
		Eigen::Vector3d reflectedDirection;
		double intensity;
		Eigen::Vector3d hitPosition;
		Eigen::Vector3d hitNormal;
		Eigen::Vector3d hitTangent;
		Eigen::Vector3d hitBitangent;
		double hitDistance;
		double weight;
		double beamFalloff;
	};
}