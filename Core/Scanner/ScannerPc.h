#pragma once

#include "Common/Common.h"
#include "Common/Coordinate.h"
#include "Container/ContainerPcRAW.h"
#include "Preprocessor/PreprocessorPc.h"

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
			rawDataCoordSys(CoordSys::CoordSys_UNKNOWN){}
	};

	class ScannerPc 
	{
	public:
		ScannerPc(
			const PTR(ContainerPcRAW)& containerPcRAW)
			: scanMeta(), containerPcRAW(containerPcRAW), preprocessor(nullptr)
		{
			if (!containerPcRAW)
				THROW_EXCEPTION("containerPcRAW is not set");
		}

	public:
		virtual void ShipData() const = 0;

	public:
		std::vector<ScanMeta> getScanMeta() const {return scanMeta;}
		ScanMeta getScanMeta(std::size_t i) const { return scanMeta[i]; }
		PTR(ContainerPcRAW) getContainerPcRAW () const { return containerPcRAW; }
		CONST_PTR(PreprocessorPc) getPreprocessor () const { return preprocessor; }

		void setPreprocessor(CONST_PTR(PreprocessorPc) preprocessor_) { preprocessor = preprocessor_; }

	protected:
		std::vector<ScanMeta> scanMeta;
		PTR(ContainerPcRAW) containerPcRAW;
		CONST_PTR(PreprocessorPc) preprocessor;
	};
}

#include "ScannerPc.hpp"