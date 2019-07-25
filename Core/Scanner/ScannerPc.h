#pragma once

#include "Common/Common.h"
#include "Common/Coordinate.h"
#include "Container/ContainerPcRAW.h"
#include "Container/ContainerPcLF.h"
#include "Preprocessor/PreprocessorPc.h"

namespace RecRoom
{
	struct ScanMeta
	{
		Eigen::Matrix4d transform;
		Eigen::Quaterniond orientation; // cache from transform
		Eigen::Vector3d position; // cache from transform
		std::size_t numPoints;
		int serialNumber;
		bool hasPointXYZ;
		bool hasPointNormal;
		bool hasPointRGB;
		bool hasPointI;
		CoordSys rawDataCoordSys;

		ScanMeta()
			: transform(Eigen::Matrix4d::Identity()), orientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)), position(Eigen::Vector3d(0.0, 0.0, 0.0)),
			numPoints(0), serialNumber(-1),
			hasPointXYZ(false), hasPointNormal(false), hasPointRGB(false), hasPointI(false),
			rawDataCoordSys(CoordSys::CoordSys_UNKNOWN) {}
	};

	struct ScanLaser
	{
		Eigen::Vector3f incidentDirection;
		float intensity;

		Eigen::Vector3f reflectedDirection;
		float beamFalloff;

		ScanLaser(): intensity(0.0f), beamFalloff(0.0f) {}
	};

	std::ostream& operator << (std::ostream& os, const ScanMeta& v);

	class ScannerPc 
	{
	public:
		ScannerPc(const PTR(ContainerPcRAW)& containerPcRAW)
			: scanMetaSet(new std::vector<ScanMeta>), containerPcRAW(containerPcRAW), preprocessor(nullptr)
		{
			if (!containerPcRAW)
				THROW_EXCEPTION("containerPcRAW is not set");
		}

	public:
		void DoShipPcRAW() const;
		void DoShipPcLF() const;
		virtual void LoadPcRAW(int serialNumber, PcRAW& pc, bool local = false) const = 0;
		virtual void LoadPcLF(int serialNumber, PcLF& pc, bool local = false) const = 0;
		virtual std::size_t ScanImageWidth () const { THROW_EXCEPTION("Interface is not implemented") };
		virtual std::size_t ScanImageHeight () const { THROW_EXCEPTION("Interface is not implemented") };
		virtual Eigen::Vector3d ToScanImageUVDepth (const Eigen::Vector3d& worldXYZ) const { THROW_EXCEPTION("Interface is not implemented") };

		virtual bool ToScanLaser (const PointRAW& scanPoint, ScanLaser& scanLaser) const = 0;

	protected:
		virtual void ShipPcRAW() const = 0;
		virtual void ShipPcLF() const = 0;
		
		virtual bool Valid(const PointRAW& pointRAW) const
		{
			return pcl::isFinite(pointRAW);
		}

	public:
		PTR(std::vector<ScanMeta>) getScanMetaSet() const {return scanMetaSet;}

		const ScanMeta& getScanMeta(int serialNumber) const
		{ 
			if((serialNumber >= scanMetaSet->size()) || (serialNumber < 0))
				THROW_EXCEPTION("serialNumber is not valid");
			const ScanMeta& s = (*scanMetaSet)[serialNumber];
			if (s.serialNumber < 0)
				THROW_EXCEPTION("The scan is not valid");
			if (serialNumber != s.serialNumber)
				THROW_EXCEPTION("serialNumber is not match");
			return s; 
		}

		PTR(ContainerPcRAW) getContainerPcRAW () const { return containerPcRAW; }
		PTR(ContainerPcLF) getContainerPcLF() const { return containerPcLF; }
		CONST_PTR(PreprocessorPc) getPreprocessor () const { return preprocessor; }

		void setPreprocessor(PTR(ContainerPcLF) containerPcLF_) { containerPcLF = containerPcLF_; }
		void setPreprocessor(CONST_PTR(PreprocessorPc) preprocessor_) { preprocessor = preprocessor_; }

	protected:
		PTR(std::vector<ScanMeta>) scanMetaSet;
		PTR(ContainerPcRAW) containerPcRAW;
		PTR(ContainerPcLF) containerPcLF;
		CONST_PTR(PreprocessorPc) preprocessor;
	};
}

#include "ScannerPc.hpp"