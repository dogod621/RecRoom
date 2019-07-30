#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"
#include "Scanner/ScannerPc.h"

namespace RecRoom
{
	struct ScanData
	{
		ScanLaser laser;
		int index;
		float distance2Center;

		ScanData(const ScanLaser& laser = ScanLaser(), int index = -1, float distance2Center = 0.0f) : laser(laser), index(index), distance2Center(distance2Center) {}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template<class InPointType, class OutPointType>
	class EstimatorPc 
		: public SearchAnySurfaceProcessorPc2PcInOut<InPointType, OutPointType>, public ThreadAble
	{
	public:
		EstimatorPc(
			const CONST_PTR(ScannerPc)& scanner,
			float searchRadius,
			const float distInterParm,
			const float angleInterParm,
			const float cutFalloff = 0.33f, 
			const float cutGrazing = 0.26f,
			const int minRequireNumData = 1)
			: SearchAnySurfaceProcessorPc2PcInOut<InPointType, OutPointType>(), ThreadAble(),
			scanner(scanner),
			searchRadius(searchRadius),
			distInterParm(distInterParm), 
			angleInterParm(angleInterParm),
			cutFalloff(cutFalloff), 
			cutGrazing(cutGrazing),
			minRequireNumData(minRequireNumData)
		{
			name = "EstimatorPc";

			if (!scanner)
				THROW_EXCEPTION("scanner is not set");

			if (searchRadius <= 0.0f)
				THROW_EXCEPTION("searchRadius is not valid");

			if (minRequireNumData < 1)
				THROW_EXCEPTION("minRequireNumData < 1");
		}

	protected:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<InPointType>)& searchSurface,
			const CONST_PTR(Pc<InPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutPointType>& output) const;

	protected:
		inline bool CollectScanData(
			const Pc<InPointType>& cloud,
			const PcIndex& nnIndices, const std::vector<float>& nnSqrDists,
			std::vector<ScanData>& scanDataSet) const;

		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud,
			const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const = 0;

		inline virtual float DistInterWeight(float searchRadius, float distance, float interParm) const
		{
			// return std::exp(-interParm * distance/searchRadius); gaussian
			return std::pow((searchRadius - distance) / searchRadius, interParm);
		}

		inline virtual float AngleInterWeight(const Eigen::Vector3f& normal, const Eigen::Vector3f& direction, float interParm) const
		{
			float dotNN = normal.dot(direction);
			// return std::exp(interParm * (dotNN - 1.0f)); // sperical gaussian
			return std::pow(dotNN, interParm); // phone
		}

		inline virtual bool ScanLaserValid(const Eigen::Vector3f& normal, const ScanLaser& scanLaser)const
		{
			return (
				(scanLaser.incidentDirection.dot(normal) > cutGrazing) &&
				(scanLaser.beamFalloff > cutFalloff));
		}

		inline virtual void SetAttributeNAN(OutPointType& p) const = 0;

		inline virtual bool SearchPointValid(const InPointType& p) const
		{
			THROW_EXCEPTION("Interface is not implemented");
			return false;
		}

		inline virtual bool InputPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z);
		}

		inline virtual bool OutPointValid(const OutPointType& p) const
		{
			THROW_EXCEPTION("Interface is not implemented");
			return false;
		}

	public:
		CONST_PTR(ScannerPc) getScanner() const { return scanner; }
		float getSearchRadius() const { return searchRadius; }
		float getDistInterParm() const { return distInterParm; }
		float getAngleInterParm() const { return angleInterParm; }
		float getCutFalloff() const { return cutFalloff; }
		float getCutGrazing() const { return cutGrazing; }
		int getMinRequireNumData() const { return minRequireNumData; }

		void setScanner(CONST_PTR(ScannerPc) v)
		{
			if (!v)
			{
				THROW_EXCEPTION("scanner is not set");
			}
			else
			{
				scanner = v;
			}
		}
		void setSearchRadius(double v)
		{
			if (v <= 0.0)
			{
				THROW_EXCEPTION("searchRadius is not valid");
			}
			else
				searchRadius = v;
		}
		void setDistInterParm(float v) { distInterParm = v; }
		void setAngleInterParm(float v) { angleInterParm = v; }
		void setCutFalloff(float v) { cutFalloff = v; }
		void setCutGrazing(float v) { cutGrazing = v; }
		void setMinRequireNumData(int v)
		{
			if (v < 1)
			{
				THROW_EXCEPTION("minRequireNumData < 1");
			}
			else
			{
				minRequireNumData = v;
			}
		}

	protected:
		CONST_PTR(ScannerPc) scanner;
		float searchRadius;
		float distInterParm;
		float angleInterParm;
		float cutFalloff;
		float cutGrazing;
		int minRequireNumData;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "EstimatorPc.hpp"