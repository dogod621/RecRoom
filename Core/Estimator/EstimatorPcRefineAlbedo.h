#pragma once

#include "Scanner/ScannerPc.h"

#include "EstimatorPcNDF.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class EstimatorPcRefineAlbedo : public EstimatorPc<InPointType, OutPointType>
	{
	public:
		EstimatorPcRefineAlbedo(
			const CONST_PTR(ScannerPc)& scanner,
			float searchRadius,
			const float distInterParm = 3.0f,
			const float angleInterParm = 1.0f,
			const float cutFalloff = 0.33f,
			const float cutGrazing = 0.26f)
			: EstimatorPc<InPointType, OutPointType>(scanner, searchRadius, distInterParm, angleInterParm, cutFalloff, cutGrazing, 4)
		{
			name = "EstimatorPcRefineAlbedo";
		}

	protected:
		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud, const InPointType& center,
			const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const;

		inline virtual void SetAttributeNAN(OutPointType& p) const
		{
			p.diffuseAlbedo = std::numeric_limits<float>::quiet_NaN();
		}

	public:
		inline virtual bool SearchPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.intensity) &&
				pcl_isfinite(p.normal_x) &&
				pcl_isfinite(p.normal_y) &&
				pcl_isfinite(p.normal_z) &&
				pcl_isfinite(p.specularAlbedo) &&
				pcl_isfinite(p.specularSharpness) &&
				p.HasSerialNumber();
		}

		inline virtual bool InputPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z);
		}

		inline virtual bool OutputPointValid(const OutPointType& p) const
		{
			return pcl_isfinite(p.diffuseAlbedo);
		}

	protected:
		inline virtual float DiffuseDistribution(const Eigen::Vector3f& tanDir) const = 0;

		inline virtual float SpecularDistribution(const Eigen::Vector3f& tanDir, float specularSharpness) const = 0;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template<class InPointType, class OutPointType>
	class EstimatorPcRefineSGAlbedo : public EstimatorPcRefineAlbedo<InPointType, OutPointType>
	{
	public:
		EstimatorPcRefineSGAlbedo(
			const CONST_PTR(ScannerPc)& scanner,
			float searchRadius,
			const float distInterParm = 3.0f,
			const float angleInterParm = 1.0f,
			const float cutFalloff = 0.33f,
			const float cutGrazing = 0.26f)
			: EstimatorPcRefineAlbedo<InPointType, OutPointType>(scanner, searchRadius, distInterParm, angleInterParm, cutFalloff, cutGrazing)
		{
			name = "EstimatorPcRefineSGAlbedo";
		}

	protected:
		inline virtual float DiffuseDistribution(const Eigen::Vector3f& tanDir) const
		{
			return 1.0f;
		}

		inline virtual float SpecularDistribution(const Eigen::Vector3f& tanDir, float specularSharpness) const
		{
			return std::exp(specularSharpness * (tanDir.z() - 1.0f));
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "EstimatorPcRefineAlbedo.hpp"