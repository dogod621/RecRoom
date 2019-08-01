#pragma once

#include "Scanner/ScannerPc.h"

#include "EstimatorPc.h"

namespace RecRoom
{
	struct NDFSample
	{
		Eigen::Vector3f tanDir;
		float intensity;
		float weight;

		NDFSample(const Eigen::Vector3f& tanDir = Eigen::Vector3f(0.0f, 0.0f, 1.0f), float intensity = 0.0f, float weight = 0.0f)
			: tanDir(tanDir), intensity(intensity), weight(weight)
		{}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template<class InPointType, class OutPointType>
	class EstimatorPcNDF : public EstimatorPc<InPointType, OutPointType>
	{
	public:
		EstimatorPcNDF(
			const CONST_PTR(ScannerPc)& scanner,
			float searchRadius,
			const float distInterParm = 3.0f,
			const float angleInterParm = 1.0f,
			const float cutFalloff = 0.33f,
			const float cutGrazing = 0.26f)
			: EstimatorPc<InPointType, OutPointType>(scanner, searchRadius, distInterParm, angleInterParm, cutFalloff, cutGrazing, 4),
			minSharpness(0.0f), maxSharpness(1.0f), threshSNR(90.f)
		{
			name = "EstimatorPcNDF";
		}

	protected:
		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud,
			const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const;

		inline virtual void SetAttributeNAN(OutPointType& p) const
		{
			p.intensity = std::numeric_limits<float>::quiet_NaN();
			p.sharpness = std::numeric_limits<float>::quiet_NaN();
		}

	public:
		inline virtual bool SearchPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.normal_x) &&
				pcl_isfinite(p.normal_y) &&
				pcl_isfinite(p.normal_z) &&
				p.HasSerialNumber();
		}

		inline virtual bool OutPointValid(const OutPointType& p) const
		{
			return pcl_isfinite(p.intensity) &&
				pcl_isfinite(p.sharpness);
		}

	protected:
		inline virtual float Distribution(const Eigen::Vector3f& tanDir, float sharpness) const = 0;

		inline void EvaluateMSE(const std::vector<NDFSample>& samples, float sharpness, float meanIntensity, float sumWeight,
			std::vector<float>& ndfValues, float& intensity, float& mse) const
		{
			float meanNDF = 0.0;
			for (int i = 0; i < samples.size(); ++i)
			{
				ndfValues[i] = Distribution(samples[i].tanDir, sharpness);
				meanNDF += samples[i].weight * ndfValues[i];
			}
			meanNDF /= sumWeight;

			intensity = meanIntensity / meanNDF;
			mse = 0.0f;
			for (int i = 0; i < samples.size(); ++i)
			{
				ndfValues[i] *= intensity;
				float diff = samples[i].intensity - ndfValues[i];
				mse += samples[i].weight * diff * diff;
			}
			mse /= sumWeight;
		}

	protected:
		float minSharpness;
		float maxSharpness;
		float threshSNR;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template<class InPointType, class OutPointType>
	class EstimatorPcSGNDF : public EstimatorPcNDF<InPointType, OutPointType>
	{
	public:
		EstimatorPcSGNDF(
			const CONST_PTR(ScannerPc)& scanner,
			float searchRadius,
			const float distInterParm = 3.0f,
			const float angleInterParm = 1.0f,
			const float cutFalloff = 0.33f,
			const float cutGrazing = 0.26f)
			: EstimatorPcNDF<InPointType, OutPointType>(scanner, searchRadius, distInterParm, angleInterParm, cutFalloff, cutGrazing)
		{
			name = "EstimatorPcSGNDF";

			minSharpness = 0.0f;
			maxSharpness = 6.0f;
		}

	protected:
		inline virtual float Distribution(const Eigen::Vector3f& tanDir, float sharpness) const
		{
			return std::exp(sharpness * (tanDir.z() - 1.0f));
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "EstimatorPcNDF.hpp"