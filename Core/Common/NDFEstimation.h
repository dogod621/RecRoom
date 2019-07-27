#pragma once

#include <vector>

#include "AttributeEstimation.h"

namespace RecRoom
{
	enum NDF : Flag
	{
		NDF_UNKNOWN = 0,
		SG = 1
	};


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
	class NDFEstimation : public AttributeEstimation<InPointType, OutPointType>
	{
	public:
		NDFEstimation(
			const CONST_PTR(ScannerPc)& scanner,
			const float distInterParm = 0.4f, const float angleInterParm = 0.6f,
			const float cutFalloff = 0.33f, // cut attinuation less than 1/3
			const float cutGrazing = 0.26f // cut incident agngle larger than 75 degrees
		) : AttributeEstimation<InPointType, OutPointType>(scanner, cutFalloff, cutGrazing, 4),
			distInterParm(distInterParm), angleInterParm(angleInterParm), minSharpness(0.0f), maxSharpness(1.0f), threshSNR(90.f)
		{
			feature_name_ = "NDFEstimation";
		};

	protected:
		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud,
			const InPointType& center, const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const;

		inline virtual void SetAttributeNAN(OutPointType& p) const
		{
			p.intensity = std::numeric_limits<float>::quiet_NaN();
			p.sharpness = std::numeric_limits<float>::quiet_NaN();
		}

	protected:
		float distInterParm;
		float angleInterParm;

		float minSharpness;
		float maxSharpness;
		float threshSNR;

		inline virtual float Distribution(const Eigen::Vector3f& tanDir, float sharpness) const = 0;

		inline void EvaluateMSE(const std::vector<NDFSample>& samples, float sharpness, float meanIntensity, float sumWeight,
			std::vector<float>& ndfValues, float& intensity, float& mse) const
		{
			float meanNDF = 0.0;
			for (std::size_t i = 0; i < samples.size(); ++i)
			{
				ndfValues[i] = Distribution(samples[i].tanDir, sharpness);
				meanNDF += samples[i].weight * ndfValues[i];
			}
			meanNDF /= sumWeight;

			intensity = meanIntensity / meanNDF;
			mse = 0.0f;
			for (std::size_t i = 0; i < samples.size(); ++i)
			{
				ndfValues[i] *= intensity;
				float diff = samples[i].intensity - ndfValues[i];
				mse += samples[i].weight * diff * diff;
			}
			mse /= sumWeight;
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template<class InPointType, class OutPointType>
	class SGEstimation : public NDFEstimation<InPointType, OutPointType>
	{
	public:
		SGEstimation(
			const CONST_PTR(ScannerPc)& scanner,
			const float distInterParm = 0.4f, const float angleInterParm = 0.6f,
			const float cutFalloff = 0.33f, // cut attinuation less than 1/3
			const float cutGrazing = 0.26f // cut incident agngle larger than 75 degrees
		) : NDFEstimation<InPointType, OutPointType>(scanner, distInterParm, angleInterParm, cutFalloff, cutGrazing)
		{
			feature_name_ = "SGEstimation";

			minSharpness = 0.0f;
			maxSharpness = 6.0f;
		};

	protected:
		inline virtual float Distribution(const Eigen::Vector3f& tanDir, float sharpness) const
		{
			return std::exp(sharpness * (tanDir.z() - 1.0f));
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "NDFEstimation.hpp"
