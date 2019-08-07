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
			: EstimatorPc<InPointType, OutPointType>(scanner, searchRadius, distInterParm, angleInterParm, cutFalloff, cutGrazing, 6),
			minSharpness(0.0f), maxSharpness(1.0f)
		{
			name = "EstimatorPcNDF";
		}

	protected:
		inline virtual bool ComputeAttribute(
			const Pc<InPointType>& cloud, const InPointType& center,
			const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const;

		inline virtual void SetAttributeNAN(OutPointType& p) const
		{
			p.intensity = std::numeric_limits<float>::quiet_NaN();
			p.sharpness = std::numeric_limits<float>::quiet_NaN();
			p.specularIntensity = std::numeric_limits<float>::quiet_NaN();
		}

	public:
		inline virtual bool SearchPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.normal_x) &&
				pcl_isfinite(p.normal_y) &&
				pcl_isfinite(p.normal_z) &&
				p.HasSerialNumber();
		}

		inline virtual bool InputPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z) &&
				pcl_isfinite(p.intensity);
		}

		inline virtual bool OutPointValid(const OutPointType& p) const
		{
			return pcl_isfinite(p.intensity) &&
				pcl_isfinite(p.sharpness) &&
				pcl_isfinite(p.specularIntensity);
		}

	protected:
		inline virtual float Distribution(const Eigen::Vector3f& tanDir, float intensity, float sharpness, float specularIntensity) const = 0;

		inline float Evaluate_MSE(const std::vector<NDFSample>& samples, float intensity, float sharpness, float specularIntensity) const
		{
			float mse = 0.0f;
			for (int i = 0; i < samples.size(); ++i)
			{
				float diff = samples[i].intensity - Distribution(samples[i].tanDir, intensity, sharpness, specularIntensity);
				mse += samples[i].weight * diff * diff;
			}
			return mse;
		}

		inline float Evaluate_SpecularIntensity_MSE(const std::vector<NDFSample>& samples, float intensity, float sharpness, float& specularIntensity) const
		{
			std::vector<float> specularValues(samples.size());
			std::vector<float> specularSamples(samples.size());
			float meanSpecularValues = 0;
			float meanSpecularSamples = 0;
			for (int i = 0; i < samples.size(); ++i)
			{
				specularValues[i] = Distribution(samples[i].tanDir, 0.0f, sharpness, 1.0f);
				specularSamples[i] = std::max(samples[i].intensity - intensity, 0.0f);
				meanSpecularValues += samples[i].weight * specularValues[i];
				meanSpecularSamples += samples[i].weight * specularSamples[i];
			}
			specularIntensity = meanSpecularSamples / meanSpecularValues;
			float mse = 0.0f;
			for (int i = 0; i < samples.size(); ++i)
			{
				float diff = samples[i].intensity - Distribution(samples[i].tanDir, intensity, sharpness, specularIntensity);
				mse += samples[i].weight * diff * diff;
			}
			return mse;
		}

	protected:
		float minSharpness;
		float maxSharpness;

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
			maxSharpness = 10.0f;
		}

	protected:
		inline virtual float Distribution(const Eigen::Vector3f& tanDir, float intensity, float sharpness, float specularIntensity) const
		{
			return intensity + specularIntensity * std::exp(sharpness * (tanDir.z() - 1.0f));
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "EstimatorPcNDF.hpp"