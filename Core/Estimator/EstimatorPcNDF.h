#pragma once

#include "Scanner/ScannerPc.h"

#include "EstimatorPc.h"

namespace RecRoom
{
	struct NDFSample
	{
		Eigen::Vector3f tanDir;
		float diffuseAlbedo;
		float intensity;
		float weight;

		NDFSample(const Eigen::Vector3f& tanDir = Eigen::Vector3f(0.0f, 0.0f, 1.0f), float diffuseAlbedo = 0.0f, float intensity = 0.0f, float weight = 0.0f)
			: tanDir(tanDir), diffuseAlbedo(diffuseAlbedo), intensity(intensity), weight(weight)
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
			p.diffuseAlbedo = std::numeric_limits<float>::quiet_NaN();
			p.specularAlbedo = std::numeric_limits<float>::quiet_NaN();
			p.specularSharpness = std::numeric_limits<float>::quiet_NaN();
		}

	public:
		inline virtual bool SearchPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.intensity) &&
				pcl_isfinite(p.normal_x) &&
				pcl_isfinite(p.normal_y) &&
				pcl_isfinite(p.normal_z) &&
				pcl_isfinite(p.diffuseAlbedo) &&
				p.HasSerialNumber();
		}

		inline virtual bool InputPointValid(const InPointType& p) const
		{
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z);
		}

		inline virtual bool OutPointValid(const OutPointType& p) const
		{
			return pcl_isfinite(p.diffuseAlbedo) &&
				pcl_isfinite(p.specularAlbedo) &&
				pcl_isfinite(p.specularSharpness);
		}

	protected:
		inline virtual float Distribution(const Eigen::Vector3f& tanDir, float diffuseAlbedo, float specularAlbedo, float specularSharpness) const = 0;

		inline float Evaluate_MSE(const std::vector<NDFSample>& samples, float diffuseAlbedo, float specularAlbedo, float specularSharpness) const
		{
			float mse = 0.0f;
			for (int i = 0; i < samples.size(); ++i)
			{
				float diff = samples[i].intensity - Distribution(samples[i].tanDir, diffuseAlbedo, specularAlbedo, specularSharpness);
				mse += samples[i].weight * diff * diff;
			}
			return mse;
		}

		inline float Evaluate_Intensity_SpecularIntensity_MSE(const std::vector<NDFSample>& samples, float& diffuseAlbedo, float& specularAlbedo, float specularSharpness) const
		{
			float meanSpecularValues = 0;
			float meanSpecularSamples = 0;
			for (int i = 0; i < samples.size(); ++i)
			{
				float specularValues = Distribution(samples[i].tanDir, 0.0f, 1.0, specularSharpness);
				float specularSamples = std::max(samples[i].intensity - samples[i].diffuseAlbedo, 0.0f);
				meanSpecularValues += samples[i].weight * specularValues;
				meanSpecularSamples += samples[i].weight * specularSamples;
			}
			if (meanSpecularSamples > 0.0f)
				specularAlbedo = meanSpecularSamples / meanSpecularValues;
			else
				specularAlbedo = 0.0f;

			float meanDiffuseValues = 0;
			float meanDiffuseSamples = 0;
			for (int i = 0; i < samples.size(); ++i)
			{
				float specularValues = Distribution(samples[i].tanDir, 0.0f, specularAlbedo, specularSharpness);
				float diffuseSamples = std::max(samples[i].intensity - specularValues, 0.0f);
				meanDiffuseValues += samples[i].weight;
				meanDiffuseSamples += samples[i].weight * diffuseSamples;
			}
			if (meanDiffuseSamples > 0.0f)
				diffuseAlbedo = meanDiffuseSamples / meanDiffuseValues;
			else
				diffuseAlbedo = 0.0f;

			float mse = 0.0f;
			for (int i = 0; i < samples.size(); ++i)
			{
				float diff = samples[i].intensity - Distribution(samples[i].tanDir, diffuseAlbedo, specularAlbedo, specularSharpness);
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
		inline virtual float Distribution(const Eigen::Vector3f& tanDir, float diffuseAlbedo, float specularAlbedo, float specularSharpness) const
		{
			return diffuseAlbedo + specularAlbedo * std::exp(specularSharpness * (tanDir.z() - 1.0f));
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "EstimatorPcNDF.hpp"