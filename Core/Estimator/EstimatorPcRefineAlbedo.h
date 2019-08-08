/*#pragma once

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
				pcl_isfinite(p.diffuseAlbedo) &&
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

		inline virtual bool OutPointValid(const OutPointType& p) const
		{
			return pcl_isfinite(p.diffuseAlbedo);
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

		inline float Evaluate_Albedo_MSE(const std::vector<NDFSample>& samples, float& diffuseAlbedo, float& specularAlbedo, float specularSharpness) const
		{
			float meanDiffuseValues = 0;
			float meanNDFValues = 0;
			std::vector<float> ndf(samples.size());
			for (int i = 0; i < samples.size(); ++i)
			{
				ndf[i] = Distribution(samples[i].tanDir, 0.0f, 1.0, specularSharpness);
				meanDiffuseValues += samples[i].weight * samples[i].tanDir.z();
				meanNDFValues += samples[i].weight * ndf[i];
			}

			//
			float meanSpecularSamples = 0;
			for (int i = 0; i < samples.size(); ++i)
				meanSpecularSamples += samples[i].weight * std::max(samples[i].intensity - samples[i].tanDir.z() * samples[i].diffuseAlbedo, 0.0f);
			if (meanSpecularSamples > 0.0f)
				specularAlbedo = meanSpecularSamples / meanNDFValues;
			else
				specularAlbedo = 0.0f;

			//
			float meanDiffuseSamples = 0;
			for (int i = 0; i < samples.size(); ++i)
				meanDiffuseSamples += samples[i].weight * std::max(samples[i].intensity - specularAlbedo * ndf[i], 0.0f);
			if (meanDiffuseSamples > 0.0f)
				diffuseAlbedo = meanDiffuseSamples / meanDiffuseValues;
			else
				diffuseAlbedo = 0.0f;

			//
			meanSpecularSamples = 0;
			for (int i = 0; i < samples.size(); ++i)
				meanSpecularSamples += samples[i].weight * std::max(samples[i].intensity - samples[i].tanDir.z() * diffuseAlbedo, 0.0f);
			if (meanSpecularSamples > 0.0f)
				specularAlbedo = meanSpecularSamples / meanNDFValues;
			else
				specularAlbedo = 0.0f;

			//
			meanDiffuseSamples = 0;
			for (int i = 0; i < samples.size(); ++i)
				meanDiffuseSamples += samples[i].weight * std::max(samples[i].intensity - specularAlbedo * ndf[i], 0.0f);
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
			: EstimatorPcNDF<InPointType, OutPointType>(scanner, searchRadius, distInterParm, angleInterParm, cutFalloff, cutGrazing)
		{
			name = "EstimatorPcRefineSGAlbedo";
		}

	protected:
		inline virtual float Distribution(const Eigen::Vector3f& tanDir, float diffuseAlbedo, float specularAlbedo, float specularSharpness) const
		{
			return tanDir.z() * (diffuseAlbedo + specularAlbedo * std::exp(specularSharpness * (tanDir.z() - 1.0f)));
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "EstimatorPcRefineAlbedo.hpp"*/