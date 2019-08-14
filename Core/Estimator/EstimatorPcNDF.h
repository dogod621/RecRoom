#pragma once

#include "Scanner/ScannerPc.h"

#include "EstimatorPc.h"

namespace RecRoom
{
	struct NDFSample
	{
		float initDiffuseAlbedo;
		float initSpecularAlbedo;
		float initSpecularSharpness;

		Eigen::Vector3f tanDir;
		Eigen::Vector3d tanDir64;
		float intensity;
		double intensity64;
		float weight;
		double weight64;

		NDFSample(
			float initDiffuseAlbedo = 0.0f, float initSpecularAlbedo = 0.0f, float initSpecularSharpness = 0.0f,
			const Eigen::Vector3f& tanDir = Eigen::Vector3f(0.0f, 0.0f, 1.0f), float intensity = 0.0f, float weight = 0.0f)
			: 
			initDiffuseAlbedo(initDiffuseAlbedo), initSpecularAlbedo(initSpecularAlbedo), initSpecularSharpness(initSpecularSharpness),
			tanDir(tanDir), tanDir64(tanDir.x(), tanDir.y(), tanDir.z()), 
			intensity(intensity), intensity64(intensity),
			weight(weight), weight64(weight)
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
			minSharpness(0.0f), maxSharpness(1.0f), optimization(true)
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

		inline virtual bool OutputPointValid(const OutPointType& p) const
		{
			return pcl_isfinite(p.diffuseAlbedo) &&
				pcl_isfinite(p.specularAlbedo) &&
				pcl_isfinite(p.specularSharpness);
		}

	public:
		inline bool SolveSpecular(OutPointType& outPoint, const PcNDF& rawSamples) const;

		inline virtual float DiffuseDistribution(const Eigen::Vector3f& tanDir) const = 0;

		inline virtual float SpecularDistribution(const Eigen::Vector3f& tanDir, float specularSharpness) const = 0;

		inline virtual float SpecularDistributionGradient(const Eigen::Vector3f& tanDir, float specularSharpness) const = 0;

		inline virtual double DiffuseDistribution(const Eigen::Vector3d& tanDir) const = 0;

		inline virtual double SpecularDistribution(const Eigen::Vector3d& tanDir, double specularSharpness) const = 0;

		inline virtual double SpecularDistributionGradient(const Eigen::Vector3d& tanDir, double specularSharpness) const = 0;

	protected:
		inline float Evaluate_MSE(const std::vector<NDFSample>& samples, float diffuseAlbedo, float specularAlbedo, float specularSharpness) const
		{
			float mse = 0.0f;

			for (std::vector<NDFSample>::const_iterator it = samples.cbegin(); it != samples.cend(); ++it)
			{
				double diff = it->intensity - it->tanDir.z() * (
					diffuseAlbedo * DiffuseDistribution(it->tanDir) +
					specularAlbedo * SpecularDistribution(it->tanDir, specularSharpness));
				mse += it->weight * diff * diff;
			}
			return mse;
		}

		inline float Evaluate_Albedo_MSE(const std::vector<NDFSample>& samples, float& specularAlbedo, float specularSharpness) const
		{
			float meanSpecularValues = 0;
			std::vector<float> specularValue(samples.size());
			for (int i = 0; i < samples.size(); ++i)
			{
				specularValue[i] = samples[i].tanDir.z() * SpecularDistribution(samples[i].tanDir, specularSharpness);
				meanSpecularValues += samples[i].weight * specularValue[i];
			}

			float temp = 0;
			for (int i = 0; i < samples.size(); ++i)
				temp += samples[i].weight * samples[i].intensity;
			if (temp > 0.0f)
				specularAlbedo = temp / meanSpecularValues;
			else
				specularAlbedo = 0.0f;

			return Evaluate_MSE(samples, 0.0, specularAlbedo, specularSharpness);
		}

		inline float Evaluate_Albedo_MSE(const std::vector<NDFSample>& samples, float& diffuseAlbedo, float& specularAlbedo, float specularSharpness) const
		{
			float meanDiffuseValues = 0;
			float meanSpecularValues = 0;
			std::vector<float> diffuseValue(samples.size());
			std::vector<float> specularValue(samples.size());
			for (int i = 0; i < samples.size(); ++i)
			{
				diffuseValue[i] = samples[i].tanDir.z() * DiffuseDistribution(samples[i].tanDir);
				specularValue[i] = samples[i].tanDir.z() * SpecularDistribution(samples[i].tanDir, specularSharpness);
				meanDiffuseValues += samples[i].weight * diffuseValue[i];
				meanSpecularValues += samples[i].weight * specularValue[i];
			}

			//
			{
				float temp = 0;
				for (int i = 0; i < samples.size(); ++i)
					temp += samples[i].weight * std::max(samples[i].intensity - samples[i].initDiffuseAlbedo * diffuseValue[i], 0.0f);
				if (temp > 0.0f)
					specularAlbedo = temp / meanSpecularValues;
				else
					specularAlbedo = 0.0f;
			}

			//
			for (int iter = 0; iter < 5; iter++)
			{
				{
					float temp = 0;
					for (int i = 0; i < samples.size(); ++i)
						temp += samples[i].weight * std::max(samples[i].intensity - specularAlbedo * specularValue[i], 0.0f);
					if (temp > 0.0f)
						diffuseAlbedo = temp / meanDiffuseValues;
					else
						diffuseAlbedo = 0.0f;
				}

				//
				{
					float temp = 0;
					for (int i = 0; i < samples.size(); ++i)
						temp += samples[i].weight * std::max(samples[i].intensity - diffuseAlbedo * diffuseValue[i], 0.0f);
					if (temp > 0.0f)
						specularAlbedo = std::max(temp / meanSpecularValues, diffuseAlbedo * 0.1f);
					else
						specularAlbedo = diffuseAlbedo * 0.1f;
				}
			}

			//
			{
				float temp = 0;
				for (int i = 0; i < samples.size(); ++i)
					temp += samples[i].weight * std::max(samples[i].intensity - specularAlbedo * specularValue[i], 0.0f);
				if (temp > 0.0f)
					diffuseAlbedo = temp / meanDiffuseValues;
				else
					diffuseAlbedo = 0.0f;
			}

			float mse = 0.0f;
			for (int i = 0; i < samples.size(); ++i)
			{
				float diff = samples[i].intensity - (diffuseAlbedo * diffuseValue[i] + specularAlbedo * specularValue[i]);
				mse += samples[i].weight * diff * diff;
			}
			return mse;
		}

	protected:
		float minSharpness;
		float maxSharpness;
		bool optimization;

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

			minSharpness = 1.0f;
			maxSharpness = 25.0f;
			optimization = false;
		}

	public:
		inline virtual float DiffuseDistribution(const Eigen::Vector3f& tanDir) const
		{
			return 1.0f;
		}

		inline virtual float SpecularDistribution(const Eigen::Vector3f& tanDir, float specularSharpness) const
		{
			return std::exp(specularSharpness * (tanDir.z() - 1.0f));
		}

		inline virtual float SpecularDistributionGradient(const Eigen::Vector3f& tanDir, float specularSharpness) const
		{
			return std::exp(specularSharpness * (tanDir.z() - 1.0f)) * (tanDir.z() - 1.0f);
		}

		inline virtual double DiffuseDistribution(const Eigen::Vector3d& tanDir) const
		{
			return 1.0;
		}

		inline virtual double SpecularDistribution(const Eigen::Vector3d& tanDir, double specularSharpness) const
		{
			return std::exp(specularSharpness * (tanDir.z() - 1.0));
		}

		inline virtual double SpecularDistributionGradient(const Eigen::Vector3d& tanDir, double specularSharpness) const
		{
			return std::exp(specularSharpness * (tanDir.z() - 1.0)) * (tanDir.z() - 1.0);
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "EstimatorPcNDF.hpp"