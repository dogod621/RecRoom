#pragma once

#include "Common/LBFGSB.h"

#include "EstimatorPcNDF.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	struct Temp
	{
		const EstimatorPcNDF<InPointType, OutPointType>* self;
		const std::vector<NDFSample>* samples;

		Temp(const EstimatorPcNDF<InPointType, OutPointType>* self = nullptr, const std::vector<NDFSample>* samples = nullptr)
			: self(self), samples(samples) {}
	};

	template<class InPointType, class OutPointType>
	double ObjValue(const Eigen::VectorXd& x, void* data)
	{
		Temp<InPointType, OutPointType>& temp = *((Temp<InPointType, OutPointType>*)(data));

		const EstimatorPcNDF<InPointType, OutPointType>& self = *temp.self;
		const std::vector<NDFSample>& samples = *temp.samples;

		double specularAlbedo = x(0);
		double specularSharpness = x(1);

		double r = 0.0;
		for (std::vector<NDFSample>::const_iterator it = samples.cbegin(); it != samples.cend(); ++it)
		{
			double diff = it->intensity64 - it->tanDir64.z() * (
				specularAlbedo * self.SpecularDistribution(it->tanDir64, specularSharpness));
			r += it->weight64 * diff * diff;
		}
		return r;
	}

	template<class InPointType, class OutPointType>
	void ObjGradient(const Eigen::VectorXd& x, Eigen::VectorXd& g, void* data)
	{
		Temp<InPointType, OutPointType>& temp = *((Temp<InPointType, OutPointType>*)(data));

		const EstimatorPcNDF<InPointType, OutPointType>& self = *temp.self;
		const std::vector<NDFSample>& samples = *temp.samples;

		double specularAlbedo = x(0);
		double specularSharpness = x(1);

		g = Eigen::VectorXd(2);
		g(0) = 0.0;
		g(1) = 0.0;
		for (std::vector<NDFSample>::const_iterator it = samples.cbegin(); it != samples.cend(); ++it)
		{
			double specularDistribution = self.SpecularDistribution(it->tanDir64, specularSharpness);
			double specularDistributionGradient = self.SpecularDistributionGradient(it->tanDir64, specularSharpness);
			double diff = it->intensity64 - it->tanDir64.z() * (specularAlbedo * specularDistribution);
			double temp = -2.0 * it->weight64 * diff * it->tanDir64.z();
			g(1) += temp * specularDistribution;
			g(2) += temp * specularAlbedo * specularDistributionGradient;
		}
	}

	template<class InPointType, class OutPointType>
	inline bool EstimatorPcNDF<InPointType, OutPointType>::ComputeAttribute(
		const Pc<InPointType>& cloud, const InPointType& center,
		const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const
	{
		std::vector<NDFSample> samples;
		samples.reserve(scanDataSet.size());
		float sumWeight = 0.0;
		std::vector<uint32_t> temp;
		temp.reserve(scanDataSet.size());
		for (std::vector<ScanData>::const_iterator it = scanDataSet.begin(); it != scanDataSet.end(); ++it)
		{
			const InPointType& hitPoint = cloud[it->index];
			Eigen::Vector3f hitNormal(hitPoint.normal_x, hitPoint.normal_y, hitPoint.normal_z);
			Eigen::Vector3f hitTangent;
			Eigen::Vector3f hitBitangent;
			if (!Common::GenFrame(hitNormal, hitTangent, hitBitangent))
			{
				PRINT_WARNING("GenFrame failed, ignore");
				return false;
			}

			Eigen::Vector3f hafway = it->laser.incidentDirection + it->laser.reflectedDirection;
			float hafwayNorm = hafway.norm();
			if (hafwayNorm > std::numeric_limits<float>::epsilon())
			{
				hafway /= hafwayNorm;
				//float dotLN = hitNormal.dot(it->laser.incidentDirection);
				float weight = DistInterWeight(searchRadius, it->distance2Center, distInterParm) * AngleInterWeight(hitNormal, it->laser.incidentDirection, angleInterParm);
				//float intensity = it->laser.intensity / (it->laser.beamFalloff * dotLN);
				float intensity = it->laser.intensity / it->laser.beamFalloff;
				sumWeight += weight;
				samples.push_back(NDFSample(
					hitPoint.diffuseAlbedo,
					hitPoint.specularAlbedo,
					hitPoint.specularSharpness,
					Eigen::Vector3f(
						hitTangent.dot(hafway),
						hitBitangent.dot(hafway),
						hitNormal.dot(hafway)),
					intensity, weight));
				temp.push_back(hitPoint.serialNumber);
			}
		}
		for (std::vector<NDFSample>::iterator it = samples.begin(); it != samples.end(); ++it)
		{
			it->weight /= sumWeight;
		}
		std::sort(temp.begin(), temp.end());
		if (std::distance(temp.begin(), std::unique(temp.begin(), temp.end())) < minRequireNumData)
			return false;

		const int numInitSharpness = 32;
		float epsInitSpecularSharpness = (maxSharpness - minSharpness) / (float)(numInitSharpness);
		float bestMSE = std::numeric_limits<float>::max();
		for (int testInitSpecularSharpness = 0; testInitSpecularSharpness < numInitSharpness; ++testInitSpecularSharpness)
		{
			float testDiffuseAlbedo;
			float testSpecularAlbedo;
			float testSpecularSharpness = ((float)testInitSpecularSharpness + 0.5f) * epsInitSpecularSharpness;
			float testMSE = Evaluate_Albedo_MSE(samples, testDiffuseAlbedo, testSpecularAlbedo, testSpecularSharpness);

			if (testMSE < bestMSE)
			{
				outPoint.diffuseAlbedo = testDiffuseAlbedo;
				outPoint.specularAlbedo = testSpecularAlbedo;
				outPoint.specularSharpness = testSpecularSharpness;

				bestMSE = testMSE;
			}
		}

		//
		for (int i = 0; i < samples.size(); ++i)
		{
			float diffuseValue = samples[i].tanDir.z() * DiffuseDistribution(samples[i].tanDir);
			samples[i].intensity = std::max(samples[i].intensity - outPoint.diffuseAlbedo * diffuseValue, 0.0f);
		}

		//
		Eigen::VectorXd lowerBound(2);
		Eigen::VectorXd upperBound(2);
		lowerBound << (0.1f * outPoint.diffuseAlbedo), minSharpness;
		upperBound << 512.0, maxSharpness;
		LBFGSB solver(lowerBound, upperBound);
		
		Eigen::VectorXd optX(2);
		optX << outPoint.specularAlbedo, outPoint.specularSharpness;

		Temp<InPointType, OutPointType> data(this, &samples);
		solver.Solve(optX, ObjValue<InPointType, OutPointType>, ObjGradient<InPointType, OutPointType>, (void*)(&data));

		outPoint.specularAlbedo = optX(0);
		outPoint.specularSharpness = optX(1);

		if (!OutputPointValid(outPoint))
			return false;

		if( (optX(0) < lowerBound(0)) ||
			(optX(0) > upperBound(0)) || 
			(optX(1) < lowerBound(1)) ||
			(optX(1) > upperBound(1)))
			return false;

		return true;
	}
}