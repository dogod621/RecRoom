#pragma once

#include "Common/LBFGSB.h"

#include "EstimatorPcNDF.h"

namespace RecRoom
{
	double SG_Distribution_ObjValue(const Eigen::VectorXd& x, void* data)
	{
		std::vector<NDFSample>& samples = *((std::vector<NDFSample>*)(data));

		double diffuseAlbedo = x(0);
		double specularAlbedo = x(1);
		double specularSharpness = x(2);

		double r = 0.0;
		for (std::vector<NDFSample>::iterator it = samples.begin(); it != samples.end(); ++it)
		{
			double nFun = std::exp(specularSharpness * (it->tanDir.z() - 1.0));
			double diff = it->intensity - (diffuseAlbedo + specularAlbedo * nFun);
			r += it->weight * diff * diff;
		}
		return r;
	}

	void SG_Distributio_ObjGradient(const Eigen::VectorXd& x, Eigen::VectorXd& g, void* data)
	{
		std::vector<NDFSample>& samples = *((std::vector<NDFSample>*)(data));

		double diffuseAlbedo = x(0);
		double specularAlbedo = x(1);
		double specularSharpness = x(2);

		g = Eigen::VectorXd(3);
		g(0) = 0.0;
		g(1) = 0.0;
		g(2) = 0.0;
		for (std::vector<NDFSample>::iterator it = samples.begin(); it != samples.end(); ++it)
		{
			double nFun = std::exp(specularSharpness * (it->tanDir.z() - 1.0));
			double diff = it->intensity - (diffuseAlbedo + specularAlbedo * nFun);
			double temp = -2.0 * it->weight * diff;
			g(0) += temp;
			g(1) += temp * nFun;
			g(2) += temp * (specularAlbedo * nFun * (it->tanDir.z() - 1.0));
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
					Eigen::Vector3f(
						hitTangent.dot(hafway),
						hitBitangent.dot(hafway),
						hitNormal.dot(hafway)),
					hitPoint.diffuseAlbedo,
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
			float testMSE = Evaluate_Intensity_SpecularIntensity_MSE(samples, testDiffuseAlbedo, testSpecularAlbedo, testSpecularSharpness);

			if (testMSE < bestMSE)
			{
				outPoint.diffuseAlbedo = testDiffuseAlbedo;
				outPoint.specularAlbedo = testSpecularAlbedo;
				outPoint.specularSharpness = testSpecularSharpness;

				bestMSE = testMSE;
			}
		}

		Eigen::VectorXd lowerBound(3);
		Eigen::VectorXd upperBound(3);
		lowerBound << 0.0, 0.0, minSharpness;
		upperBound << 512.0, 512.0, maxSharpness;
		LBFGSB solver(lowerBound, upperBound);
		
		Eigen::VectorXd optX(3);
		optX << outPoint.diffuseAlbedo, outPoint.specularAlbedo, outPoint.specularSharpness;

		solver.Solve(optX, SG_Distribution_ObjValue, SG_Distributio_ObjGradient, (void*)(&samples));

		outPoint.diffuseAlbedo = optX(0);
		outPoint.specularAlbedo = optX(2);
		outPoint.specularSharpness = optX(1);

		if (!OutPointValid(outPoint))
			return false;

		if( (optX(0) < lowerBound(0)) ||
			(optX(0) > upperBound(0)) || 
			(optX(1) < lowerBound(1)) ||
			(optX(1) > upperBound(1)) ||
			(optX(2) < lowerBound(2)) ||
			(optX(2) > upperBound(2)))
			return false;

		return true;
	}
}