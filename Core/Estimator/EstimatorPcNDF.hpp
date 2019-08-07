#pragma once

#include "Common/LBFGSB.h"

#include "EstimatorPcNDF.h"

namespace RecRoom
{
	double SG_Distribution_ObjValue(const Eigen::VectorXd& x, void* data)
	{
		std::vector<NDFSample>& samples = *((std::vector<NDFSample>*)(data));

		double intensity = x(0);
		double sharpness = x(1);
		double specularIntensity = x(2);

		double r = 0.0;
		for (std::vector<NDFSample>::iterator it = samples.begin(); it != samples.end(); ++it)
		{
			double nFun = std::exp(sharpness * (it->tanDir.z() - 1.0));
			double diff = it->intensity - (intensity + specularIntensity * nFun);
			r += it->weight * diff * diff;
		}
		return r;
	}

	void SG_Distributio_ObjGradient(const Eigen::VectorXd& x, Eigen::VectorXd& g, void* data)
	{
		std::vector<NDFSample>& samples = *((std::vector<NDFSample>*)(data));

		double intensity = x(0);
		double sharpness = x(1);
		double specularIntensity = x(2);

		g = Eigen::VectorXd(3);
		g(0) = 0.0;
		g(1) = 0.0;
		g(2) = 0.0;
		for (std::vector<NDFSample>::iterator it = samples.begin(); it != samples.end(); ++it)
		{
			double nFun = std::exp(sharpness * (it->tanDir.z() - 1.0));
			double diff = it->intensity - (intensity + specularIntensity * nFun);
			double temp = -2.0 * it->weight * diff;
			g(0) += temp;
			g(1) += temp * (specularIntensity * nFun * (it->tanDir.z() - 1.0));
			g(2) += temp * nFun;
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
					hitPoint.rgb,
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
		float epsInitSharpness = (maxSharpness - minSharpness) / (float)(numInitSharpness);
		float bestMSE = std::numeric_limits<float>::max();
		for (int testInitSharpness = 0; testInitSharpness < numInitSharpness; ++testInitSharpness)
		{
			float testSharpness = ((float)testInitSharpness + 0.5f) * epsInitSharpness;
			float testIntensity;
			float testSpecularIntensity;
			float testMSE = Evaluate_Intensity_SpecularIntensity_MSE(samples, testIntensity, testSharpness, testSpecularIntensity);

			if (testMSE < bestMSE)
			{
				outPoint.intensity = testIntensity;
				outPoint.sharpness = testSharpness;
				outPoint.specularIntensity = testSpecularIntensity;

				bestMSE = testMSE;
			}
		}

		/*const int numInitSharpness = 32;
		const int numInitSpecularIntensity = 32;

		float epsInitSharpness = (maxSharpness - minSharpness) / (float)(numInitSharpness);
		float epsInitSpecularIntensity = 1.0 / (float)(numInitSpecularIntensity);

		float bestMSE = std::numeric_limits<float>::max();
		for (int testInitSharpness = 0; testInitSharpness < numInitSharpness; ++testInitSharpness)
		{
			for (int testInitSpecularIntensity = 0; testInitSpecularIntensity < numInitSpecularIntensity; ++testInitSpecularIntensity)
			{
				float testSharpness = ((float)testInitSharpness + 0.5f) * epsInitSharpness;
				float testSpecularIntensity = ((float)testInitSpecularIntensity + 0.5f) * epsInitSpecularIntensity;
				float testMSE = Evaluate_MSE(samples, center.intensity, testSharpness, testSpecularIntensity);

				if (testMSE < bestMSE)
				{
					outPoint.sharpness = testSharpness;
					outPoint.intensity = center.intensity;
					outPoint.specularIntensity = testSpecularIntensity;

					bestMSE = testMSE;
				}
			}
		}*/

		Eigen::VectorXd lowerBound(3);
		Eigen::VectorXd upperBound(3);
		lowerBound << 0.0, minSharpness, 0.0;
		upperBound << 512.0, maxSharpness, 512.0;
		LBFGSB solver(lowerBound, upperBound);
		
		Eigen::VectorXd optX(3);
		optX << outPoint.intensity, outPoint.sharpness, outPoint.specularIntensity;

		solver.Solve(optX, SG_Distribution_ObjValue, SG_Distributio_ObjGradient, (void*)(&samples));

		outPoint.intensity = optX(0);
		outPoint.sharpness = optX(1);
		outPoint.specularIntensity = optX(2);

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