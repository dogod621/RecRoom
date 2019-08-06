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
		double diffuseRatio = x(2);

		double r = 0.0;
		for (std::vector<NDFSample>::iterator it = samples.begin(); it != samples.end(); ++it)
		{
			double diff = it->intensity - (intensity * (diffuseRatio + (1.0 - diffuseRatio) * std::exp(sharpness * (it->tanDir.z() - 1.0))));
			r += it->weight * diff * diff;
		}
		return r;
	}

	void SG_Distributio_ObjGradient(const Eigen::VectorXd& x, Eigen::VectorXd& g, void* data)
	{
		std::vector<NDFSample>& samples = *((std::vector<NDFSample>*)(data));

		double intensity = x(0);
		double sharpness = x(1);
		double diffuseRatio = x(2);

		g = Eigen::VectorXd(3);
		g(0) = 0.0;
		g(1) = 0.0;
		g(2) = 0.0;
		for (std::vector<NDFSample>::iterator it = samples.begin(); it != samples.end(); ++it)
		{
			double nFun = std::exp(sharpness * (it->tanDir.z() - 1.0));
			double dFun = diffuseRatio + (1.0 - diffuseRatio) * nFun;
			double diff = it->intensity - (intensity * dFun);
			double temp = -2.0 * it->weight * diff;
			g(0) += temp * dFun;
			g(1) += temp * (intensity * (1.0 - diffuseRatio) * nFun * (it->tanDir.z() - 1.0));
			g(2) += temp * (intensity * (1.0 - nFun));
		}
	}

	template<class InPointType, class OutPointType>
	inline bool EstimatorPcNDF<InPointType, OutPointType>::ComputeAttribute(
		const Pc<InPointType>& cloud,
		const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const
	{
		std::vector<NDFSample> samples;
		samples.reserve(scanDataSet.size());
		float meanIntensity = 0.0;
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
				float dotLN = hitNormal.dot(it->laser.incidentDirection);
				float weight = DistInterWeight(searchRadius, it->distance2Center, distInterParm) * AngleInterWeight(hitNormal, it->laser.incidentDirection, angleInterParm);
				float intensity = it->laser.intensity / (it->laser.beamFalloff * dotLN);
				//float intensity = it->laser.intensity / it->laser.beamFalloff;
				meanIntensity += weight * intensity;
				sumWeight += weight;
				samples.push_back(NDFSample(
					Eigen::Vector3f(
						hitTangent.dot(hafway),
						hitBitangent.dot(hafway),
						hitNormal.dot(hafway)),
					intensity, weight));
				temp.push_back(hitPoint.serialNumber);
			}
		}
		meanIntensity /= sumWeight;
		for (std::vector<NDFSample>::iterator it = samples.begin(); it != samples.end(); ++it)
		{
			it->weight /= sumWeight;
		}
		std::sort(temp.begin(), temp.end());
		if (std::distance(temp.begin(), std::unique(temp.begin(), temp.end())) < minRequireNumData)
			return false;

		std::vector<float> ndfValues(samples.size());

		//
		/*
		const int numDepth = 3;
		const int numTest = 64;

		//
		outPoint.sharpness = (maxSharpness - minSharpness) * 0.5f; // start center
		float bestMSE;
		EvaluateMSE(samples, outPoint.sharpness, meanIntensity,
			ndfValues, outPoint.intensity, outPoint.diffuseRatio, bestMSE);

		// 
		float testSharpness;
		float testIntensity;
		float testDiffuseRatio;
		float testMSE;
		float stopEps = 1e-6;
		int extNumGap = numTest / 2;
		for (int d = 1; d <= numDepth; ++d)
		{
			float eps = (maxSharpness - minSharpness) * std::pow(1.0f / (float)(numTest), (float)(d));
			for (int i = -extNumGap; i <= extNumGap; ++i)
			{
				if (i != 0)
				{
					testSharpness = outPoint.sharpness + float(i) * eps;
					EvaluateMSE(samples, testSharpness, meanIntensity, 
						ndfValues, testIntensity, testDiffuseRatio, testMSE);

					if (testMSE < bestMSE)
					{
						outPoint.sharpness = testSharpness;
						outPoint.intensity = testIntensity;
						outPoint.diffuseRatio = testDiffuseRatio;

						if (((bestMSE - testMSE) / bestMSE) < stopEps)
							return true;

						bestMSE = testMSE;
					}
				}
			}
		}*/

		const int numInitSharpness = 32;
		const int numInitDiffuseRatio = 32;

		float epsInitSharpness = (maxSharpness - minSharpness) / (float)(numInitSharpness);
		float epsInitDiffuseRatio = 1.0 / (float)(numInitDiffuseRatio);

		float bestMSE = std::numeric_limits<float>::max();
		for (int testInitSharpness = 0; testInitSharpness < numInitSharpness; ++testInitSharpness)
		{
			for (int testInitDiffuseRatio = 0; testInitDiffuseRatio < numInitDiffuseRatio; ++testInitDiffuseRatio)
			{
				float testSharpness = ((float)testInitSharpness + 0.5f) * epsInitSharpness;
				float testDiffuseRatio = ((float)testInitDiffuseRatio + 0.5f) * epsInitDiffuseRatio;
				float testIntensity;
				float testMSE;
				EvaluateMSE(samples, testSharpness, testDiffuseRatio, meanIntensity,
					ndfValues, testIntensity, testMSE);

				if (testMSE < bestMSE)
				{
					outPoint.sharpness = testSharpness;
					outPoint.intensity = testIntensity;
					outPoint.diffuseRatio = testDiffuseRatio;

					bestMSE = testMSE;
				}
			}
		}

		/*Eigen::VectorXd lowerBound(3);
		Eigen::VectorXd upperBound(3);
		lowerBound << 0.0, minSharpness, 0.0;
		upperBound << 512.0, maxSharpness, 1.0;
		LBFGSB solver(lowerBound, upperBound);
		
		Eigen::VectorXd optX(3);
		optX << outPoint.intensity, outPoint.sharpness, outPoint.diffuseRatio;

		solver.Solve(optX, SG_Distribution_ObjValue, SG_Distributio_ObjGradient, (void*)(&samples));
		outPoint.intensity = optX(0);
		outPoint.sharpness = optX(1);
		outPoint.diffuseRatio = optX(2);

		return OutPointValid(outPoint);*/
	}
}