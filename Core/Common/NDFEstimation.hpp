#pragma once

#include "NDFEstimation.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	inline bool NDFEstimation<InPointType, OutPointType>::ComputeAttribute(
		const Pc<InPointType>& cloud,
		const InPointType& center, const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const
	{
		std::vector<NDFSample> samples;
		samples.reserve(scanDataSet.size());
		float meanIntensity = 0.0;
		float sumWeight = 0.0;
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
				float dotNN = hitNormal.dot(hafway);
				float weight = std::pow((search_radius_ - it->distance2Center) / search_radius_, distInterParm) * std::pow(dotNN, angleInterParm);
				float intensity = it->laser.intensity / it->laser.beamFalloff;
				meanIntensity += weight * intensity;
				sumWeight += weight;
				samples.push_back(NDFSample(
					Eigen::Vector3f(
						hitTangent.dot(hafway),
						hitBitangent.dot(hafway),
						hitNormal.dot(hafway)),
					intensity, weight));
			}
		}
		meanIntensity /= sumWeight;
		std::vector<float> ndfValues(samples.size());
		
		//
		const int numDepth = 5;
		const int numTest = 9; 

		//
		outPoint.sharpness = (maxSharpness - minSharpness) * 0.5f; // start center
		float bestMSE;
		float bestSNR;
		EvaluateMSE(samples, outPoint.sharpness, meanIntensity, sumWeight,
			ndfValues, outPoint.intensity, bestMSE);
		bestSNR = 10.0f * std::log10f(meanIntensity / bestMSE);
		if (bestSNR > threshSNR)
			return true;

		// 
		float testSharpness;
		float testIntensity;
		float testMSE;
		float stopEps = 0.001f;
		int extNumGap = numTest/2;
		for (int d = 1; d <= numDepth; ++d)
		{
			float eps = (maxSharpness - minSharpness) * std::pow(1.0f / (float)(numTest), (float)(d));
			for (int i = -extNumGap; i <= extNumGap; ++i)
			{
				if (i != 0)
				{
					testSharpness = outPoint.sharpness + float(i) * eps;
					EvaluateMSE(samples, testSharpness, meanIntensity, sumWeight,
						ndfValues, testIntensity, testMSE);

					if (testMSE < bestMSE)
					{
						outPoint.sharpness = testSharpness;
						outPoint.intensity = testIntensity;

						if(((bestMSE - testMSE) / bestMSE) < stopEps)
							return true;

						bestMSE = testMSE;
						bestSNR = 10.0f * std::log10f(meanIntensity / bestMSE);
						if (bestSNR > threshSNR)
							return true;
					}
				}
			}
		}
		return true;
	}
}