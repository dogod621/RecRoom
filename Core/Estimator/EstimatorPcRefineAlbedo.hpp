#pragma once

#include "Common/LBFGSB.h"

#include "EstimatorPcRefineAlbedo.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	inline bool EstimatorPcRefineAlbedo<InPointType, OutPointType>::ComputeAttribute(
		const Pc<InPointType>& cloud, const InPointType& center,
		const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const
	{
		std::vector<NDFSample> samples;
		samples.reserve(scanDataSet.size());
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
			}
		}
		for (std::vector<NDFSample>::iterator it = samples.begin(); it != samples.end(); ++it)
		{
			it->weight /= sumWeight;
		}

		float temp = 0.0f;
		float meanDiffuseValues = 0.0f;
		for (std::vector<NDFSample>::const_iterator it = samples.cbegin(); it != samples.cend(); ++it)
		{
			float diffuseValue = it->tanDir.z() * DiffuseDistribution(it->tanDir);
			float specularValue = it->tanDir.z() * SpecularDistribution(it->tanDir, it->initSpecularSharpness);
			meanDiffuseValues += it->weight * diffuseValue;
			temp += it->weight * std::max(it->intensity - it->initSpecularAlbedo * specularValue, 0.0f);
		}
		outPoint.diffuseAlbedo = temp / meanDiffuseValues;

		return true;
	}
}