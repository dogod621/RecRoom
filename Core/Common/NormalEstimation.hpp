#pragma once

#pragma once

#include "NormalEstimation.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	inline bool NormalEstimation<InPointType, OutPointType>::ComputeAttribute(
		const Pc<InPointType>& cloud,
		const InPointType& center, const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const
	{
		Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero();
		EIGEN_ALIGN16 Eigen::Matrix3f covarianceMatrix;
		Eigen::Vector4f centroid;

		float sumWeight = 0.0;
		for (std::vector<ScanData>::const_iterator it = scanDataSet.begin(); it != scanDataSet.end(); ++it)
		{
			const InPointType& hitPoint = cloud[it->index];
			float weight = std::pow((search_radius_ - it->distance2Center) / search_radius_, distInterParm);
			sumWeight += weight;

			accu[0] += weight * hitPoint.x * hitPoint.x;
			accu[1] += weight * hitPoint.x * hitPoint.y;
			accu[2] += weight * hitPoint.x * hitPoint.z;
			accu[3] += weight * hitPoint.y * hitPoint.y;
			accu[4] += weight * hitPoint.y * hitPoint.z;
			accu[5] += weight * hitPoint.z * hitPoint.z;
			accu[6] += weight * hitPoint.x;
			accu[7] += weight * hitPoint.y;
			accu[8] += weight * hitPoint.z;
		}

		accu /= sumWeight;
		centroid[0] = accu[6]; centroid[1] = accu[7]; centroid[2] = accu[8]; centroid[3] = 1;
		covarianceMatrix.coeffRef(0) = accu[0] - accu[6] * accu[6];
		covarianceMatrix.coeffRef(1) = accu[1] - accu[6] * accu[7];
		covarianceMatrix.coeffRef(2) = accu[2] - accu[6] * accu[8];
		covarianceMatrix.coeffRef(4) = accu[3] - accu[7] * accu[7];
		covarianceMatrix.coeffRef(5) = accu[4] - accu[7] * accu[8];
		covarianceMatrix.coeffRef(8) = accu[5] - accu[8] * accu[8];
		covarianceMatrix.coeffRef(3) = covarianceMatrix.coeff(1);
		covarianceMatrix.coeffRef(6) = covarianceMatrix.coeff(2);
		covarianceMatrix.coeffRef(7) = covarianceMatrix.coeff(5);

		// Get the plane normal and surface curvature
		pcl::solvePlaneParameters(covarianceMatrix, outPoint.normal_x, outPoint.normal_y, outPoint.normal_z, outPoint.curvature);

		//
		float norm = std::sqrt(outPoint.normal_x*outPoint.normal_x + outPoint.normal_y*outPoint.normal_y + outPoint.normal_z*outPoint.normal_z);
		if (norm > 0.1)
		{
			// Flip
			double pScore = 0;
			double nScore = 0;
			for (std::vector<ScanData>::const_iterator it = scanDataSet.begin(); it != scanDataSet.end(); ++it)
			{
				const InPointType& hitPoint = cloud[it->index];
				Eigen::Vector3d pNormal(hitPoint.normal_x, hitPoint.normal_y, hitPoint.normal_z);
				Eigen::Vector3d nNormal(-hitPoint.normal_x, -hitPoint.normal_y, -hitPoint.normal_z);

				pScore += pNormal.dot(it->laser.incidentDirection);
				nScore += nNormal.dot(it->laser.incidentDirection);
			}
			if (nScore > pScore)
				norm = -norm;
			
			outPoint.normal_x /= norm;
			outPoint.normal_y /= norm;
			outPoint.normal_z /= norm;
			return true;
		}
		return false;
	}
}