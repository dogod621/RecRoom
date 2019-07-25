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
		//Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero();
		float accu[9] = {
			0.0f, 0.0f, 0.0f, 
			0.0f, 0.0f, 0.0f, 
			0.0f, 0.0f, 0.0f };

		float sumWeight = 0.0f;
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

		accu[0] /= sumWeight;
		accu[1] /= sumWeight;
		accu[2] /= sumWeight;
		accu[3] /= sumWeight;
		accu[4] /= sumWeight;
		accu[5] /= sumWeight;
		accu[6] /= sumWeight;
		accu[7] /= sumWeight;
		accu[8] /= sumWeight;

		//Eigen::Vector4f centroid;
		//centroid[0] = accu[6]; centroid[1] = accu[7]; centroid[2] = accu[8]; centroid[3] = 1;
		EIGEN_ALIGN16 Eigen::Matrix3f covarianceMatrix;
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
		float nx, ny, nz, cv;
		pcl::solvePlaneParameters(covarianceMatrix, nx, ny, nz, cv);

		//
		float norm = nx * nx + ny * ny + nz * nz;
		if (norm > 0.1f)
		{
			float invNorm = 1.0f / std::sqrt(norm);

			// Flip
			float score = 0.0f;
			for (std::vector<ScanData>::const_iterator it = scanDataSet.begin(); it != scanDataSet.end(); ++it)
				score += it->laser.incidentDirection.x() * nx + it->laser.incidentDirection.y() * ny + it->laser.incidentDirection.z() * nz;

			if (score < 0.0f)
				invNorm *= -1.0f;
			
			outPoint.normal_x = nx * invNorm;
			outPoint.normal_y = ny * invNorm;
			outPoint.normal_z = nz * invNorm;
			outPoint.curvature = cv;
			return true;
		}
		else
		{
			PRINT_WARNING("Generate normal failed, ignore");
			return false;
		}
	}
}