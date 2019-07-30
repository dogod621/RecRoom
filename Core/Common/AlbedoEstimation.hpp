#pragma once

#include "AlbedoEstimation.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	inline bool AlbedoEstimation<InPointType, OutPointType>::ComputeAttribute(
		const Pc<InPointType>& cloud, 
		const InPointType& center, const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const
	{
		Eigen::MatrixXf A;
		Eigen::MatrixXf B;

		A = Eigen::MatrixXf(scanDataSet.size() * 3, 3);
		B = Eigen::MatrixXf(scanDataSet.size() * 3, 1);
		int shifter = 0;
		float r = 0.0;
		float g = 0.0;
		float b = 0.0;
		float sumWeight = 0;
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

			float dotNN = hitNormal.dot(it->laser.incidentDirection);
			float weight = std::pow(((float)search_radius_ - it->distance2Center) / (float)search_radius_, distInterParm) * std::pow(dotNN, angleInterParm);
			sumWeight += weight;

			A(shifter, 0) = weight * it->laser.incidentDirection.x();
			A(shifter, 1) = weight * it->laser.incidentDirection.y();
			A(shifter, 2) = weight * it->laser.incidentDirection.z();
			B(shifter, 0) = weight * (it->laser.intensity / it->laser.beamFalloff);

			A(shifter + 1, 0) = weight * hitTangent.x();
			A(shifter + 1, 1) = weight * hitTangent.y();
			A(shifter + 1, 2) = weight * hitTangent.z();
			B(shifter + 1, 0) = 0.0f;

			A(shifter + 2, 0) = weight * hitBitangent.x();
			A(shifter + 2, 1) = weight * hitBitangent.y();
			A(shifter + 2, 2) = weight * hitBitangent.z();
			B(shifter + 2, 0) = 0.0f;

			r += weight * hitPoint.r;
			g += weight * hitPoint.g;
			b += weight * hitPoint.b;

			shifter += 3;
		}

		Eigen::MatrixXf X;
		switch (linearSolver)
		{
		case LinearSolver::EIGEN_QR:
		{
			X = A.colPivHouseholderQr().solve(B);
		}
		break;
		case LinearSolver::EIGEN_SVD:
		{
			X = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
		}
		break;
		case LinearSolver::EIGEN_NE:
		{
			Eigen::MatrixXf localAT = A.transpose();
			X = (localAT * A).ldlt().solve(localAT * B);
		}
		break;
		default:
		{
			THROW_EXCEPTION("LinearSolver is not supported.");
		}
		break;
		}

		//
		Eigen::Vector3f xVec(X(0, 0), X(1, 0), X(2, 0));
		if (std::isfinite(xVec.x()) && std::isfinite(xVec.y()) && std::isfinite(xVec.z()))
		{
			float xVecNorm = xVec.norm();
			if (xVecNorm > std::numeric_limits<float>::epsilon())
			{
				outPoint.intensity = xVecNorm;
				xVec /= xVecNorm;
				outPoint.normal_x = xVec.x();
				outPoint.normal_y = xVec.y();
				outPoint.normal_z = xVec.z();
				outPoint.r = std::max(std::min((r / sumWeight), 255.0f), 0.0f);
				outPoint.g = std::max(std::min((g / sumWeight), 255.0f), 0.0f);
				outPoint.b = std::max(std::min((b / sumWeight), 255.0f), 0.0f);
				
				return true;
			}
			else
			{
				PRINT_WARNING("LinearSolver solve zero norm");
				return false;
			}
		}
		else
		{
			PRINT_WARNING("LinearSolver solve non finite value");
			return false;
		}
		return true;
	}
}