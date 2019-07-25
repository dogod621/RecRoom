#pragma once

#include "NDFEstimation.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	inline bool NDFEstimation<InPointType, OutPointType>::ComputeAttribute(
		const Pc<InPointType>& cloud,
		const InPointType& center, const std::vector<ScanData>& scanDataSet, OutPointType& outPoint) const
	{
		Eigen::Vector3d macroNormal(center.normal_x, center.normal_y, center.normal_z);

		Eigen::MatrixXf A;
		Eigen::MatrixXf B;

		A = Eigen::MatrixXf(scanDataSet.size() * 3, 3);
		B = Eigen::MatrixXf(scanDataSet.size() * 3, 1);

		std::size_t shifter = 0;
		for (std::vector<ScanData>::const_iterator it = scanDataSet.begin(); it != scanDataSet.end(); ++it)
		{
			const InPointType& hitPoint = cloud[it->index];
			Eigen::Vector3d hitNormal(hitPoint.normal_x, hitPoint.normal_y, hitPoint.normal_z);
			Eigen::Vector3d hitTangent;
			Eigen::Vector3d hitBitangent;
			if (!Common::GenFrame(hitNormal, hitTangent, hitBitangent))
			{
				PRINT_WARNING("GenFrame failed, ignore");
				return false;
			}

			double dotNN = hitNormal.dot(macroNormal);
			double weight = std::pow((search_radius_ - it->distance2Center) / search_radius_, distInterParm) * std::pow(dotNN, angleInterParm);

			A(shifter, 0) = weight * it->laser.incidentDirection.x();
			A(shifter, 1) = weight * it->laser.incidentDirection.y();
			A(shifter, 2) = weight * it->laser.incidentDirection.z();
			B(shifter, 0) = weight * (it->laser.intensity / it->laser.beamFalloff);

			A(shifter + 1, 0) = weight * hitTangent.x();
			A(shifter + 1, 1) = weight * hitTangent.y();
			A(shifter + 1, 2) = weight * hitTangent.z();
			B(shifter + 1, 0) = 0.0;

			A(shifter + 2, 0) = weight * hitBitangent.x();
			A(shifter + 2, 1) = weight * hitBitangent.y();
			A(shifter + 2, 2) = weight * hitBitangent.z();
			B(shifter + 2, 0) = 0.0;

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
		Eigen::Vector3d xVec(X(0, 0), X(1, 0), X(2, 0));
		if (std::isfinite(xVec.x()) && std::isfinite(xVec.y()) && std::isfinite(xVec.z()))
		{
			double xVecNorm = xVec.norm();
			if (xVecNorm > Common::eps)
			{
				outPoint.intensity = xVecNorm;
				xVec /= xVecNorm;
				outPoint.normal_x = xVec.x();
				outPoint.normal_y = xVec.y();
				outPoint.normal_z = xVec.z();
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