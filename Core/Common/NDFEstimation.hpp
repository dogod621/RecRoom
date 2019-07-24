#pragma once

#include "NDFEstimation.h"

namespace RecRoom
{
	template<class InPointIN, class OutPointINS>
	inline bool NDFEstimation<InPointIN, OutPointINS>::CollectScanLaserInfo(const Pc<InPointIN>& cloud, const std::size_t k, const std::vector<int>& indices, const std::vector<float>& distance, const InPointIN& inPoint, std::vector<ScanLaser>& scanLaserSet)
	{
		scanLaserSet.clear();
		scanLaserSet.reserve(k);

		Eigen::Vector3d inNormal(inPoint.normal_x, inPoint.normal_y, inPoint.normal_z);

		if (!Common::IsUnitVector(inNormal))
		{
			std::stringstream ss;
			ss << "inNormal is not valid: " << inNormal;
			PRINT_WARNING(ss.str());
			return false;
		}

		//
		for (std::size_t idx = 0; idx < k; ++idx)
		{
			int px = indices[idx];
			double d = std::sqrt((double)distance[idx]);
			if (d > search_radius_)
			{
				std::stringstream ss;
				ss << "distance is larger then search_radius_, ignore: " << d;
				PRINT_WARNING(ss.str());
			}
			else
			{
				ScanLaser scanLaser;
				if (scanner->ToScanLaser(cloud[px], inNormal, scanLaser))
				{
					double dotNN = scanLaser.hitNormal.dot(inNormal);
					if ((dotNN > cutGrazing) && (scanLaser.beamFalloff > cutFalloff))
					{
						scanLaser.weight = std::pow((search_radius_ - d) / search_radius_, distInterParm) * std::pow(dotNN, angleInterParm);
						scanLaserSet.push_back(scanLaser);
					}
				}
			}
		}

		return scanLaserSet.size() > 0;
	}

	template<class InPointIN, class OutPointINS>
	inline bool NDFEstimation<InPointIN, OutPointINS>::ComputePointAlbedo(const std::vector<ScanLaser>& scanLaserSet, const InPointIN& inPoint, OutPointINS& outPoint)
	{
		Eigen::MatrixXf A;
		Eigen::MatrixXf B;

		A = Eigen::MatrixXf(scanLaserSet.size() * 3, 3);
		B = Eigen::MatrixXf(scanLaserSet.size() * 3, 1);

		std::size_t shifter = 0;
		for (std::vector<ScanLaser>::const_iterator it = scanLaserSet.begin(); it != scanLaserSet.end(); ++it)
		{
			A(shifter, 0) = it->weight * it->incidentDirection.x();
			A(shifter, 1) = it->weight * it->incidentDirection.y();
			A(shifter, 2) = it->weight * it->incidentDirection.z();
			B(shifter, 0) = it->weight * (it->intensity / it->beamFalloff);

			A(shifter + 1, 0) = it->weight * it->hitTangent.x();
			A(shifter + 1, 1) = it->weight * it->hitTangent.y();
			A(shifter + 1, 2) = it->weight * it->hitTangent.z();
			B(shifter + 1, 0) = 0.0;

			A(shifter + 2, 0) = it->weight * it->hitBitangent.x();
			A(shifter + 2, 1) = it->weight * it->hitBitangent.y();
			A(shifter + 2, 2) = it->weight * it->hitBitangent.z();
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

	template<class InPointIN, class OutPointINS>
	void NDFEstimation<InPointIN, OutPointINS>::computeFeature(PointCloudOut &output)
	{
		if (!(search_radius_ > 0.0))
			THROW_EXCEPTION("search_radius_ is not set");

		std::vector<int> nn_indices(k_);
		std::vector<float> nn_dists(k_);

		output.is_dense = true;
		if (input_->is_dense)
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				const PointMED& inPoint = (*input_)[(*indices_)[idx]];
				PointMED& outPoint = output.points[idx];

				std::vector<ScanLaser> scanLaserSet;
				if (CollectScanLaserInfo(*surface_,
					searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists),
					nn_indices, nn_dists,
					inPoint, scanLaserSet))
				{
					if (!ComputePointAlbedo(scanLaserSet, inPoint, outPoint))
					{
						PRINT_WARNING("ComputePointAlbedo failed");
						outPoint.normal_x = inPoint.normal_x;
						outPoint.normal_y = inPoint.normal_y;
						outPoint.normal_z = inPoint.normal_z;
						outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PRINT_WARNING("CollectScanLaserInfo failed");
					outPoint.normal_x = inPoint.normal_x;
					outPoint.normal_y = inPoint.normal_y;
					outPoint.normal_z = inPoint.normal_z;
					outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
		}
		else
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				const PointMED& inPoint = (*input_)[(*indices_)[idx]];
				PointMED& outPoint = output.points[idx];
				if (pcl::isFinite(inPoint))
				{
					std::vector<ScanLaser> scanLaserSet;
					if (CollectScanLaserInfo(*surface_,
						searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists),
						nn_indices, nn_dists,
						inPoint, scanLaserSet))
					{
						if (!ComputePointAlbedo(scanLaserSet, inPoint, outPoint))
						{
							PRINT_WARNING("ComputePointAlbedo failed");
							outPoint.normal_x = inPoint.normal_x;
							outPoint.normal_y = inPoint.normal_y;
							outPoint.normal_z = inPoint.normal_z;
							outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
							output.is_dense = false;
						}
					}
					else
					{
						PRINT_WARNING("CollectScanLaserInfo failed");
						outPoint.normal_x = inPoint.normal_x;
						outPoint.normal_y = inPoint.normal_y;
						outPoint.normal_z = inPoint.normal_z;
						outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PRINT_WARNING("Input point contain non finite value");
					outPoint.normal_x = inPoint.normal_x;
					outPoint.normal_y = inPoint.normal_y;
					outPoint.normal_z = inPoint.normal_z;
					outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
		}
	}
}