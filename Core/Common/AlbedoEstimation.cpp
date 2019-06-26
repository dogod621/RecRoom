#include "AlbedoEstimation.h"

namespace RecRoom
{
	inline bool AlbedoEstimation::CollectScannLaserInfo(const pcl::PointCloud<PointMED>& cloud, const std::size_t k, const std::vector<int>& indices, const std::vector<float>& distance, const PointMED& inPoint, std::vector<ScannLaser>& scannLaserSet)
	{
		scannLaserSet.clear();
		scannLaserSet.reserve(k);
		double radius = search_radius_;

#ifdef POINT_MED_WITH_NORMAL
#ifdef POINT_MED_WITH_LABEL
#ifdef POINT_MED_WITH_INTENSITY
		Eigen::Vector3d inNormal (inPoint.normal_x, inPoint.normal_y, inPoint.normal_z);

		if (!Common::IsUnitVector(inNormal))
		{
			PRINT_WARNING("inNormal is not valid!!?");
			return false;
		}

		//
		for (std::size_t idx = 0; idx < k; ++idx)
		{
			int px = indices[idx];
			double d = std::sqrt((double)distance[idx]);
			if (d > radius)
			{
				PRINT_WARNING("distance is larger then radius, ignore");
			}
			else
			{
				const PointMED& scanPoint = cloud[px];
				const ScanMeta& scanMeta = scanMetaSet[scanPoint.label];
				ScannLaser scannLaser;

				//
				scannLaser.hitNormal = Eigen::Vector3d(cloud[px].normal_x, cloud[px].normal_y, cloud[px].normal_z);
				if (!Common::IsUnitVector(scannLaser.hitNormal))
				{
					PRINT_WARNING("scannLaser.hitNormal is not valid, ignore");
				}
				else
				{
					double dotNN = scannLaser.hitNormal.dot(inNormal);
					if (dotNN > cutGrazing)
					{
						scannLaser.hitPosition = Eigen::Vector3d(scanPoint.x, scanPoint.y, scanPoint.z);
						switch (scanMeta.scanner)
						{
						case Scanner::BLK360:
						{
							scannLaser.incidentDirection = scanMeta.position - scannLaser.hitPosition;
							scannLaser.hitDistance = scannLaser.incidentDirection.norm();
							scannLaser.incidentDirection /= scannLaser.hitDistance;
							if (scannLaser.incidentDirection.dot(inNormal) < 0)
								scannLaser.incidentDirection *= -1.0;
							scannLaser.reflectedDirection = scannLaser.incidentDirection; // BLK360 

							// Ref - BLK 360 Spec - laser wavelength & Beam divergence : https://lasers.leica-geosystems.com/global/sites/lasers.leica-geosystems.com.global/files/leica_media/product_documents/blk/853811_leica_blk360_um_v2.0.0_en.pdf
							// Ref - Gaussian beam : https://en.wikipedia.org/wiki/Gaussian_beam
							// Ref - Beam divergence to Beam waist(w0) : http://www2.nsysu.edu.tw/optics/laser/angle.htm
							double temp = scannLaser.hitDistance / 26.2854504782;
							scannLaser.beamFalloff = 1.0f / (1 + temp * temp);
							if ((scannLaser.beamFalloff > cutFalloff))
							{
								if (Common::GenFrame(scannLaser.hitNormal, scannLaser.hitTangent, scannLaser.hitBitangent))
								{
									scannLaser.weight = std::pow((radius - d) / radius, distInterParm) * std::pow(dotNN, angleInterParm);
									scannLaser.intensity = (double)scanPoint.intensity;
									scannLaserSet.push_back(scannLaser);
								}
							}
						}
						break;

						default:
							PRINT_WARNING("Scan data Scanner type is not support, ignore");
							break;
						}
					}
				}
			}
		}
#endif
#endif
#endif
		return scannLaserSet.size() > 0;
	}

	inline bool AlbedoEstimation::ComputePointAlbedo(const std::vector<ScannLaser>& scannLaserSet, const PointMED& inPoint, PointMED& outPoint)
	{
#ifdef POINT_MED_WITH_NORMAL
#ifdef POINT_MED_WITH_INTENSITY
		Eigen::MatrixXf A;
		Eigen::MatrixXf B;

		A = Eigen::MatrixXf(scannLaserSet.size() * 3, 3);
		B = Eigen::MatrixXf(scannLaserSet.size() * 3, 1);

		std::size_t shifter = 0;
		for (std::vector<ScannLaser>::const_iterator it = scannLaserSet.begin(); it != scannLaserSet.end(); ++it)
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
#endif
#endif
		return true;
	}

	void AlbedoEstimation::computeFeature(PointCloudOut &output)
	{
		if (!(search_radius_ > 0.0))
			THROW_EXCEPTION("search_radius_ is not set");
#ifdef POINT_MED_WITH_NORMAL
#ifdef POINT_MED_WITH_LABEL
#ifdef POINT_MED_WITH_INTENSITY
		std::vector<int> nn_indices(k_);
		std::vector<float> nn_dists(k_);

		output.is_dense = true;
		if (input_->is_dense)
		{
			for (std::size_t idx = 0; idx < indices_->size(); ++idx)
			{
				const PointMED& inPoint = (*input_)[(*indices_)[idx]];
				PointMED& outPoint = output.points[idx];

				std::vector<ScannLaser> scannLaser;
				if (CollectScannLaserInfo(*surface_, 
					this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists),
					nn_indices, nn_dists,
					inPoint, scannLaser))
				{
					if (!ComputePointAlbedo(scannLaser, inPoint, outPoint))
					{
						PRINT_WARNING("ComputePointAlbedo failed");
						outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PRINT_WARNING("CollectScannLaserInfo failed");
					outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
		}
		else
		{
			for (std::size_t idx = 0; idx < indices_->size(); ++idx)
			{
				const PointMED& inPoint = (*input_)[(*indices_)[idx]];
				PointMED& outPoint = output.points[idx];
				if (pcl::isFinite(inPoint))
				{
					std::vector<ScannLaser> scannLaser;
					if (CollectScannLaserInfo(*surface_, 
						this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists),
						nn_indices, nn_dists,
						inPoint, scannLaser))
					{
						if (!ComputePointAlbedo(scannLaser, inPoint, outPoint))
						{
							PRINT_WARNING("ComputePointAlbedo failed");
							outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
							output.is_dense = false;
						}
					}
					else
					{
						PRINT_WARNING("CollectScannLaserInfo failed");
						outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PRINT_WARNING("Input point contain non finite value");
					outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
		}
#endif
#endif
#endif
	}

	void AlbedoEstimationOMP::SetNumberOfThreads(unsigned int nr_threads)
	{
		if (nr_threads == 0)
#ifdef _OPENMP
			threads_ = omp_get_num_procs();
#else
			threads_ = 1;
#endif
		else
			threads_ = nr_threads;
	}

	void AlbedoEstimationOMP::computeFeature(PointCloudOut &output)
	{
		if (!(search_radius_ > 0.0))
			THROW_EXCEPTION("search_radius_ is not set");
#ifdef POINT_MED_WITH_NORMAL
#ifdef POINT_MED_WITH_LABEL
#ifdef POINT_MED_WITH_INTENSITY
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

				std::vector<ScannLaser> scannLaser;
				if (CollectScannLaserInfo(*surface_, 
					this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists),
					nn_indices, nn_dists,
					inPoint, scannLaser))
				{
					if (!ComputePointAlbedo(scannLaser, inPoint, outPoint))
					{
						PRINT_WARNING("ComputePointAlbedo failed");
						outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PRINT_WARNING("CollectScannLaserInfo failed");
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
					std::vector<ScannLaser> scannLaser;
					if (CollectScannLaserInfo(*surface_, 
						this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists),
						nn_indices, nn_dists,
						inPoint, scannLaser))
					{
						if (!ComputePointAlbedo(scannLaser, inPoint, outPoint))
						{
							PRINT_WARNING("ComputePointAlbedo failed");
							outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
							output.is_dense = false;
						}
					}
					else
					{
						PRINT_WARNING("CollectScannLaserInfo failed");
						outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PRINT_WARNING("Input point contain non finite value");
					outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
		}
#endif
#endif
#endif
	}
}