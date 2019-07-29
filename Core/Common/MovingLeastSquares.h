#pragma once

 // PCL includes
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>

#include <pcl/search/pcl_search.h>
#include <pcl/common/common.h>

#include <pcl/surface/boost.h>
#include <pcl/surface/eigen.h>
#include <pcl/surface/processing.h>

#include <map>
#include <boost/function.hpp>

#include "Common.h"
#include "Point.h"
#include "Random.h"

namespace RecRoom
{
	enum MLSProjectionMethod
	{
		MLSProjectionMethod_NONE,	// brief Project to the mls plane.
		SIMPLE,						// brief Project along the mls plane normal to the polynomial surface.
		ORTHOGONAL					// brief Project to the closest point on the polynonomial surface.
	};

	enum MLSUpsamplingMethod
	{
		MLSUpsamplingMethod_NONE,	// brief No upsampling will be done, only the input points will be projected to their own MLS surfaces.
		DISTINCT_CLOUD,				// brief Project the points of the distinct cloud to the MLS surface. 
		SAMPLE_LOCAL_PLANE,			// brief The local plane of each input point will be sampled in a circular fashion
									// using the upsamplingRadius and the upsamplingStep parameters. 
		RANDOM_UNIFORM_DENSITY,		// brief The local plane of each input point will be sampled using an uniform random
									// distribution such that the density of points is constant throughout the
									// cloud - given by the desired_num_points_in_radius_ parameter. 
		VOXEL_GRID_DILATION			// brief The input cloud will be inserted into a voxel grid with voxels of
									// size voxelSize; this voxel grid will be dilated dilationIterationNum
									// times and the resulting points will be projected to the MLS surface
									// of the closest point in the input cloud; the result is a point cloud
									// with filled holes and a constant point density. 
	};

	// brief Data structure used to store the MLS polynomial partial derivatives
	struct PolynomialPartialDerivative
	{
		double z;    // brief The z component of the polynomial evaluated at z(u, v).
		double z_u;  // brief The partial derivative dz/du.
		double z_v;  // brief The partial derivative dz/dv.
		double z_uu; // brief The partial derivative d^2z/du^2.
		double z_vv; // brief The partial derivative d^2z/dv^2.
		double z_uv; // brief The partial derivative d^2z/dudv.
	};

	// brief Data structure used to store the MLS projection results
	struct MLSProjectionResults
	{
		MLSProjectionResults() : u(0), v(0) {}

		double u;               // brief The u-coordinate of the projected point in local MLS frame.
		double v;               // brief The u-coordinate of the projected point in local MLS frame.
		Eigen::Vector3d point;  // brief The projected point.
		Eigen::Vector3d normal; // brief The projected point's normal.
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	// brief Data structure used to store the results of the MLS fitting
	struct MLSResult
	{
		MLSResult() : numNeighbors(0), curvature(0.0f), order(0), valid(false) {}

		MLSResult(const Eigen::Vector3d& queryPoint, const Eigen::Vector3d& mean, const Eigen::Vector3d& planeNormal,
			const Eigen::Vector3d& uAxis, const Eigen::Vector3d& vAxis, const Eigen::VectorXd& cAxis,
			const int numNeighbors, const float curvature, const int order) :
			queryPoint(queryPoint), mean(mean), planeNormal(planeNormal),
			uAxis(uAxis), vAxis(vAxis), cAxis(cAxis), numNeighbors(numNeighbors),
			curvature(curvature), order(order), valid(true) {}

		// brief Given a point calculate it's 3D location in the MLS frame.
		inline void getMLSCoordinates(const Eigen::Vector3d &pt, double &u, double &v, double &w) const;

		// brief Given a point calculate it's 2D location in the MLS frame.
		inline void getMLSCoordinates(const Eigen::Vector3d &pt, double &u, double &v) const;

		// brief Calculate the polynomial
		inline double getPolynomialValue(const double u, const double v) const;

		// brief Calculate the polynomial's first and second partial derivatives.
		inline PolynomialPartialDerivative getPolynomialPartialDerivative(const double u, const double v) const;

		// brief Calculate the principle curvatures using the polynomial surface.
		inline Eigen::Vector2f calculatePrincipleCurvatures(const double u, const double v) const;

		// brief Project a point orthogonal to the polynomial surface.
		inline MLSProjectionResults projectPointOrthogonalToPolynomialSurface(const double u, const double v, const double w) const;

		// brief Project a point onto the MLS plane.
		inline MLSProjectionResults projectPointToMLSPlane(const double u, const double v) const;

		// brief Project a point along the MLS plane normal to the polynomial surface.
		inline MLSProjectionResults projectPointSimpleToPolynomialSurface(const double u, const double v) const;

		// brief Project a point using the specified method.
		inline MLSProjectionResults projectPoint(const Eigen::Vector3d &pt, MLSProjectionMethod method, int requiredNeighbors = 0) const;

		// brief Project the query point used to generate the mls surface about using the specified method.
		inline MLSProjectionResults projectQueryPoint(MLSProjectionMethod method, int requiredNeighbors = 0) const;

		// brief Smooth a given point and its neighborghood using Moving Least Squares.
		template <class PointT>
		void computeMLSSurface(const pcl::PointCloud<PointT> &cloud, int index, const std::vector<int> &nnIndices,
			double searchRadius, int polynomialOrder = 2, boost::function<double(const double)> WeightFunc = 0);

		Eigen::Vector3d queryPoint;		// brief The query point about which the mls surface was generated 
		Eigen::Vector3d mean;			// brief The mean point of all the neighbors. 
		Eigen::Vector3d planeNormal;	// brief The normal of the local plane of the query point. 
		Eigen::Vector3d uAxis;			// brief The axis corresponding to the u-coordinates of the local plane of the query point. 
		Eigen::Vector3d vAxis;			// brief The axis corresponding to the v-coordinates of the local plane of the query point. 
		Eigen::VectorXd cAxis;			// brief The polynomial coefficients Example: z = cAxis[0] + cAxis[1]*v + cAxis[2]*v^2 + cAxis[3]*u + cAxis[4]*u*v + cAxis[5]*u^2 
		int numNeighbors;				// brief The number of neighbors used to create the mls surface. 
		float curvature;				// brief The curvature at the query point. 
		int order;						// brief The order of the polynomial. If order > 1 then use polynomial fit 
		bool valid;						// brief If True, the mls results data is valid, otherwise False. 
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		// brief The default weight function used when fitting a polynomial surface
		inline double computeMLSWeight(const double sqrDist, const double sqrRadius) { return (exp(-sqrDist / sqrRadius)); }
	};

	template <class InPointN, class OutPointN>
	class MovingLeastSquares : public pcl::CloudSurfaceProcessing<InPointN, OutPointN>, public ThreadAble
	{
	public:
		typedef boost::shared_ptr<MovingLeastSquares<InPointN, OutPointN> > Ptr;
		typedef boost::shared_ptr<const MovingLeastSquares<InPointN, OutPointN> > ConstPtr;

		using pcl::PCLBase<InPointN>::input_;
		using pcl::PCLBase<InPointN>::indices_;
		using pcl::PCLBase<InPointN>::fake_indices_;
		using pcl::PCLBase<InPointN>::initCompute;
		using pcl::PCLBase<InPointN>::deinitCompute;

		MovingLeastSquares(
			double searchRadius,
			int order = 2,
			MLSProjectionMethod projectionMethod = MLSProjectionMethod::SIMPLE,
			MLSUpsamplingMethod upsampleMethod = MLSUpsamplingMethod::MLSUpsamplingMethod_NONE,
			bool computeNormals = true ) 
			: 
			pcl::CloudSurfaceProcessing<InPointN, OutPointN>(),
			ThreadAble(),
			tree(),

			// Parms
			searchRadius(searchRadius),
			order(order),
			projectionMethod(projectionMethod),
			upsampleMethod(upsampleMethod),
			computeNormals(computeNormals),

			// Options
			upsamplingRadius(0.0),
			upsamplingStep(0.0),
			desired_num_points_in_radius_(0),
			cacheMLSResults(true),
			mlsResults(),
			voxelSize(1.0),
			dilationIterationNum(0),

			// Vars
			distinctCloud_(),
			correspondingInputIndices(),
			rng_alg_(),
			rng_uniform_distribution_()
		{
			numCoeff = (order + 1) * (order + 2) / 2;
			sqrGaussParam = searchRadius * searchRadius;

			if (searchRadius <= 0 || sqrGaussParam <= 0)
			{
				THROW_EXCEPTION("Invalid search radius or Gaussian parameter");
				return;
			}
		};

		void process(Pc<OutPointN>& output);

		// brief Get the set of indices with each point in output having the corresponding point in input 
		inline PointIndicesPtr getCorrespondingIndices() const { return (correspondingInputIndices); }

	public:
		inline void setDistinctCloud(CONST_PTR(Pc<InPointN>) distinct_cloud) { distinctCloud_ = distinct_cloud; }
		inline CONST_PTR(Pc<InPointN>) getDistinctCloud() const { return (distinctCloud_); }

		inline void setUpsamplingRadius(double radius) { upsamplingRadius = radius; }
		inline double getUpsamplingRadius() const { return (upsamplingRadius); }

		inline void setUpsamplingStepSize(double step_size) { upsamplingStep = step_size; }
		inline double getUpsamplingStepSize() const { return (upsamplingStep); }

		inline void setPointDensity(int desired_num_points_in_radius) { desired_num_points_in_radius_ = desired_num_points_in_radius; }
		inline int getPointDensity() const { return (desired_num_points_in_radius_); }

		inline void setDilationVoxelSize(float voxel_size) { voxelSize = voxel_size; }
		inline float getDilationVoxelSize() const { return (voxelSize); }

		inline void setDilationIterations(int iterations) { dilationIterationNum = iterations; }
		inline int getDilationIterations() const { return (dilationIterationNum); }

		inline void setCacheMLSResults(bool cache_mls_results) { cacheMLSResults = cache_mls_results; }
		inline bool getCacheMLSResults() const { return (cacheMLSResults); }

		inline const std::vector<MLSResult>& getMLSResults() const { return (mlsResults); }

	protected:
		CONST_PTR(Pc<InPointN>) distinctCloud_; // brief The distinct point cloud that will be projected to the MLS surface.
		PTR(Acc<InPointN>) tree; // brief A pointer to the spatial search object.
		int order; // brief The order of the polynomial to be fit.
		int numCoeff; // brief Number of coefficients, to be computed from the requested order.
		double searchRadius; // brief The nearest neighbors search radius for each point.
		double sqrGaussParam; // brief Parameter for distance based weighting of neighbors (searchRadius * searchRadius works fine)
		bool computeNormals; // brief Parameter that specifies whether the normals should be computed for the input cloud or not
		MLSUpsamplingMethod upsampleMethod; // brief Parameter that specifies the upsampling method to be used 
		MLSProjectionMethod projectionMethod; // brief Parameter that specifies the projection method to be used.
		PointIndicesPtr correspondingInputIndices; // brief Collects for each point in output the corrseponding point in the input.

	protected:
		double upsamplingRadius; // brief Radius of the circle in the local point plane that will be sampled for SAMPLE_LOCAL_PLANE upsampling
		double upsamplingStep; // brief Step size for the local plane sampling for SAMPLE_LOCAL_PLANE upsampling
		int desired_num_points_in_radius_; // brief Parameter that specifies the desired number of points within the search radius for RANDOM_UNIFORM_DENSITY upsampling
		bool cacheMLSResults; // brief True if the mls results for the input cloud should be stored. note This is forced to true when using upsampling methods VOXEL_GRID_DILATION or DISTINCT_CLOUD.
		std::vector<MLSResult> mlsResults; // brief Stores the MLS result for each point in the input cloud for VOXEL_GRID_DILATION or DISTINCT_CLOUD upsampling
		float voxelSize; // brief Voxel size for the VOXEL_GRID_DILATION upsampling method 
		int dilationIterationNum; // brief Number of dilation steps for the VOXEL_GRID_DILATION upsampling method

	protected:
		void computeMLSPointNormal(int index, const std::vector<int> &nnIndices, Pc<OutPointN>& projected_points, pcl::PointIndices &corresponding_input_indices, MLSResult &mls_result) const;

		inline void addProjectedPointNormal(int index, const Eigen::Vector3d &point, const Eigen::Vector3d &normal, double curvature, 
			Pc<OutPointN>& projected_points, pcl::PointIndices &corresponding_input_indices) const
		{
			OutPointN aux;
			aux.x = static_cast<float> (point[0]);
			aux.y = static_cast<float> (point[1]);
			aux.z = static_cast<float> (point[2]);
			// Copy additional point information if available
			copyMissingFields(input_->points[index], aux);
			if (computeNormals)
			{
				aux.normal_x = static_cast<float> (normal[0]);
				aux.normal_y = static_cast<float> (normal[1]);
				aux.normal_z = static_cast<float> (normal[2]);
				aux.curvature = curvature;
			}
			projected_points.push_back(aux);
			corresponding_input_indices.indices.push_back(index);
		}
		
		inline void copyMissingFields(const InPointN& point_in, OutPointN& point_out) const
		{
			OutPointN temp = point_out;
			pcl::copyPoint(point_in, point_out);
			point_out.x = temp.x;
			point_out.y = temp.y;
			point_out.z = temp.z;
		}

		virtual void performProcessing(Pc<OutPointN>& output);
		void performUpsampling(Pc<OutPointN>& output);

	private:
		boost::mt19937 rng_alg_;
		boost::shared_ptr<boost::variate_generator<boost::mt19937&, boost::uniform_real<float> >> rng_uniform_distribution_;
		std::string getClassName() const { return ("MovingLeastSquares"); }
	};
}

#include "MovingLeastSquares.hpp"
