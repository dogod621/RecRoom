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
									// using the upsampling_radius_ and the upsampling_step_ parameters. 
		RANDOM_UNIFORM_DENSITY,		// brief The local plane of each input point will be sampled using an uniform random
									// distribution such that the density of points is constant throughout the
									// cloud - given by the desired_num_points_in_radius_ parameter. 
		VOXEL_GRID_DILATION			// brief The input cloud will be inserted into a voxel grid with voxels of
									// size voxel_size_; this voxel grid will be dilated dilation_iteration_num_
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
		MLSResult() : num_neighbors(0), curvature(0.0f), order(0), valid(false) {}

		MLSResult(const Eigen::Vector3d &a_query_point, const Eigen::Vector3d &a_mean, const Eigen::Vector3d &a_plane_normal,
			const Eigen::Vector3d &a_u, const Eigen::Vector3d &a_v, const Eigen::VectorXd &a_c_vec,
			const int a_num_neighbors, const float a_curvature, const int a_order) :
			query_point(a_query_point), mean(a_mean), plane_normal(a_plane_normal), 
			u_axis(a_u), v_axis(a_v), c_vec(a_c_vec), num_neighbors(a_num_neighbors), 
			curvature(a_curvature), order(a_order), valid(true) {}

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
		inline MLSProjectionResults projectPoint(const Eigen::Vector3d &pt, MLSProjectionMethod method, int required_neighbors = 0) const;

		// brief Project the query point used to generate the mls surface about using the specified method.
		inline MLSProjectionResults projectQueryPoint(MLSProjectionMethod method, int required_neighbors = 0) const;

		// brief Smooth a given point and its neighborghood using Moving Least Squares.
		template <class PointT>
		void computeMLSSurface(const pcl::PointCloud<PointT> &cloud, int index, const std::vector<int> &nn_indices,
			double search_radius, int polynomial_order = 2, boost::function<double(const double)> weight_func = 0);

		Eigen::Vector3d query_point;  // brief The query point about which the mls surface was generated 
		Eigen::Vector3d mean;         // brief The mean point of all the neighbors. 
		Eigen::Vector3d plane_normal; // brief The normal of the local plane of the query point. 
		Eigen::Vector3d u_axis;       // brief The axis corresponding to the u-coordinates of the local plane of the query point. 
		Eigen::Vector3d v_axis;       // brief The axis corresponding to the v-coordinates of the local plane of the query point. 
		Eigen::VectorXd c_vec;        // brief The polynomial coefficients Example: z = c_vec[0] + c_vec[1]*v + c_vec[2]*v^2 + c_vec[3]*u + c_vec[4]*u*v + c_vec[5]*u^2 
		int num_neighbors;            // brief The number of neighbors used to create the mls surface. 
		float curvature;              // brief The curvature at the query point. 
		int order;                    // brief The order of the polynomial. If order > 1 then use polynomial fit 
		bool valid;                   // brief If True, the mls results data is valid, otherwise False. 
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		// brief The default weight function used when fitting a polynomial surface
		inline double computeMLSWeight(const double sq_dist, const double sq_mls_radius) { return (exp(-sq_dist / sq_mls_radius)); }
	};

	// brief A minimalistic implementation of a voxel grid, necessary for the point cloud upsampling
	// note Used only in the case of VOXEL_GRID_DILATION upsampling
	template <class InPointN>
	class MLSVoxelGrid
	{
	public:
		struct Leaf { Leaf() : valid(true) {} bool valid; };

		MLSVoxelGrid(PTR(Pc<InPointN>)& cloud, PTR(PcIndex)& indices, float voxel_size);

		void dilate();

		inline void getIndexIn1D(const Eigen::Vector3i &index, uint64_t &index_1d) const
		{
			index_1d = index[0] * data_size_ * data_size_ +
				index[1] * data_size_ + index[2];
		}

		inline void getIndexIn3D(uint64_t index_1d, Eigen::Vector3i& index_3d) const
		{
			index_3d[0] = static_cast<Eigen::Vector3i::Scalar> (index_1d / (data_size_ * data_size_));
			index_1d -= index_3d[0] * data_size_ * data_size_;
			index_3d[1] = static_cast<Eigen::Vector3i::Scalar> (index_1d / data_size_);
			index_1d -= index_3d[1] * data_size_;
			index_3d[2] = static_cast<Eigen::Vector3i::Scalar> (index_1d);
		}

		inline void getCellIndex(const Eigen::Vector3f &p, Eigen::Vector3i& index) const
		{
			for (int i = 0; i < 3; ++i)
				index[i] = static_cast<Eigen::Vector3i::Scalar> ((p[i] - bounding_min_(i)) / voxel_size_);
		}

		inline void getPosition(const uint64_t &index_1d, Eigen::Vector3f &point) const
		{
			Eigen::Vector3i index_3d;
			getIndexIn3D(index_1d, index_3d);
			for (int i = 0; i < 3; ++i)
				point[i] = static_cast<Eigen::Vector3f::Scalar> (index_3d[i]) * voxel_size_ + bounding_min_[i];
		}

		typedef std::map<uint64_t, Leaf> HashMap;
		HashMap voxel_grid_;
		Eigen::Vector4f bounding_min_, bounding_max_;
		uint64_t data_size_;
		float voxel_size_;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template <class InPointN, class OutPointN>
	class MovingLeastSquares : public pcl::CloudSurfaceProcessing<InPointN, OutPointN>
	{
	public:
		typedef boost::shared_ptr<MovingLeastSquares<InPointN, OutPointN> > Ptr;
		typedef boost::shared_ptr<const MovingLeastSquares<InPointN, OutPointN> > ConstPtr;

		using pcl::PCLBase<InPointN>::input_;
		using pcl::PCLBase<InPointN>::indices_;
		using pcl::PCLBase<InPointN>::fake_indices_;
		using pcl::PCLBase<InPointN>::initCompute;
		using pcl::PCLBase<InPointN>::deinitCompute;

		typedef boost::function<int(int, double, std::vector<int> &, std::vector<float> &)> SearchMethod;

		MovingLeastSquares(
			double search_radius_,
			int order_ = 2,
			MLSProjectionMethod projection_method_ = MLSProjectionMethod::SIMPLE,
			MLSUpsamplingMethod upsample_method_ = MLSUpsamplingMethod::MLSUpsamplingMethod_NONE,
			unsigned int threads_ = 1,
			bool compute_normals_ = true ) 
			: 
			pcl::CloudSurfaceProcessing<InPointN, OutPointN>(),
			search_method_(),
			tree_(),

			// Parms
			search_radius_(search_radius_),
			order_(order_),
			projection_method_(projection_method_),
			upsample_method_(upsample_method_),
			threads_(threads_),
			compute_normals_(compute_normals_),

			// Options
			upsampling_radius_(0.0),
			upsampling_step_(0.0),
			desired_num_points_in_radius_(0),
			cache_mls_results_(true),
			mls_results_(),
			voxel_size_(1.0),
			dilation_iteration_num_(0),

			// Vars
			distinct_cloud_(),
			corresponding_input_indices_(),
			rng_alg_(),
			rng_uniform_distribution_()
		{
			nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;
			sqr_gauss_param_ = search_radius_ * search_radius_;

			if (search_radius_ <= 0 || sqr_gauss_param_ <= 0)
			{
				THROW_EXCEPTION("Invalid search radius or Gaussian parameter");
				return;
			}
		};

		inline void setSearchMethod(const PTR(Acc<InPointN>) &tree)
		{
			tree_ = tree;
			// Declare the search locator definition
			int (Acc<InPointN>::*radiusSearch)(int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances, unsigned int max_nn) const = &KdTree::radiusSearch;
			search_method_ = boost::bind(radiusSearch, boost::ref(tree_), _1, _2, _3, _4, 0);
		}

		void process(Pc<OutPointN>& output);

		// brief Get the set of indices with each point in output having the corresponding point in input 
		inline PointIndicesPtr getCorrespondingIndices() const { return (corresponding_input_indices_); }

	public:
		inline void setDistinctCloud(CONST_PTR(Pc<InPointN>) distinct_cloud) { distinct_cloud_ = distinct_cloud; }
		inline CONST_PTR(Pc<InPointN>) getDistinctCloud() const { return (distinct_cloud_); }

		inline void setUpsamplingRadius(double radius) { upsampling_radius_ = radius; }
		inline double getUpsamplingRadius() const { return (upsampling_radius_); }

		inline void setUpsamplingStepSize(double step_size) { upsampling_step_ = step_size; }
		inline double getUpsamplingStepSize() const { return (upsampling_step_); }

		inline void setPointDensity(int desired_num_points_in_radius) { desired_num_points_in_radius_ = desired_num_points_in_radius; }
		inline int getPointDensity() const { return (desired_num_points_in_radius_); }

		inline void setDilationVoxelSize(float voxel_size) { voxel_size_ = voxel_size; }
		inline float getDilationVoxelSize() const { return (voxel_size_); }

		inline void setDilationIterations(int iterations) { dilation_iteration_num_ = iterations; }
		inline int getDilationIterations() const { return (dilation_iteration_num_); }

		inline void setCacheMLSResults(bool cache_mls_results) { cache_mls_results_ = cache_mls_results; }
		inline bool getCacheMLSResults() const { return (cache_mls_results_); }

		inline const std::vector<MLSResult>& getMLSResults() const { return (mls_results_); }

	protected:
		unsigned int threads_; // brief The maximum number of threads the scheduler should use.

	protected:
		CONST_PTR(Pc<InPointN>) distinct_cloud_; // brief The distinct point cloud that will be projected to the MLS surface.
		SearchMethod search_method_; // brief The search method template for indices.
		PTR(Acc<InPointN>) tree_; // brief A pointer to the spatial search object.
		int order_; // brief The order of the polynomial to be fit.
		int nr_coeff_; // brief Number of coefficients, to be computed from the requested order.
		double search_radius_; // brief The nearest neighbors search radius for each point.
		double sqr_gauss_param_; // brief Parameter for distance based weighting of neighbors (search_radius_ * search_radius_ works fine)
		bool compute_normals_; // brief Parameter that specifies whether the normals should be computed for the input cloud or not
		MLSUpsamplingMethod upsample_method_; // brief Parameter that specifies the upsampling method to be used 
		MLSProjectionMethod projection_method_; // brief Parameter that specifies the projection method to be used.
		PointIndicesPtr corresponding_input_indices_; // brief Collects for each point in output the corrseponding point in the input.

	protected:
		double upsampling_radius_; // brief Radius of the circle in the local point plane that will be sampled for SAMPLE_LOCAL_PLANE upsampling
		double upsampling_step_; // brief Step size for the local plane sampling for SAMPLE_LOCAL_PLANE upsampling
		int desired_num_points_in_radius_; // brief Parameter that specifies the desired number of points within the search radius for RANDOM_UNIFORM_DENSITY upsampling
		bool cache_mls_results_; // brief True if the mls results for the input cloud should be stored. note This is forced to true when using upsampling methods VOXEL_GRID_DILATION or DISTINCT_CLOUD.
		std::vector<MLSResult> mls_results_; // brief Stores the MLS result for each point in the input cloud for VOXEL_GRID_DILATION or DISTINCT_CLOUD upsampling
		float voxel_size_; // brief Voxel size for the VOXEL_GRID_DILATION upsampling method 
		int dilation_iteration_num_; // brief Number of dilation steps for the VOXEL_GRID_DILATION upsampling method

	protected:
		inline int searchForNeighbors(int index, std::vector<int> &indices, std::vector<float> &sqr_distances) const
		{
			return (search_method_(index, search_radius_, indices, sqr_distances));
		}

		void computeMLSPointNormal(int index, const std::vector<int> &nn_indices, Pc<OutPointN>& projected_points, pcl::PointIndices &corresponding_input_indices, MLSResult &mls_result) const;

		inline void addProjectedPointNormal(int index, const Eigen::Vector3d &point, const Eigen::Vector3d &normal, double curvature, 
			Pc<OutPointN>& projected_points, pcl::PointIndices &corresponding_input_indices) const
		{
			OutPointN aux;
			aux.x = static_cast<float> (point[0]);
			aux.y = static_cast<float> (point[1]);
			aux.z = static_cast<float> (point[2]);
			// Copy additional point information if available
			copyMissingFields(input_->points[index], aux);
			if (compute_normals_)
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
