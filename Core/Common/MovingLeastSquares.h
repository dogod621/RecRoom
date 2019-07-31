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

#include "Interpolator/InterpolatorPcNearest.h"

namespace RecRoom
{
	enum MLSProjectionMethod : Flag
	{
		MLSProjectionMethod_NONE = 0,	// brief Project to the mls plane.
		SIMPLE = 1,						// brief Project along the mls plane normal to the polynomial surface.
		ORTHOGONAL = 2					// brief Project to the closest point on the polynonomial surface.
	};

	enum MLSUpsamplingMethod : Flag
	{
		MLSUpsamplingMethod_NONE = 0,	// brief No upsampling will be done, only the input points will be projected to their own MLS surfaces.
		DISTINCT_CLOUD = 1,				// brief Project the points of the distinct cloud to the MLS surface. 
		SAMPLE_LOCAL_PLANE = 2,			// brief The local plane of each input point will be sampled in a circular fashion
										// using the upsamplingRadius and the upsamplingStep parameters. 
		RANDOM_UNIFORM_DENSITY = 3,		// brief The local plane of each input point will be sampled using an uniform random
										// distribution such that the density of points is constant throughout the
										// cloud - given by the pointDensity parameter.
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
		inline void getMLSCoordinates(const Eigen::Vector3d &pt, double &u, double &v, double &w) const
		{
			Eigen::Vector3d delta = pt - mean;
			u = delta.dot(uAxis);
			v = delta.dot(vAxis);
			w = delta.dot(planeNormal);
		}

		// brief Given a point calculate it's 2D location in the MLS frame.
		inline void getMLSCoordinates(const Eigen::Vector3d &pt, double &u, double &v) const
		{
			Eigen::Vector3d delta = pt - mean;
			u = delta.dot(uAxis);
			v = delta.dot(vAxis);
		}

		// brief Calculate the polynomial
		inline double getPolynomialValue(const double u, const double v) const
		{
			// Compute the polynomial's terms at the current point
			// Example for second order: z = a + b*y + c*y^2 + d*x + e*x*y + f*x^2
			double u_pow, v_pow, result;
			int j = 0;
			u_pow = 1;
			result = 0;
			for (int ui = 0; ui <= order; ++ui)
			{
				v_pow = 1;
				for (int vi = 0; vi <= order - ui; ++vi)
				{
					result += cAxis[j++] * u_pow * v_pow;
					v_pow *= v;
				}
				u_pow *= u;
			}

			return (result);
		}

		// brief Calculate the polynomial's first and second partial derivatives.
		inline PolynomialPartialDerivative getPolynomialPartialDerivative(const double u, const double v) const
		{
			// Compute the displacement along the normal using the fitted polynomial
			// and compute the partial derivatives needed for estimating the normal
			PolynomialPartialDerivative d;
			Eigen::VectorXd u_pow(order + 2), v_pow(order + 2);
			int j = 0;

			d.z = d.z_u = d.z_v = d.z_uu = d.z_vv = d.z_uv = 0;
			u_pow(0) = v_pow(0) = 1;
			for (int ui = 0; ui <= order; ++ui)
			{
				for (int vi = 0; vi <= order - ui; ++vi)
				{
					// Compute displacement along normal
					d.z += u_pow(ui) * v_pow(vi) * cAxis[j];

					// Compute partial derivatives
					if (ui >= 1)
						d.z_u += cAxis[j] * ui * u_pow(ui - 1) * v_pow(vi);

					if (vi >= 1)
						d.z_v += cAxis[j] * vi * u_pow(ui) * v_pow(vi - 1);

					if (ui >= 1 && vi >= 1)
						d.z_uv += cAxis[j] * ui * u_pow(ui - 1) * vi * v_pow(vi - 1);

					if (ui >= 2)
						d.z_uu += cAxis[j] * ui * (ui - 1) * u_pow(ui - 2) * v_pow(vi);

					if (vi >= 2)
						d.z_vv += cAxis[j] * vi * (vi - 1) * u_pow(ui) * v_pow(vi - 2);

					if (ui == 0)
						v_pow(vi + 1) = v_pow(vi) * v;

					++j;
				}
				u_pow(ui + 1) = u_pow(ui) * u;
			}

			return (d);
		}

		// brief Calculate the principle curvatures using the polynomial surface.
		inline Eigen::Vector2f calculatePrincipleCurvatures(const double u, const double v) const
		{
			Eigen::Vector2f k(1e-5, 1e-5);

			// Note: this use the Monge Patch to derive the Gaussian curvature and Mean Curvature found here http://mathworld.wolfram.com/MongePatch.html
			// Then:
			//      k1 = H + sqrt(H^2 - K)
			//      k1 = H - sqrt(H^2 - K)
			if (order > 1 && cAxis.size() >= (order + 1) * (order + 2) / 2 && pcl_isfinite(cAxis[0]))
			{
				PolynomialPartialDerivative d = getPolynomialPartialDerivative(u, v);
				double Z = 1 + d.z_u * d.z_u + d.z_v * d.z_v;
				double Zlen = std::sqrt(Z);
				double K = (d.z_uu * d.z_vv - d.z_uv * d.z_uv) / (Z * Z);
				double H = ((1.0 + d.z_v * d.z_v) * d.z_uu - 2.0 * d.z_u * d.z_v * d.z_uv + (1.0 + d.z_u * d.z_u) * d.z_vv) / (2.0 * Zlen * Zlen * Zlen);
				double disc2 = H * H - K;
				assert(disc2 >= 0.0);
				double disc = std::sqrt(disc2);
				k[0] = H + disc;
				k[1] = H - disc;

				if (std::abs(k[0]) > std::abs(k[1])) std::swap(k[0], k[1]);
			}
			else
			{
				THROW_EXCEPTION("No Polynomial fit data, unable to calculate the principle curvatures");
			}

			return (k);
		}

		// brief Project a point orthogonal to the polynomial surface.
		inline MLSProjectionResults projectPointOrthogonalToPolynomialSurface(const double u, const double v, const double w) const
		{
			double gu = u;
			double gv = v;
			double gw = 0;

			MLSProjectionResults result;
			result.normal = planeNormal;
			if (order > 1 && cAxis.size() >= (order + 1) * (order + 2) / 2 && pcl_isfinite(cAxis[0]))
			{
				PolynomialPartialDerivative d = getPolynomialPartialDerivative(gu, gv);
				gw = d.z;
				double err_total;
				double dist1 = std::abs(gw - w);
				double dist2;
				do
				{
					double e1 = (gu - u) + d.z_u * gw - d.z_u * w;
					double e2 = (gv - v) + d.z_v * gw - d.z_v * w;

					double F1u = 1 + d.z_uu * gw + d.z_u * d.z_u - d.z_uu * w;
					double F1v = d.z_uv * gw + d.z_u * d.z_v - d.z_uv * w;

					double F2u = d.z_uv * gw + d.z_v * d.z_u - d.z_uv * w;
					double F2v = 1 + d.z_vv * gw + d.z_v * d.z_v - d.z_vv * w;

					Eigen::MatrixXd J(2, 2);
					J(0, 0) = F1u;
					J(0, 1) = F1v;
					J(1, 0) = F2u;
					J(1, 1) = F2v;

					Eigen::Vector2d err(e1, e2);
					Eigen::Vector2d update = J.inverse() * err;
					gu -= update(0);
					gv -= update(1);

					d = getPolynomialPartialDerivative(gu, gv);
					gw = d.z;
					dist2 = std::sqrt((gu - u) * (gu - u) + (gv - v) * (gv - v) + (gw - w) * (gw - w));

					err_total = std::sqrt(e1 * e1 + e2 * e2);

				} while (err_total > 1e-8 && dist2 < dist1);

				if (dist2 > dist1) // the optimization was diverging reset the coordinates for simple projection
				{
					gu = u;
					gv = v;
					d = getPolynomialPartialDerivative(u, v);
					gw = d.z;
				}

				result.u = gu;
				result.v = gv;
				result.normal -= (d.z_u * uAxis + d.z_v * vAxis);
				result.normal.normalize();
			}

			result.point = mean + gu * uAxis + gv * vAxis + gw * planeNormal;

			return (result);
		}

		// brief Project a point onto the MLS plane.
		inline MLSProjectionResults projectPointToMLSPlane(const double u, const double v) const
		{
			MLSProjectionResults result;
			result.u = u;
			result.v = v;
			result.normal = planeNormal;
			result.point = mean + u * uAxis + v * vAxis;

			return (result);
		}

		// brief Project a point along the MLS plane normal to the polynomial surface.
		inline MLSProjectionResults projectPointSimpleToPolynomialSurface(const double u, const double v) const
		{
			MLSProjectionResults result;
			double w = 0;

			result.u = u;
			result.v = v;
			result.normal = planeNormal;

			if (order > 1 && cAxis.size() >= (order + 1) * (order + 2) / 2 && pcl_isfinite(cAxis[0]))
			{
				PolynomialPartialDerivative d = getPolynomialPartialDerivative(u, v);
				w = d.z;
				result.normal -= (d.z_u * uAxis + d.z_v * vAxis);
				result.normal.normalize();
			}

			result.point = mean + u * uAxis + v * vAxis + w * planeNormal;

			return (result);
		}

		// brief Project a point using the specified method.
		inline MLSProjectionResults projectPoint(const Eigen::Vector3d &pt, MLSProjectionMethod method, int requiredNeighbors = 0) const
		{
			double u, v, w;
			getMLSCoordinates(pt, u, v, w);

			MLSProjectionResults proj;
			if (order > 1 && numNeighbors >= requiredNeighbors && pcl_isfinite(cAxis[0]) && method != MLSProjectionMethod::MLSProjectionMethod_NONE)
			{
				if (method == MLSProjectionMethod::ORTHOGONAL)
					proj = projectPointOrthogonalToPolynomialSurface(u, v, w);
				else // MLSProjectionMethod::SIMPLE
					proj = projectPointSimpleToPolynomialSurface(u, v);
			}
			else
			{
				proj = projectPointToMLSPlane(u, v);
			}

			return  (proj);
		}

		// brief Project the query point used to generate the mls surface about using the specified method.
		inline MLSProjectionResults projectQueryPoint(MLSProjectionMethod method, int requiredNeighbors = 0) const
		{
			MLSProjectionResults proj;
			if (order > 1 && numNeighbors >= requiredNeighbors && pcl_isfinite(cAxis[0]) && method != MLSProjectionMethod::MLSProjectionMethod_NONE)
			{
				if (method == MLSProjectionMethod::ORTHOGONAL)
				{
					double u, v, w;
					getMLSCoordinates(queryPoint, u, v, w);
					proj = projectPointOrthogonalToPolynomialSurface(u, v, w);
				}
				else // MLSProjectionMethod::SIMPLE
				{
					// Projection onto MLS surface along Darboux normal to the height at (0,0)
					proj.point = mean + (cAxis[0] * planeNormal);

					// Compute tangent vectors using the partial derivates evaluated at (0,0) which is cAxis[order+1] and cAxis[1]
					proj.normal = planeNormal - cAxis[order + 1] * uAxis - cAxis[1] * vAxis;
					proj.normal.normalize();
				}
			}
			else
			{
				proj.normal = planeNormal;
				proj.point = mean;
			}

			return (proj);
		}

		// brief Smooth a given point and its neighborghood using Moving Least Squares.
		template <class PointT>
		void computeMLSSurface(const pcl::PointCloud<PointT> &cloud, int index, const PcIndex& nnIndices,
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
		using pcl::PCLBase<InPointN>::initCompute;
		using pcl::PCLBase<InPointN>::deinitCompute;

		using Interpolator = InterpolatorPc<InPointN, OutPointN>;
		using InterpolatorNearest = InterpolatorPcNearest<InPointN, OutPointN>;

		MovingLeastSquares(
			double searchRadius,
			int order = 2,
			MLSProjectionMethod projectionMethod = MLSProjectionMethod::SIMPLE,
			MLSUpsamplingMethod upsampleMethod = MLSUpsamplingMethod::MLSUpsamplingMethod_NONE,
			bool computeNormals = true)
			: 
			pcl::CloudSurfaceProcessing<InPointN, OutPointN>(),
			ThreadAble(),
			searchMethod(),

			// Parms
			searchRadius(searchRadius),
			order(order),
			projectionMethod(projectionMethod),
			upsampleMethod(upsampleMethod),
			computeNormals(computeNormals),

			// Options
			upsamplingRadius(0.0),
			upsamplingStep(0.0),
			pointDensity(0),
			cacheMLSResults(true),

			// Vars
			distinctCloud(),
			correspondingInputIndices(),
			mlsResults()
		{
			numCoeff = (order + 1) * (order + 2) / 2;

			if (searchRadius <= 0 )
			{
				THROW_EXCEPTION("Invalid search radius");
				return;
			}
		};

		void process(Pc<OutPointN>& output);

		// brief Get the set of indices with each point in output having the corresponding point in input 
		inline PTR(PcIndex) getCorrespondingIndices() const { return (correspondingInputIndices); }
		inline const std::vector<MLSResult>& getMLSResults() const { return (mlsResults); }

	public:
		inline void setSearchMethod(const PTR(Acc<InPointN>)& v) { searchMethod = v; }

		/** \brief Get a pointer to the search method used. */
		inline PTR(Acc<InPointN>) getSearchMethod() const
		{
			return searchMethod;
		}

		inline void setDistinctCloud(CONST_PTR(Pc<InPointN>) v) { distinctCloud = v; }
		inline CONST_PTR(Pc<InPointN>) getDistinctCloud() const { return (distinctCloud); }

		inline void setUpsamplingRadius(double v) { upsamplingRadius = v; }
		inline double getUpsamplingRadius() const { return (upsamplingRadius); }

		inline void setUpsamplingStep(double v) { upsamplingStep = v; }
		inline double getUpsamplingStep() const { return (upsamplingStep); }

		inline void setPointDensity(int v) { pointDensity = v; }
		inline int getPointDensity() const { return (pointDensity); }

		inline void setCacheMLSResults(bool v) { cacheMLSResults = v; }
		inline bool getCacheMLSResults() const { return (cacheMLSResults); }

	protected:
		PTR(Acc<InPointN>) searchMethod; // brief A pointer to the spatial search object.
		int order; // brief The order of the polynomial to be fit.
		int numCoeff; // brief Number of coefficients, to be computed from the requested order.
		double searchRadius; // brief The nearest neighbors search radius for each point.
		bool computeNormals; // brief Parameter that specifies whether the normals should be computed for the input cloud or not
		MLSUpsamplingMethod upsampleMethod; // brief Parameter that specifies the upsampling method to be used 
		MLSProjectionMethod projectionMethod; // brief Parameter that specifies the projection method to be used.
		PTR(PcIndex) correspondingInputIndices; // brief Collects for each point in output the corrseponding point in the input.
		std::vector<MLSResult> mlsResults; // brief Stores the MLS result for each point in the input cloud for DISTINCT_CLOUD upsampling

	protected:
		CONST_PTR(Pc<InPointN>) distinctCloud; // brief The distinct point cloud that will be projected to the MLS surface for DISTINCT_CLOUD upsampling
		double upsamplingRadius; // brief Radius of the circle in the local point plane that will be sampled for SAMPLE_LOCAL_PLANE upsampling
		double upsamplingStep; // brief Step size for the local plane sampling for SAMPLE_LOCAL_PLANE upsampling
		int pointDensity; // brief Parameter that specifies the desired number of points within the search radius for RANDOM_UNIFORM_DENSITY upsampling
		bool cacheMLSResults; // brief True if the mls results for the input cloud should be stored. note This is forced to true when using upsampling methods DISTINCT_CLOUD.
		
	protected:
		static void GenerateMLSTask(
			int id,
			void* self,
			void* projectedPointSet,
			void* correspondingInputIndicesSet );

		void computeMLSPointNormal(
			int index, const PcIndex &nnIndices, Pc<OutPointN>& projectedPoints, PcIndex& correspondingInputIndices_);

		inline void addProjectedPointNormal(
			int index, const Eigen::Vector3d &point, const Eigen::Vector3d &normal, double curvature, 
			Pc<OutPointN>& projectedPoints, PcIndex& correspondingInputIndices_) const
		{
			OutPointN aux;
			aux.x = static_cast<float> (point[0]);
			aux.y = static_cast<float> (point[1]);
			aux.z = static_cast<float> (point[2]);
			// Copy additional point information if available
			copyMissingFields(input_->points[index], aux);
			if (computeNormals)
			{
				const InPointN& inP = (*input_)[index];
				if ((inP.normal_x * normal.x() + inP.normal_y * normal.x() + inP.normal_z * normal.z()) > 0)
				{
					aux.normal_x = static_cast<float> (normal[0]);
					aux.normal_y = static_cast<float> (normal[1]);
					aux.normal_z = static_cast<float> (normal[2]);
				}
				else
				{
					aux.normal_x = static_cast<float> (-normal[0]);
					aux.normal_y = static_cast<float> (-normal[1]);
					aux.normal_z = static_cast<float> (-normal[2]);
				}
				aux.curvature = curvature;
			}
			projectedPoints.push_back(aux);
			correspondingInputIndices_.push_back(index);
		}

		inline void addProjectedPointNormal(
			int index, const Eigen::Vector3d& point, const Eigen::Vector3d& normal, double curvature,
			OutPointN& projectedPoint, int& correspondingInputIndex) const
		{
			OutPointN aux;
			aux.x = static_cast<float> (point[0]);
			aux.y = static_cast<float> (point[1]);
			aux.z = static_cast<float> (point[2]);
			// Copy additional point information if available
			copyMissingFields(input_->points[index], aux);
			if (computeNormals)
			{
				const InPointN& inP = (*input_)[index];
				if ((inP.normal_x * normal.x() + inP.normal_y * normal.x() + inP.normal_z * normal.z()) > 0)
				{
					aux.normal_x = static_cast<float> (normal[0]);
					aux.normal_y = static_cast<float> (normal[1]);
					aux.normal_z = static_cast<float> (normal[2]);
				}
				else
				{
					aux.normal_x = static_cast<float> (-normal[0]);
					aux.normal_y = static_cast<float> (-normal[1]);
					aux.normal_z = static_cast<float> (-normal[2]);
				}
				aux.curvature = curvature;
			}
			projectedPoint = aux;
			correspondingInputIndex = index;
		}
		
		inline void copyMissingFields(const InPointN& inP, OutPointN& outP) const
		{
			OutPointN temp = outP;
			pcl::copyPoint(inP, outP);
			outP.x = temp.x;
			outP.y = temp.y;
			outP.z = temp.z;
		}

		virtual void performProcessing(Pc<OutPointN>& output);
		void performUpsampling(Pc<OutPointN>& output);

	private:
		std::string getClassName() const { return ("MovingLeastSquares"); }
	};
}

#include "MovingLeastSquares.hpp"
