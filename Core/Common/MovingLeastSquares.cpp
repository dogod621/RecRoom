#include "MovingLeastSquares.h"

namespace RecRoom
{
	void MLSResult::getMLSCoordinates(const Eigen::Vector3d &pt, double &u, double &v, double &w) const
	{
		Eigen::Vector3d delta = pt - mean;
		u = delta.dot(uAxis);
		v = delta.dot(vAxis);
		w = delta.dot(planeNormal);
	}

	void MLSResult::getMLSCoordinates(const Eigen::Vector3d &pt, double &u, double &v) const
	{
		Eigen::Vector3d delta = pt - mean;
		u = delta.dot(uAxis);
		v = delta.dot(vAxis);
	}

	double MLSResult::getPolynomialValue(const double u, const double v) const
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

	PolynomialPartialDerivative MLSResult::getPolynomialPartialDerivative(const double u, const double v) const
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

	Eigen::Vector2f MLSResult::calculatePrincipleCurvatures(const double u, const double v) const
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

	MLSProjectionResults MLSResult::projectPointOrthogonalToPolynomialSurface(const double u, const double v, const double w) const
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

	MLSProjectionResults MLSResult::projectPointToMLSPlane(const double u, const double v) const
	{
		MLSProjectionResults result;
		result.u = u;
		result.v = v;
		result.normal = planeNormal;
		result.point = mean + u * uAxis + v * vAxis;

		return (result);
	}

	MLSProjectionResults MLSResult::projectPointSimpleToPolynomialSurface(const double u, const double v) const
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

	MLSProjectionResults MLSResult::projectPoint(const Eigen::Vector3d &pt, MLSProjectionMethod method, int requiredNeighbors) const
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

	MLSProjectionResults MLSResult::projectQueryPoint(MLSProjectionMethod method, int requiredNeighbors) const
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
}
