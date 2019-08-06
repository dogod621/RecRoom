#include "LBFGSB.h"

namespace RecRoom
{
	void LBFGSB::GetGeneralizedCauchyPoint(Eigen::VectorXd &x, Eigen::VectorXd &g, Eigen::VectorXd &x_cauchy, Eigen::VectorXd &c)
	{
		// PAGE 8
		// Algorithm CP: Computation of the generalized Cauchy point
		// Given x,l,u,g, and B = \theta I-WMW

		// {all t_i} = { (idx,value), ... }
		// TODO: use "std::set" ?
		std::vector<std::pair<int, double> > SetOfT;
		// the feasible set is implicitly given by "SetOfT - {t_i==0}"
		Eigen::VectorXd d = Eigen::VectorXd::Zero(DIM, 1);

		// n operations
		for (int j = 0; j < DIM; j++) {
			if (g(j) == 0) {
				SetOfT.push_back(std::make_pair(j, DOUBLE_INF));
			}
			else {
				double tmp = 0;
				if (g(j) < 0) {
					tmp = (x(j) - upperBound(j)) / g(j);
				}
				else {
					tmp = (x(j) - lowerBound(j)) / g(j);
				}
				d(j) = -g(j);
				SetOfT.push_back(std::make_pair(j, tmp));
			}

		}

		// paper: using heapsort
		// sortedindices [1,0,2] means the minimal element is on the 1th entry
		std::vector<int> SortedIndices = SortIndexes(SetOfT);

		x_cauchy = x;
		// Initialize
		// p := 	W^T*p
		Eigen::VectorXd p = (W.transpose() * d);						// (2mn operations)
		// c := 	0
		c = Eigen::MatrixXd::Zero(M.rows(), 1);
		// f' := 	g^T*d = -d^Td
		double f_prime = -d.dot(d);							// (n operations)
		// f'' :=	\theta*d^T*d-d^T*W*M*W^T*d = -\theta*f' - p^T*M*p
		double f_doubleprime = (double)(-1.0 * theta) * f_prime - p.dot(M * p);// (O(m^2) operations)
		// \delta t_min :=	-f'/f''
		double dt_min = -f_prime / f_doubleprime;
		// t_old := 	0
		double t_old = 0;
		// b := 	argmin {t_i , t_i >0}
		int i = 0;
		for (int j = 0; j < DIM; j++) {
			i = j;
			if (SetOfT[SortedIndices[j]].second != 0)
				break;
		}
		int b = SortedIndices[i];
		// see below
		// t        			:= 	min{t_i : i in F}
		double t = SetOfT[b].second;
		// \delta t 			:= 	t - 0
		double dt = t - t_old;

		// examination of subsequent segments
		while ((dt_min >= dt) && (i < DIM)) {
			if (d(b) > 0)
				x_cauchy(b) = upperBound(b);
			else if (d(b) < 0)
				x_cauchy(b) = lowerBound(b);

			// z_b = x_p^{cp} - x_b
			double zb = x_cauchy(b) - x(b);
			// c   :=  c +\delta t*p
			c += dt * p;
			// cache
			Eigen::VectorXd wbt = W.row(b);

			f_prime += dt * f_doubleprime + (double)g(b) * g(b)
				+ (double)theta * g(b) * zb
				- (double)g(b) * wbt.transpose() * (M * c);
			f_doubleprime += (double)-1.0 * theta * g(b) * g(b)
				- (double) 2.0 * (g(b) * (wbt.dot(M * p)))
				- (double)g(b) * g(b) * wbt.transpose() * (M * wbt);
			p += g(b) * wbt.transpose();
			d(b) = 0;
			dt_min = -f_prime / f_doubleprime;
			t_old = t;
			++i;
			if (i < DIM) {
				b = SortedIndices[i];
				t = SetOfT[b].second;
				dt = t - t_old;
			}

		}

		dt_min = std::max(dt_min, 0.0);
		t_old += dt_min;

		for (int ii = i; ii < x_cauchy.rows(); ii++)
		{
			x_cauchy(SortedIndices[ii]) = x(SortedIndices[ii]) + t_old * d(SortedIndices[ii]);
		}

		c += dt_min * p;
	}

	double LBFGSB::FindAlpha(Eigen::VectorXd &x_cp, Eigen::VectorXd &du, std::vector<int> &FreeVariables)
	{
		/* this returns
		 * a* = max {a : a <= 1 and  l_i-xc_i <= a*d_i <= u_i-xc_i}
		 */
		double alphastar = 1;
		const unsigned int n = FreeVariables.size();
		for (unsigned int i = 0; i < n; i++) 
		{
			if (du(i) > 0) 
			{
				alphastar = std::min(alphastar, (upperBound(FreeVariables[i]) - x_cp(FreeVariables[i])) / du(i));
			}
			else 
			{
				alphastar = std::min(alphastar, (lowerBound(FreeVariables[i]) - x_cp(FreeVariables[i])) / du(i));
			}
		}
		return alphastar;
	}

	void LBFGSB::LineSearch(Eigen::VectorXd &x, Eigen::VectorXd dx, double &f, Eigen::VectorXd &g, double &t, void* data)
	{

		const double alpha = 0.2;
		const double beta = 0.8;

		const double f_in = f;
		const Eigen::VectorXd g_in = g;
		const double Cache = alpha * g_in.dot(dx);

		t = 1.0;
		f = FunctionObjectiveOracle_(x + t * dx, data);
		while (f > f_in + t * Cache) 
		{
			t *= beta;
			f = FunctionObjectiveOracle_(x + t * dx, data);
		}
		FunctionGradientOracle_(x + t * dx, g, data);
		x += t * dx;
	}

	void LBFGSB::SubspaceMinimization(Eigen::VectorXd &x_cauchy, Eigen::VectorXd &x, Eigen::VectorXd &c, Eigen::VectorXd &g, Eigen::VectorXd &SubspaceMin)
	{
		// cached value: ThetaInverse=1/theta;
		double theta_inverse = 1 / theta;

		// size of "t"
		std::vector<int> FreeVariablesIndex;

		//std::cout << "free vars " << FreeVariables.rows() << std::endl;
		for (int i = 0; i < x_cauchy.rows(); i++)
		{
			if ((x_cauchy(i) != upperBound(i)) && (x_cauchy(i) != lowerBound(i)))
			{
				FreeVariablesIndex.push_back(i);
			}
		}
		const int FreeVarCount = FreeVariablesIndex.size();

		Eigen::MatrixXd WZ = Eigen::MatrixXd::Zero(W.cols(), FreeVarCount);

		for (int i = 0; i < FreeVarCount; i++)
			WZ.col(i) = W.row(FreeVariablesIndex[i]);

		// r=(g+theta*(x_cauchy-x)-W*(M*c));
		Eigen::VectorXd rr = (g + theta * (x_cauchy - x) - W * (M * c));
		// r=r(FreeVariables);
		Eigen::VectorXd r = Eigen::MatrixXd::Zero(FreeVarCount, 1);
		for (int i = 0; i < FreeVarCount; i++)
			r.row(i) = rr.row(FreeVariablesIndex[i]);

		// STEP 2: "v = w^T*Z*r" and STEP 3: "v = M*v"
		Eigen::VectorXd v = M * (WZ * r);
		// STEP 4: N = 1/theta*W^T*Z*(W^T*Z)^T
		Eigen::MatrixXd N = theta_inverse * WZ * WZ.transpose();
		// N = I - MN
		N = Eigen::MatrixXd::Identity(N.rows(), N.rows()) - M * N;
		// STEP: 5
		// v = N^{-1}*v
		v = N.lu().solve(v);
		// STEP: 6
		// HERE IS A MISTAKE IN THE ORIGINAL PAPER!
		Eigen::VectorXd du = -theta_inverse * r
			- theta_inverse * theta_inverse * WZ.transpose() * v;
		// STEP: 7
		double alpha_star = FindAlpha(x_cauchy, du, FreeVariablesIndex);

		// STEP: 8
		Eigen::VectorXd dStar = alpha_star * du;

		SubspaceMin = x_cauchy;
		for (int i = 0; i < FreeVarCount; i++) {
			SubspaceMin(FreeVariablesIndex[i]) = SubspaceMin(
				FreeVariablesIndex[i]) + dStar(i);
		}
	}

	void LBFGSB::Solve(Eigen::VectorXd &x0, const FunctionOracleType& FunctionValue, const GradientOracleType& FunctionGradient, void* data)
	{
		FunctionObjectiveOracle_ = FunctionValue;
		FunctionGradientOracle_ = FunctionGradient;

		if (x0.rows() != lowerBound.rows())
			THROW_EXCEPTION("lower bound size incorrect");
		if (x0.rows() != upperBound.rows())
			THROW_EXCEPTION("upper bound size incorrect");

		if ((x0.array() < lowerBound.array()).all())
			THROW_EXCEPTION("seed is not feasible (violates lower bound)");
		if ((x0.array() > upperBound.array()).all())
			THROW_EXCEPTION("seed is not feasible (violates upper bound)");

		Eigen::MatrixXd yHistory = Eigen::MatrixXd::Zero(DIM, 0);
		Eigen::MatrixXd sHistory = Eigen::MatrixXd::Zero(DIM, 0);

		Eigen::VectorXd x = x0;
		Eigen::VectorXd g;
		int k = 0;

		double f = FunctionObjectiveOracle_(x, data);
		FunctionGradientOracle_(x, g, data);

		theta = 1.0;

		W = Eigen::MatrixXd::Zero(DIM, 0);
		M = Eigen::MatrixXd::Zero(0, 0);

		auto noConvergence = [&](Eigen::VectorXd& x, Eigen::VectorXd& g)->bool 
		{
			return (((x - g).cwiseMax(lowerBound).cwiseMin(upperBound) - x).lpNorm<Eigen::Infinity>() >= Options_.tol);
		};

		while (noConvergence(x, g) && (k < Options_.maxIter))
		{
			double f_old = f;
			Eigen::VectorXd x_old = x;
			Eigen::VectorXd g_old = g;

			// STEP 2: compute the cauchy point by algorithm CP
			Eigen::VectorXd CauchyPoint = Eigen::MatrixXd::Zero(DIM, 1), c = Eigen::MatrixXd::Zero(DIM, 1);
			GetGeneralizedCauchyPoint(x, g, CauchyPoint, c);

			// STEP 3: compute a search direction d_k by the primal method
			Eigen::VectorXd SubspaceMin;
			SubspaceMinimization(CauchyPoint, x, c, g, SubspaceMin);

			Eigen::MatrixXd H;
			double Length = 0;

			// STEP 4: perform linesearch and STEP 5: compute gradient
			LineSearch(x, SubspaceMin - x, f, g, Length, data);

			// prepare for next iteration
			Eigen::VectorXd newY = g - g_old;
			Eigen::VectorXd newS = x - x_old;

			// STEP 6:
			double test = newS.dot(newY);
			test = (test < 0) ? -1.0 * test : test;

			if (test > DOUBLE_EPS * newY.squaredNorm())
			{
				if (k < Options_.m) 
				{
					yHistory.conservativeResize(DIM, k + 1);
					sHistory.conservativeResize(DIM, k + 1);
				}
				else 
				{

					yHistory.leftCols(Options_.m - 1) = yHistory.rightCols(Options_.m - 1).eval();
					sHistory.leftCols(Options_.m - 1) = sHistory.rightCols(Options_.m - 1).eval();
				}
				yHistory.rightCols(1) = newY;
				sHistory.rightCols(1) = newS;

				// STEP 7:
				theta = (double)(newY.transpose() * newY) / (newY.transpose() * newS);

				W = Eigen::MatrixXd::Zero(yHistory.rows(), yHistory.cols() + sHistory.cols());
				W << yHistory, (theta * sHistory);

				Eigen::MatrixXd A = sHistory.transpose() * yHistory;
				Eigen::MatrixXd L = A.triangularView<Eigen::StrictlyLower>();
				Eigen::MatrixXd MM(A.rows() + L.rows(), A.rows() + L.cols());
				Eigen::MatrixXd D = -1 * A.diagonal().asDiagonal();
				MM << D, L.transpose(), L, ((sHistory.transpose() * sHistory) * theta);

				M = MM.inverse();
			}

			Eigen::VectorXd ttt = Eigen::MatrixXd::Zero(1, 1);
			ttt(0) = f_old - f;
			if (ttt.norm() < Options_.tol) 
			{
				// successive function values too similar
				break;
			}
			k++;

		}
		x0 = x;
	}
}