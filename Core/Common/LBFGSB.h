#pragma once

#include <stdexcept>
#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>

#include "Common.h"

// 
namespace RecRoom
{
	struct Options 
	{
		double tol;
		double functol;
		double constrtol;
		int maxIter;
		int m;

		Options() : tol(1e-6), functol(1e-8), constrtol(1e-2), maxIter(1e4), m(10)
		{}
	};

#define DOUBLE_EPS 2.2204e-016
#define DOUBLE_INF HUGE_VAL

	typedef std::function<double(const Eigen::VectorXd &x, void* data)> FunctionOracleType;
	typedef std::function<void(const Eigen::VectorXd &x, Eigen::VectorXd &gradient, void* data)> GradientOracleType;

	class LBFGSB
	{
	protected:
		// contains options for optimization process
		Options Options_;

		// oracles for function value and gradient
		FunctionOracleType FunctionObjectiveOracle_;
		GradientOracleType FunctionGradientOracle_;

		Eigen::MatrixXd W, M;
		Eigen::VectorXd lowerBound, upperBound;
		double theta;
		int DIM;

		std::vector<int> SortIndexes(const std::vector< std::pair<int, double> > &v)
		{
			std::vector<int> idx(v.size());
			for (size_t i = 0; i != idx.size(); ++i)
				idx[i] = v[i].first;
			sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1].second < v[i2].second; });
			return idx;
		}

		/// <summary>
		/// find cauchy point in x
		/// </summary>
		/// <parameter name="x">start in x</parameter>
		void GetGeneralizedCauchyPoint(Eigen::VectorXd &x, Eigen::VectorXd &g, Eigen::VectorXd &x_cauchy, Eigen::VectorXd &c);

		/// <summary>
		/// find valid alpha for (8.5)
		/// </summary>
		/// <parameter name="x_cp">cauchy point</parameter>
		/// <parameter name="du">unconstrained solution of subspace minimization</parameter>
		/// <parameter name="FreeVariables">flag (1 if is free variable and 0 if is not free variable)</parameter>
		double FindAlpha(Eigen::VectorXd &x_cp, Eigen::VectorXd &du, std::vector<int> &FreeVariables);

		/// <summary>
		/// using linesearch to determine step width
		/// </summary>
		/// <parameter name="x">start in x</parameter>
		/// <parameter name="dx">direction</parameter>
		/// <parameter name="f">current value of objective (will be changed)</parameter>
		/// <parameter name="g">current gradient of objective (will be changed)</parameter>
		/// <parameter name="t">step width (will be changed)</parameter>
		void LineSearch(Eigen::VectorXd &x, Eigen::VectorXd dx, double &f, Eigen::VectorXd &g, double &t, void* data);

		/// <summary>
		/// direct primal approach
		/// </summary>
		/// <parameter name="x">start in x</parameter>
		void SubspaceMinimization(Eigen::VectorXd &x_cauchy, Eigen::VectorXd &x, Eigen::VectorXd &c, Eigen::VectorXd &g, Eigen::VectorXd &SubspaceMin);

	public:

		LBFGSB(const Eigen::VectorXd &lowerBound, const Eigen::VectorXd &upperBound)
			: lowerBound(lowerBound), upperBound(upperBound), theta(1.0), DIM(lowerBound.rows())
		{
			W = Eigen::MatrixXd::Zero(DIM, 0);
			M = Eigen::MatrixXd::Zero(0, 0);
		}

		LBFGSB(Options &Options, const Eigen::VectorXd &lowerBound, const Eigen::VectorXd &upperBound)
			: lowerBound(lowerBound), upperBound(upperBound), theta(1.0), DIM(lowerBound.rows())
		{
			Options_ = Options;
			W = Eigen::MatrixXd::Zero(DIM, 0);
			M = Eigen::MatrixXd::Zero(0, 0);
		}

		void Solve(Eigen::VectorXd &x0, const FunctionOracleType& FunctionValue, const GradientOracleType& FunctionGradient, void* data);
	};
}

#include "LBFGSB.hpp"
