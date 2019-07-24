#pragma once

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include "Common.h"

namespace RecRoom
{
	template<>
	inline LinearSolver Convert<LinearSolver, std::string>(const std::string& v)
	{
		if (v == "EIGEN_QR") return LinearSolver::EIGEN_QR;
		else if (v == "EIGEN_SVD") return LinearSolver::EIGEN_SVD;
		else if (v == "EIGEN_NE") return LinearSolver::EIGEN_NE;
		else return LinearSolver::LinearSolver_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, LinearSolver>(const LinearSolver& v)
	{
		switch (v)
		{
		case LinearSolver::EIGEN_QR: return std::string("EIGEN_QR"); break;
		case LinearSolver::EIGEN_SVD: return std::string("EIGEN_SVD"); break;
		case LinearSolver::EIGEN_NE: return std::string("EIGEN_NE"); break;
		default: return std::string("UNKNOWN"); break;
		}
	}

	template<>
	inline Color Convert<Color, uint32_t>(const uint32_t& v)
	{
		// 4-byte Integer Hashing: http://burtleburtle.net/bob/hash/integer.html
		Color c;
		c.rgba = (v ^ 61) ^ (v >> 16);
		c.rgba = c.rgba + (c.rgba << 3);
		c.rgba = c.rgba ^ (c.rgba >> 4);
		c.rgba = c.rgba * 0x27d4eb2d;
		c.rgba = c.rgba ^ (c.rgba >> 15);
		return c;
	}

	template<class SearchPointType, class InputPointType, class OutputPointType>
	void ProcesserPc2Pc<SearchPointType, InputPointType, OutputPointType>::ProcessInOut(
		const CONST_PTR(Acc<SearchPointType>)& searchSurface,
		const PTR(Pc<InputPointType>)& inOut,
		const CONST_PTR(PcIndex)& filter) const
	{
		if (filter)
		{
			Pc<InputPointType> temp1;
			Pc<OutputPointType> temp2;

			pcl::ExtractIndices<InputPointType> extract;
			extract.setInputCloud(inOut);
			extract.setIndices(filter);
			extract.setNegative(false);
			extract.filter(temp1);

			temp2.resize(temp1.size());
			for (std::size_t px = 0; px < temp1.size(); ++px)
				temp2[px] = temp1[px];

			this->Process(searchSurface, inOut, filter, temp2);

			for (std::size_t idx = 0; idx < filter->size(); ++idx)
				(*inOut)[(*filter)[idx]] = temp2[idx];
		}
		else
		{
			Pc<OutputPointType> temp;
			temp.resize(inOut->size());
			for (std::size_t px = 0; px < inOut->size(); ++px)
				temp[px] = (*inOut)[px];

			this->Process(searchSurface, inOut, filter, temp);

			for (std::size_t px = 0; px < inOut->size(); ++px)
				(*inOut)[px] = temp[px];
		}
	}

	template<class InputPointType, class OutputType>
	bool SearchAnySurfaceProcesserPc<InputPointType, OutputType>::ImplementCheck(
		const CONST_PTR(Acc<InputPointType>)& searchSurface,
		const CONST_PTR(Pc<InputPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		OutputType& output) const
	{
		if (!searchSurface)
		{
			THROW_EXCEPTION("searchSurface is not set");
			return false;
		}

		if (searchSurface->getInputCloud())
		{
		}
		else
		{
			THROW_EXCEPTION("searchSurface is not valid");
			return false;
		}

		return true;
	}

	template<class InputPointType, class OutputPointType>
	bool SearchAnySurfaceProcesserPc2Pc<InputPointType, OutputPointType>::ImplementCheck(
		const CONST_PTR(Acc<InputPointType>)& searchSurface,
		const CONST_PTR(Pc<InputPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<OutputPointType>& output) const
	{
		if (!searchSurface)
		{
			THROW_EXCEPTION("searchSurface is not set");
			return false;
		}

		if (searchSurface->getInputCloud())
		{
		}
		else
		{
			THROW_EXCEPTION("searchSurface is not valid");
			return false;
		}

		return true;
	}

	template<class InputPointType, class OutputType>
	bool SearchInputSurfaceProcesserPc<InputPointType, OutputType>::ImplementCheck(
		const CONST_PTR(Acc<InputPointType>)& searchSurface,
		const CONST_PTR(Pc<InputPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		OutputType& output) const
	{
		if (!searchSurface)
		{
			THROW_EXCEPTION("searchSurface is not set");
			return false;
		}

		if (searchSurface->getInputCloud() != input) // In general case, make sure the tree searches the surface
		{
			THROW_EXCEPTION("searchSurface is not valid, input not match");
			return false;
		}

		if (searchSurface->getIndices())
		{
			if (searchSurface->getIndices() != filter) // In general case, make sure the tree searches the surface
			{
				THROW_EXCEPTION("searchSurface is not valid, filter not match");
				return false;
			}
		}
		else
		{
			if (filter) // In general case, make sure the tree searches the surface
			{
				THROW_EXCEPTION("searchSurface is not valid, filter not match");
				return false;
			}
		}
		
		return true;
	}

	template<class InputPointType, class OutputPointType>
	bool SearchInputSurfaceProcesserPc2Pc<InputPointType, OutputPointType>::ImplementCheck(
		const CONST_PTR(Acc<InputPointType>)& searchSurface,
		const CONST_PTR(Pc<InputPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<OutputPointType>& output) const
	{
		if (!searchSurface)
		{
			THROW_EXCEPTION("searchSurface is not set");
			return false;
		}

		if (searchSurface->getInputCloud() != input) // In general case, make sure the tree searches the surface
		{
			THROW_EXCEPTION("searchSurface is not valid, input not match");
			return false;
		}

		if (searchSurface->getIndices())
		{
			if (searchSurface->getIndices() != filter) // In general case, make sure the tree searches the surface
			{
				THROW_EXCEPTION("searchSurface is not valid, filter not match");
				return false;
			}
		}
		else
		{
			if (filter) // In general case, make sure the tree searches the surface
			{
				THROW_EXCEPTION("searchSurface is not valid, filter not match");
				return false;
			}
		}

		return true;
	}
}