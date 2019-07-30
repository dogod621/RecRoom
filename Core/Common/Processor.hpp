#pragma once

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include "Processor.h"

namespace RecRoom
{
	template<class SearchPointType, class InputPointType, class OutputType>
	PTR(PcIndex) ProcessorPc<SearchPointType, InputPointType, OutputType>::GenFilter(
		const CONST_PTR(Pc<InputPointType>)& input,
		const CONST_PTR(PcIndex)& filter) const
	{
		PTR(PcIndex) filter2(new PcIndex);

		if (!input->is_dense)
		{
			if (filter)
			{
				filter2->reserve(filter->size());
				for (PcIndex::const_iterator it = filter->begin(); it != filter->end(); ++it)
				{
					if (InputPointValid((*input)[(*it)]))
						filter2->push_back(*it);
				}
				std::sort(filter2->begin(), filter2->end());
			}
			else
			{
				filter2->reserve(input->size());
				for (int px = 0; px < input->size(); ++px)
				{
					if (InputPointValid((*input)[px]))
						filter2->push_back(px);
				}
			}
		}
		else
		{
			if (filter)
			{
				(*filter2) = (*filter);
				std::sort(filter2->begin(), filter2->end());
			}
			else
			{
				filter2->reserve(input->size());
				for (int px = 0; px < input->size(); ++px)
				{
					if (InputPointValid((*input)[px]))
						filter2->push_back(px);
				}
			}
		}

		return filter2;
	}

	template<class SearchPointType, class InputPointType, class OutputType>
	void ProcessorPc<SearchPointType, InputPointType, OutputType>::Process(
		const CONST_PTR(Acc<SearchPointType>)& searchSurface,
		const CONST_PTR(Pc<InputPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		OutputType& output) const
	{
		//if ((&(*input)) == (&output))
		//	THROW_EXCEPTION("input point to output");

		PTR(PcIndex) filter2 = GenFilter(input, filter);

		PRINT_INFO(name + " - Start");

		if (ImplementCheck(searchSurface, input, filter, output))
			ImplementProcess(searchSurface, input, filter2, output);
		else
			THROW_EXCEPTION(name + " - Not pass check");

		{
			std::stringstream ss;
			ss << name << " - End - inSize: " << filter->size() << ", outSize: " << OutputSize(output);
			PRINT_INFO(ss.str());
		}
	}

	template<class SearchPointType, class InputPointType, class OutputPointType>
	void ProcessorPc2Pc<SearchPointType, InputPointType, OutputPointType>::Process(
		const CONST_PTR(Acc<SearchPointType>)& searchSurface,
		const CONST_PTR(Pc<InputPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<OutputPointType>& output) const
	{
		//if ((&(*input)) == (&output))
		//	THROW_EXCEPTION("input point to output");

		PTR(PcIndex) filter2 = GenFilter(input, filter);

		output.clear();

		PRINT_INFO(name + " - Start");

		if (ImplementCheck(searchSurface, input, filter, output))
			ImplementProcess(searchSurface, input, filter2, output);
		else
			THROW_EXCEPTION(name + " - Not pass check");

		{
			std::stringstream ss;
			ss << name << " - End - inSize: " << filter->size() << ", outSize: " << OutputSize(output);
			PRINT_INFO(ss.str());
		}

		//
		output.is_dense = true;
		for (Pc<OutputPointType>::iterator it = output.begin(); it != output.end(); ++it)
		{
			if (!OutPointValid(*it))
			{
				output.is_dense = false;
				break;
			}
		}
	}

	template<class SearchPointType, class InputPointType, class OutputPointType>
	void ProcessorPc2PcInOut<SearchPointType, InputPointType, OutputPointType>::Process(
		const CONST_PTR(Acc<SearchPointType>)& searchSurface,
		const CONST_PTR(Pc<InputPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<OutputPointType>& output) const
	{
		//if ((&(*input)) == (&output))
		//	THROW_EXCEPTION("input point to output");

		PTR(PcIndex) filter2 = GenFilter(input, filter);

		if (output.size() != filter2->size())
			output.resize(filter2->size());

		PRINT_INFO(name + " - Start");

		if (ImplementCheck(searchSurface, input, filter, output))
			ImplementProcess(searchSurface, input, filter2, output);
		else
			THROW_EXCEPTION(name + " - Not pass check");

		{
			std::stringstream ss;
			ss << name << " - End - inSize: " << filter->size() << ", outSize: " << OutputSize(output);
			PRINT_INFO(ss.str());
		}

		//
		output.is_dense = true;
		for (Pc<OutputPointType>::iterator it = output.begin(); it != output.end(); ++it)
		{
			if (!OutPointValid(*it))
			{
				output.is_dense = false;
				break;
			}
		}
	}

	template<class SearchPointType, class InputPointType, class OutputPointType>
	void ProcessorPc2PcInOut<SearchPointType, InputPointType, OutputPointType>::ProcessInOut(
		const CONST_PTR(Acc<SearchPointType>)& searchSurface,
		const PTR(Pc<InputPointType>)& inOut,
		const CONST_PTR(PcIndex)& filter) const
	{
		PTR(PcIndex) filter2 = GenFilter(inOut, filter);

		Pc<InputPointType> temp1;
		Pc<OutputPointType> temp2;

		pcl::ExtractIndices<InputPointType> extract;
		extract.setInputCloud(inOut);
		extract.setIndices(filter2);
		extract.setNegative(false);
		extract.filter(temp1);

		temp2.resize(temp1.size());
		for (std::size_t px = 0; px < temp1.size(); ++px)
			temp2[px] = temp1[px];

		PRINT_INFO(name + " - Start");

		if (ImplementCheck(searchSurface, inOut, filter, temp2))
			ImplementProcess(searchSurface, inOut, filter2, temp2);
		else
			THROW_EXCEPTION(name + " - Not pass check");

		if (filter2->size() != temp2.size())
			THROW_EXCEPTION(name + " - INOUT size not match");

		{
			std::stringstream ss;
			ss << name << " - End - inSize: " << filter->size() << ", outSize: " << OutputSize(temp2);
			PRINT_INFO(ss.str());
		}

		for (std::size_t idx = 0; idx < filter2->size(); ++idx)
		{
			(*inOut)[(*filter2)[idx]] = temp2[idx];

			if (!OutPointValid((*inOut)[(*filter2)[idx]]))
				inOut->is_dense = false;
		}
	}

	template<class InputPointType, class OutputType>
	bool SearchAnySurfaceProcessorPc<InputPointType, OutputType>::ImplementCheck(
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
	bool SearchAnySurfaceProcessorPc2Pc<InputPointType, OutputPointType>::ImplementCheck(
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

	template<class InputPointType, class OutputPointType>
	bool SearchAnySurfaceProcessorPc2PcInOut<InputPointType, OutputPointType>::ImplementCheck(
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
	bool SearchInputSurfaceProcessorPc<InputPointType, OutputType>::ImplementCheck(
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
			if (filter)
			{
				if (searchSurface->getIndices()->size() != filter->size()) // In general case, make sure the tree searches the surface
				{
					THROW_EXCEPTION("searchSurface is not valid, filter not match");
					return false;
				}
				else
				{
					PcIndex temp = (*searchSurface->getIndices());
					std::sort(temp.begin(), temp.end());
					for (std::size_t idx = 0; idx < filter->size(); ++idx)
					{
						if ((*filter)[idx] != temp[idx])
						{
							THROW_EXCEPTION("searchSurface is not valid, filter not match");
							return false;
						}
					}
				}
			}
			else
			{
				THROW_EXCEPTION("searchSurface is not valid, filter not match");
				return false;
			}
		}
		else
		{
			if (filter)
			{
				if (filter->size() != input->size())
				{
					THROW_EXCEPTION("searchSurface is not valid, filter not match");
					return false;
				}
				else
				{
					for (int idx = 0; idx < filter->size(); ++idx)
					{
						if ((*filter)[idx] != idx)
						{
							THROW_EXCEPTION("searchSurface is not valid, filter not match");
							return false;
						}
					}
				}
			}
		}

		return true;
	}

	template<class InputPointType, class OutputPointType>
	bool SearchInputSurfaceProcessorPc2Pc<InputPointType, OutputPointType>::ImplementCheck(
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
			if (filter)
			{
				if (searchSurface->getIndices()->size() != filter->size()) // In general case, make sure the tree searches the surface
				{
					THROW_EXCEPTION("searchSurface is not valid, filter not match");
					return false;
				}
				else
				{
					PcIndex temp = (*searchSurface->getIndices());
					std::sort(temp.begin(), temp.end());
					for (std::size_t idx = 0; idx < filter->size(); ++idx)
					{
						if ((*filter)[idx] != temp[idx])
						{
							THROW_EXCEPTION("searchSurface is not valid, filter not match");
							return false;
						}
					}
				}
			}
			else
			{
				THROW_EXCEPTION("searchSurface is not valid, filter not match");
				return false;
			}
		}
		else
		{
			if (filter)
			{
				if (filter->size() != input->size())
				{
					THROW_EXCEPTION("searchSurface is not valid, filter not match");
					return false;
				}
				else
				{
					for (int idx = 0; idx < filter->size(); ++idx)
					{
						if ((*filter)[idx] != idx)
						{
							THROW_EXCEPTION("searchSurface is not valid, filter not match");
							return false;
						}
					}
				}
			}
		}

		return true;
	}

	template<class InputPointType, class OutputPointType>
	bool SearchInputSurfaceProcessorPc2PcInOut<InputPointType, OutputPointType>::ImplementCheck(
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
			if (filter)
			{
				if (searchSurface->getIndices()->size() != filter->size()) // In general case, make sure the tree searches the surface
				{
					THROW_EXCEPTION("searchSurface is not valid, filter not match");
					return false;
				}
				else
				{
					PcIndex temp = (*searchSurface->getIndices());
					std::sort(temp.begin(), temp.end());
					for (std::size_t idx = 0; idx < filter->size(); ++idx)
					{
						if ((*filter)[idx] != temp[idx])
						{
							THROW_EXCEPTION("searchSurface is not valid, filter not match");
							return false;
						}
					}
				}
			}
			else
			{
				THROW_EXCEPTION("searchSurface is not valid, filter not match");
				return false;
			}
		}
		else
		{
			if (filter)
			{
				if (filter->size() != input->size())
				{
					THROW_EXCEPTION("searchSurface is not valid, filter not match");
					return false;
				}
				else
				{
					for (int idx = 0; idx < filter->size(); ++idx)
					{
						if ((*filter)[idx] != idx)
						{
							THROW_EXCEPTION("searchSurface is not valid, filter not match");
							return false;
						}
					}
				}
			}
		}

		return true;
	}
}