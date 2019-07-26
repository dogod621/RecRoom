#pragma once

#include "Common.h"
#include "Point.h"

namespace RecRoom
{
	template<class SearchPointType, class InputPointType, class OutputType>
	class ProcessorPc
	{
	public:
		ProcessorPc() 
		{
			name = "ProcessorPc";
		}

		void Process(
			const CONST_PTR(Acc<SearchPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			OutputType& output) const
		{
			//if ((&(*input)) == (&output))
			//	THROW_EXCEPTION("input point to output");

			PRINT_INFO(name + " - Start");

			if (ImplementCheck(searchSurface, input, filter, output))
				ImplementProcess(searchSurface, input, filter, output);
			else
				THROW_EXCEPTION(name + " - Not pass check");

			if (filter)
			{
				std::stringstream ss;
				ss << name << " - End - inSize: " << filter->size() << ", outSize: " << OutputSize(output);
				PRINT_INFO(ss.str());
			}
			else
			{
				std::stringstream ss;
				ss << name << " - End - inSize: " << input->size() << ", outSize: " << OutputSize(output);
				PRINT_INFO(ss.str());
			}
		}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<SearchPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			OutputType& output) const = 0;

		virtual void ImplementProcess(
			const CONST_PTR(Acc<SearchPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			OutputType& output) const = 0;

		virtual int OutputSize(OutputType& output) const
		{
			return 0;
		}

	protected:
		std::string name;
	};

	template<class SearchPointType, class InputPointType, class OutputPointType>
	class ProcessorPc2Pc : public ProcessorPc<SearchPointType, InputPointType, Pc<OutputPointType>>
	{
	public:
		ProcessorPc2Pc() : ProcessorPc<SearchPointType, InputPointType, Pc<OutputPointType>>() 
		{
			name = "ProcessorPc2Pc";
		}

		void ProcessInOut(
			const CONST_PTR(Acc<SearchPointType>)& searchSurface,
			const PTR(Pc<InputPointType>)& inOut,
			const CONST_PTR(PcIndex)& filter) const;

	protected:
		virtual int OutputSize(Pc<OutputPointType>& output) const
		{
			return output.size();
		}
	};

	template<class InputPointType, class OutputType>
	class SearchInputTypeProcessorPc : public ProcessorPc<InputPointType, InputPointType, OutputType>
	{
	public:
		SearchInputTypeProcessorPc() : ProcessorPc<InputPointType, InputPointType, OutputType>()
		{
			name = "SearchInputTypeProcessorPc";
		}
	};

	template<class InputPointType, class OutputPointType>
	class SearchInputTypeProcessorPc2Pc : public ProcessorPc2Pc<InputPointType, InputPointType, OutputPointType>
	{
	public:
		SearchInputTypeProcessorPc2Pc() : ProcessorPc2Pc<InputPointType, InputPointType, OutputPointType>() 
		{
			name = "SearchInputTypeProcessorPc2Pc";
		}
	};

	template<class InputPointType, class OutputType>
	class SearchAnySurfaceProcessorPc : public SearchInputTypeProcessorPc<InputPointType, OutputType>
	{
	public:
		SearchAnySurfaceProcessorPc() : SearchInputTypeProcessorPc<InputPointType, OutputType>() 
		{
			name = "SearchAnySurfaceProcessorPc";
		}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<InputPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			OutputType& output) const;
	};

	template<class InputPointType, class OutputPointType>
	class SearchAnySurfaceProcessorPc2Pc : public SearchInputTypeProcessorPc2Pc<InputPointType, OutputPointType>
	{
	public:
		SearchAnySurfaceProcessorPc2Pc() : SearchInputTypeProcessorPc2Pc<InputPointType, OutputPointType>() 
		{
			name = "SearchAnySurfaceProcessorPc2Pc";
		}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<InputPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutputPointType>& output) const;
	};

	template<class InputPointType, class OutputType>
	class SearchInputSurfaceProcessorPc : public SearchInputTypeProcessorPc<InputPointType, OutputType>
	{
	public:
		SearchInputSurfaceProcessorPc() : SearchInputTypeProcessorPc<InputPointType, OutputType>() 
		{
			name = "SearchInputSurfaceProcessorPc";
		}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<InputPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			OutputType& output) const;
	};

	template<class InputPointType, class OutputPointType>
	class SearchInputSurfaceProcessorPc2Pc : public SearchInputTypeProcessorPc2Pc<InputPointType, OutputPointType>
	{
	public:
		SearchInputSurfaceProcessorPc2Pc() : SearchInputTypeProcessorPc2Pc<InputPointType, OutputPointType>() 
		{
			name = "SearchInputSurfaceProcessorPc2Pc";
		}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<InputPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutputPointType>& output) const;
	};
}

#include "Processor.hpp"