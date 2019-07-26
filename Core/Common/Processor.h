#pragma once

#include "Common.h"
#include "Point.h"

namespace RecRoom
{
	template<class SearchPointType, class InputPointType, class OutputType>
	class ProcessorPc
	{
	public:
		ProcessorPc() {}

		void Process(
			const CONST_PTR(Acc<SearchPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			OutputType& output) const
		{
			if (ImplementCheck(searchSurface, input, filter, output))
				ImplementProcess(searchSurface, input, filter, output);
			else
				THROW_EXCEPTION("Not pass check");
		}

		/*void Process(
			const CONST_PTR(Pc<SearchPointType>)& searchSurface,
			const CONST_PTR(AccVNN)& searchVNN,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			OutputType& output) const
		{
			if (ImplementCheck(searchSurface, searchVNN, input, filter, output))
				ImplementProcess(searchSurface, searchVNN, input, filter, output);
			else
				THROW_EXCEPTION("Not pass check");
		}*/

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

		/*virtual bool ImplementCheck(
			const CONST_PTR(Pc<SearchPointType>)& searchSurface,
			const CONST_PTR(AccVNN)& searchVNN,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			OutputType& output) const = 0;

		virtual void ImplementProcess(
			const CONST_PTR(Pc<SearchPointType>)& searchSurface,
			const CONST_PTR(AccVNN)& searchVNN,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			OutputType& output) const = 0;*/
	};

	template<class SearchPointType, class InputPointType, class OutputPointType>
	class ProcessorPc2Pc : public ProcessorPc<SearchPointType, InputPointType, Pc<OutputPointType>>
	{
	public:
		ProcessorPc2Pc() : ProcessorPc<SearchPointType, InputPointType, Pc<OutputPointType>>() {}

		void ProcessInOut(
			const CONST_PTR(Acc<SearchPointType>)& searchSurface,
			const PTR(Pc<InputPointType>)& inOut,
			const CONST_PTR(PcIndex)& filter) const;
	};

	template<class InputPointType, class OutputType>
	class SearchInputTypeProcessorPc : public ProcessorPc<InputPointType, InputPointType, OutputType>
	{
	public:
		SearchInputTypeProcessorPc() : ProcessorPc<InputPointType, InputPointType, OutputType>() {}
	};

	template<class InputPointType, class OutputPointType>
	class SearchInputTypeProcessorPc2Pc : public ProcessorPc2Pc<InputPointType, InputPointType, OutputPointType>
	{
	public:
		SearchInputTypeProcessorPc2Pc() : ProcessorPc2Pc<InputPointType, InputPointType, OutputPointType>() {}
	};

	template<class InputPointType, class OutputType>
	class SearchAnySurfaceProcessorPc : public SearchInputTypeProcessorPc<InputPointType, OutputType>
	{
	public:
		SearchAnySurfaceProcessorPc() : SearchInputTypeProcessorPc<InputPointType, OutputType>() {}

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
		SearchAnySurfaceProcessorPc2Pc() : SearchInputTypeProcessorPc2Pc<InputPointType, OutputPointType>() {}

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
		SearchInputSurfaceProcessorPc() : SearchInputTypeProcessorPc<InputPointType, OutputType>() {}

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
		SearchInputSurfaceProcessorPc2Pc() : SearchInputTypeProcessorPc2Pc<InputPointType, OutputPointType>() {}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<InputPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutputPointType>& output) const;
	};
}

#include "Processor.hpp"