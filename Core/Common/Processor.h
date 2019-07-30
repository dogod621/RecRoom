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

		virtual void Process(
			const CONST_PTR(Acc<SearchPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			OutputType& output) const;

	protected:
		PTR(PcIndex) GenFilter(
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter) const;

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

		inline virtual bool SearchPointValid(const SearchPointType& p) const
		{
			return pcl::isFinite(p);
		}

		inline virtual bool InputPointValid(const InputPointType& p) const
		{
			return pcl::isFinite(p);
		}

		inline virtual int OutputSize(OutputType& output) const
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

		virtual void Process(
			const CONST_PTR(Acc<SearchPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutputPointType>& output) const;

	protected:
		inline virtual bool OutPointValid(const OutputPointType& p) const
		{
			return pcl::isFinite(p);
		}

		virtual int OutputSize(Pc<OutputPointType>& output) const
		{
			return output.size();
		}
	};

	template<class SearchPointType, class InputPointType, class OutputPointType>
	class ProcessorPc2PcInOut : public ProcessorPc2Pc<SearchPointType, InputPointType, OutputPointType>
	{
	public:
		ProcessorPc2PcInOut() : ProcessorPc2Pc<SearchPointType, InputPointType, OutputPointType>()
		{
			name = "ProcessorPc2PcInOut";
		}

		virtual void Process(
			const CONST_PTR(Acc<SearchPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutputPointType>& output) const;

		virtual void ProcessInOut(
			const CONST_PTR(Acc<SearchPointType>)& searchSurface,
			const PTR(Pc<InputPointType>)& inOut,
			const CONST_PTR(PcIndex)& filter) const;
	};

	template<class InputPointType, class OutputType>
	class SearchAnySurfaceProcessorPc : public ProcessorPc<InputPointType, InputPointType, OutputType>
	{
	public:
		SearchAnySurfaceProcessorPc() : ProcessorPc<InputPointType, InputPointType, OutputType>()
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
	class SearchAnySurfaceProcessorPc2Pc : public ProcessorPc2Pc<InputPointType, InputPointType, OutputPointType>
	{
	public:
		SearchAnySurfaceProcessorPc2Pc() : ProcessorPc2Pc<InputPointType, InputPointType, OutputPointType>()
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

	template<class InputPointType, class OutputPointType>
	class SearchAnySurfaceProcessorPc2PcInOut : public ProcessorPc2PcInOut<InputPointType, InputPointType, OutputPointType>
	{
	public:
		SearchAnySurfaceProcessorPc2PcInOut() : ProcessorPc2PcInOut<InputPointType, InputPointType, OutputPointType>()
		{
			name = "SearchAnySurfaceProcessorPc2PcInOut";
		}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<InputPointType>)& searchSurface,
			const CONST_PTR(Pc<InputPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutputPointType>& output) const;
	};

	template<class InputPointType, class OutputType>
	class SearchInputSurfaceProcessorPc : public ProcessorPc<InputPointType, InputPointType, OutputType>
	{
	public:
		SearchInputSurfaceProcessorPc() : ProcessorPc<InputPointType, InputPointType, OutputType>()
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
	class SearchInputSurfaceProcessorPc2Pc : public ProcessorPc2Pc<InputPointType, InputPointType, OutputPointType>
	{
	public:
		SearchInputSurfaceProcessorPc2Pc() : ProcessorPc2Pc<InputPointType, InputPointType, OutputPointType>()
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

	template<class InputPointType, class OutputPointType>
	class SearchInputSurfaceProcessorPc2PcInOut : public ProcessorPc2PcInOut<InputPointType, InputPointType, OutputPointType>
	{
	public:
		SearchInputSurfaceProcessorPc2PcInOut() : ProcessorPc2PcInOut<InputPointType, InputPointType, OutputPointType>()
		{
			name = "SearchInputSurfaceProcessorPc2PcInOut";
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