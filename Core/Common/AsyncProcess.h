#pragma once

#include <future>

#include "Common.h"

namespace RecRoom
{
	class AsyncGlobal
	{
	public:
		virtual int Check() const = 0;
	};

	template<class GlobalT>
	class AsyncQuery
	{
	public:
		virtual int Check(const GlobalT&) const = 0;
		virtual std::string Info(const GlobalT&) const = 0;
	};

	template<class GlobalT, class QueryT, class DataT>
	class AsyncProcess
	{
	public:
		typedef int(*AStep)(const GlobalT&, const QueryT&, DataT&);
		typedef int(*BStep)(const GlobalT&, const QueryT&, DataT&);
		typedef int(*CStep)(GlobalT&, const QueryT&, const DataT&);

	public:
		static int Default_AStep(const GlobalT&, const QueryT&, DataT&) { return 0; }
		static int Default_BStep(const GlobalT&, const QueryT&, DataT&) { return 0; }
		static int Default_CStep(GlobalT&, const QueryT&, DataT&) { return 0; }

		static int WarpAStep(AStep a_Callback, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData);

		static int WarpBStep(BStep b_Callback, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData);

		static int WarpCStep(CStep c_Callback, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData);

		struct BufferT
		{
			PTR(DataT) data;
			std::future<int> a_future;
			std::future<int> b_future;
			BufferT() : data(PTR(DataT)(new DataT)), a_future(), b_future() {}
		};

	public:
		AsyncProcess(GlobalT& globalData, std::vector<QueryT>& queries, 
			AStep a = AsyncProcess::Default_AStep, 
			BStep b = AsyncProcess::Default_BStep,
			CStep c = AsyncProcess::Default_CStep,
			std::size_t size = 1 );

	protected:
		std::vector<std::vector<BufferT>> doubleBuffer;
		bool a_bufferPointer;
		std::size_t size;
	};
}

#include "AsyncProcess.hpp"