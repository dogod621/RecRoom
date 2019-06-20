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

	template<class GlobalT, class QueryT, class DataT, std::size_t size = 1>
	class AsyncProcess
	{
	public:
		using Self = AsyncProcess<GlobalT, QueryT, DataT, size>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

		typedef int(*AStep)(GlobalT&, QueryT&, DataT&);
		typedef int(*BStep)(GlobalT&, QueryT&, DataT&);
		typedef int(*CStep)(GlobalT&, QueryT&, DataT&);

	public:
		static int Default_AStep(GlobalT&, QueryT&, DataT&) { return 0; }
		static int Default_BStep(GlobalT&, QueryT&, DataT&) { return 0; }
		static int Default_CStep(GlobalT&, QueryT&, DataT&) { return 0; }

		static int WarpAStep(AStep a_Callback, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData);

		static int WarpBStep(BStep b_Callback, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData);

		static int WarpCStep(CStep c_Callback, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData);

		struct BufferT
		{
			PTR(DataT) data;
			std::future<int>* a_future;
			std::future<int>* b_future;
			int a_status;
			int b_status;
			BufferT() : data(PTR(DataT)(new DataT)), a_future(nullptr), b_future(nullptr), a_status(-1), b_status(-1) {}
		};

	public:
		AsyncProcess(GlobalT& globalData, std::vector<QueryT>& queries, 
			AStep a = AsyncProcess::Default_AStep, 
			BStep b = AsyncProcess::Default_BStep,
			CStep c = AsyncProcess::Default_CStep);

	protected:
		std::vector<std::vector<BufferT>> doubleBuffer;
		bool a_bufferPointer;
	};
}

#include "AsyncProcess.hpp"