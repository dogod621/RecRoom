#pragma once

#include <future>

#include "Common.h"

namespace RecRoom
{
	template<class GlobalT, class QueryT, class DataT, std::size_t size = 1>
	class AsyncProcess
	{
	public:
		using Self = AsyncProcess<GlobalT, QueryT, DataT, size>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

		typedef int(*A_Callback)(GlobalT&, QueryT&, DataT&);
		typedef int(*B_Callback)(GlobalT&, QueryT&, DataT&);
		typedef int(*C_Callback)(GlobalT&, DataT&);

	public:
		static int Default_A(GlobalT&, QueryT&, DataT&) { return 0; }
		static int Default_B(GlobalT&, QueryT&, DataT&) { return 0; }
		static int Default_C(GlobalT&, DataT&) { return 0; }

		static int A(A_Callback a_Callback, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData);

		static int B(B_Callback b_Callback, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData);

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
		AsyncProcess(GlobalT& globalData, std::vector<QueryT>& queries, A_Callback a_Callback = AsyncProcess::Default_A, B_Callback b_Callback = AsyncProcess::Default_B, C_Callback c_Callback = AsyncProcess::Default_C);

	protected:
		std::vector<std::vector<BufferT>> doubleBuffer;
		bool a_bufferPointer;
	};
}

#include "AsyncProcess.hpp"