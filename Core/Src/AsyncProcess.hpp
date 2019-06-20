#pragma once

#include "AsyncProcess.h"

namespace RecRoom
{
	template<class GlobalT, class QueryT, class DataT, std::size_t size>
	int AsyncProcess<GlobalT, QueryT, DataT, size>::A(A_Callback a_Callback, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData)
	{
		if (!globalData) return -2;
		if (!query) return -3;
		if (!bufferData) return -4;

		//
		return a_Callback(*globalData, *query, *(*bufferData));
	}

	template<class GlobalT, class QueryT, class DataT, std::size_t size>
	int AsyncProcess<GlobalT, QueryT, DataT, size>::B(B_Callback b_Callback, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData)
	{
		if (!globalData) return -2;
		if (!query) return -3;
		if (!bufferData) return -4;

		//
		int status = b_Callback(*globalData, *query, *(*bufferData));
		return status;
	}

	template<class GlobalT, class QueryT, class DataT, std::size_t size>
	AsyncProcess<GlobalT, QueryT, DataT, size>::AsyncProcess(GlobalT& globalData, std::vector<QueryT>& queries, A_Callback a_Callback, B_Callback b_Callback, C_Callback c_Callback)
		: doubleBuffer(2), a_bufferPointer(false)
	{
		doubleBuffer[a_bufferPointer].resize(size);
		doubleBuffer[!a_bufferPointer].resize(size);

		std::size_t a_index = 0;
		std::size_t b_index = 0;
		{
			std::vector<BufferT>& a_buffer = doubleBuffer[a_bufferPointer];

			// Send queries
			for (std::size_t i = 0; (i < size) && (a_index < queries.size()); ++i)
			{
				a_buffer[i].a_future = &std::async(AsyncProcess::A, a_Callback, &globalData, &queries[a_index], &a_buffer[i].data);
				a_index++;
			}

			// Wait done
			for (std::size_t i = 0; (i < size) && (a_index < queries.size()); ++i)
			{
				a_buffer[i].a_status = a_buffer[i].a_future->get();
				if (a_buffer[i].a_status != 0)
					THROW_EXCEPTION("A failed: " + std::to_string(a_buffer[i].a_status));
			}
		}

		for (;;)
		{
			std::vector<BufferT>& a_buffer = doubleBuffer[a_bufferPointer];
			std::vector<BufferT>& b_buffer = doubleBuffer[!a_bufferPointer];

			// Send queries
			for (std::size_t i = 0; (i < size) && (b_index < queries.size()); ++i)
			{
				if (b_buffer[i].a_status == 0)
				{
					b_buffer[i].b_future = &std::async(AsyncProcess::B, b_Callback, &globalData, &queries[b_index], &b_buffer[i].data);
					b_index++;
				}
			}
			for (std::size_t i = 0; (i < size) && (a_index < queries.size()); ++i)
			{
				a_buffer[i].a_future = &std::async(AsyncProcess::A, a_Callback, &globalData, &queries[a_index], &a_buffer[i].data);
				a_index++;
			}

			// Wait done
			for (std::size_t i = 0; (i < size) && (b_index < queries.size()); ++i)
			{
				if (b_buffer[i].a_status == 0)
				{
					b_buffer[i].b_status = b_buffer[i].b_future->get();
					if (b_buffer[i].b_status != 0)
						THROW_EXCEPTION("B failed: " + std::to_string(b_buffer[i].b_status));

					int c_status = c_Callback(globalData, *b_buffer[i].data);
					if(c_status != 0)
						THROW_EXCEPTION("C failed: " + std::to_string(c_status));
				}
			}

			for (std::size_t i = 0; (i < size) && (a_index < queries.size()); ++i)
			{
				a_buffer[i].a_status = a_buffer[i].a_future->get();
				if (a_buffer[i].a_status != 0)
					THROW_EXCEPTION("A failed: " + std::to_string(a_buffer[i].a_status));
			}

			//
			a_bufferPointer = !a_bufferPointer;
		}
	}
}