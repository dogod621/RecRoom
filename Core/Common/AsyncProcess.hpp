#pragma once

#include "AsyncProcess.h"

namespace RecRoom
{
	template<class GlobalT, class QueryT, class DataT, std::size_t size>
	int AsyncProcess<GlobalT, QueryT, DataT, size>::WarpAStep(AStep a, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData)
	{
		PRINT_INFO("Async A Step - " + query->Info(*globalData));

		//
		if (!globalData) return -2;
		if (!query) return -3;
		if (!bufferData) return -4;

		//
		int cerr = 0; 
		cerr = globalData->Check();
		if (cerr != 0) return cerr;
		cerr = query->Check(*globalData);
		if (cerr != 0) return cerr;

		//
		return a(*globalData, *query, *(*bufferData));
	}

	template<class GlobalT, class QueryT, class DataT, std::size_t size>
	int AsyncProcess<GlobalT, QueryT, DataT, size>::WarpBStep(BStep b, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData)
	{
		PRINT_INFO("Async B Step - " + query->Info(*globalData));

		//
		if (!globalData) return -2;
		if (!query) return -3;
		if (!bufferData) return -4;

		//
		int cerr = 0;
		cerr = globalData->Check();
		if (cerr != 0) return cerr;
		cerr = query->Check(*globalData);
		if (cerr != 0) return cerr;

		//
		return b(*globalData, *query, *(*bufferData));
	}

	template<class GlobalT, class QueryT, class DataT, std::size_t size>
	int AsyncProcess<GlobalT, QueryT, DataT, size>::WarpCStep(CStep c, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData)
	{
		PRINT_INFO("Async C Step - " + query->Info(*globalData));

		//
		if (!globalData) return -2;
		if (!query) return -3;
		if (!bufferData) return -4;

		//
		int cerr = 0;
		cerr = globalData->Check();
		if (cerr != 0) return cerr;
		cerr = query->Check(*globalData);
		if (cerr != 0) return cerr;

		//
		return c(*globalData, *query, *(*bufferData));
	}

	template<class GlobalT, class QueryT, class DataT, std::size_t size>
	AsyncProcess<GlobalT, QueryT, DataT, size>::AsyncProcess(GlobalT& globalData, std::vector<QueryT>& queries, AStep a, BStep b, CStep c)
		: doubleBuffer(2), a_bufferPointer(false)
	{
		doubleBuffer[a_bufferPointer].resize(size);
		doubleBuffer[!a_bufferPointer].resize(size);

		std::size_t a_gIdx = 0;
		std::size_t b_gIdx = 0;
		{
			std::size_t a_perGIdx = a_gIdx;
			std::size_t b_perGIdx = b_gIdx;
			
			std::vector<BufferT>& a_buffer = doubleBuffer[a_bufferPointer];

			// Send queries
			for (std::size_t a_locIdx = 0; (a_locIdx < size) && (a_gIdx < queries.size()); ++a_locIdx)
			{
				a_buffer[a_locIdx].a_future = &std::async(AsyncProcess::WarpAStep, a, &globalData, &queries[a_gIdx], &a_buffer[a_locIdx].data);
				a_gIdx++;
			}

			// Wait done
			for (std::size_t a_locIdx = 0; a_locIdx < (std::size_t)((int)a_gIdx - (int)a_perGIdx); ++a_locIdx)
			{
				a_buffer[a_locIdx].a_status = a_buffer[a_locIdx].a_future->get();
				if (a_buffer[a_locIdx].a_status != 0)
					THROW_EXCEPTION("A failed: " + std::to_string(a_buffer[a_locIdx].a_status));
			}
		}

		for (; b_gIdx < queries.size();)
		{
			std::size_t a_perGIdx = a_gIdx;
			std::size_t b_perGIdx = b_gIdx;

			std::vector<BufferT>& a_buffer = doubleBuffer[a_bufferPointer];
			std::vector<BufferT>& b_buffer = doubleBuffer[!a_bufferPointer];

			// Send queries
			for (std::size_t b_locIdx = 0; (b_locIdx < size) && (a_gIdx < queries.size()); ++b_locIdx)
			{
				if (b_buffer[b_locIdx].a_status == 0)
				{
					b_buffer[b_locIdx].b_future = &std::async(AsyncProcess::WarpBStep, b, &globalData, &queries[b_gIdx], &b_buffer[b_locIdx].data);
					b_gIdx++;
				}
			}
			for (std::size_t a_locIdx = 0; (a_locIdx < size) && (a_gIdx < queries.size()); ++a_locIdx)
			{
				a_buffer[a_locIdx].a_future = &std::async(AsyncProcess::WarpAStep, a, &globalData, &queries[a_gIdx], &a_buffer[a_locIdx].data);
				a_gIdx++;
			}

			// Wait done
			for (std::size_t b_locIdx = 0; b_locIdx < (std::size_t)((int)b_gIdx - (int)b_perGIdx); ++b_locIdx)
			{
				if (b_buffer[b_locIdx].a_status == 0)
				{
					b_buffer[b_locIdx].b_status = b_buffer[b_locIdx].b_future->get();
					if (b_buffer[b_locIdx].b_status != 0)
						THROW_EXCEPTION("B failed: " + std::to_string(b_buffer[b_locIdx].b_status));

					int c_status = WarpCStep(c, &globalData, &queries[b_perGIdx + b_locIdx], &b_buffer[b_locIdx].data);
					if(c_status != 0)
						THROW_EXCEPTION("C failed: " + std::to_string(c_status));
				}
			}

			for (std::size_t a_locIdx = 0; a_locIdx < (std::size_t)((int)a_gIdx - (int)a_perGIdx); ++a_locIdx)
			{
				a_buffer[a_locIdx].a_status = a_buffer[a_locIdx].a_future->get();
				if (a_buffer[a_locIdx].a_status != 0)
					THROW_EXCEPTION("A failed: " + std::to_string(a_buffer[a_locIdx].a_status));
			}

			//
			a_bufferPointer = !a_bufferPointer;
		}
	}
}