#pragma once

#include "AsyncProcess.h"

namespace RecRoom
{
	template<class GlobalT, class QueryT, class DataT>
	int AsyncProcess<GlobalT, QueryT, DataT>::WarpAStep(AStep a, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData)
	{
		PRINT_INFO("Async A Step - Start - " + query->Info(*globalData));

		//
		(*bufferData) = PTR(DataT)(new DataT);
		int r = 0;

		//
		if (!globalData) r = -1;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		if (!query) r = -2;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		if (!bufferData) r = -3;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		r = globalData->Check();
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		r = query->Check(*globalData);
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		r = a(*globalData, *query, *(*bufferData));
		PRINT_INFO("Async A Step - End - " + std::to_string(r));
		return r;
	}

	template<class GlobalT, class QueryT, class DataT>
	int AsyncProcess<GlobalT, QueryT, DataT>::WarpBStep(BStep b, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData)
	{
		PRINT_INFO("Async B Step - Start - " + query->Info(*globalData));

		//
		int r = 0;

		//
		if (!globalData) r = -1;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		if (!query) r = -2;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		if (!bufferData) r = -3;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		if (!(*bufferData)) r = -4;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		r = globalData->Check();
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		r = query->Check(*globalData);
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		r = b(*globalData, *query, *(*bufferData));
		PRINT_INFO("Async B Step - End - " + std::to_string(r));
		return r;
	}

	template<class GlobalT, class QueryT, class DataT>
	int AsyncProcess<GlobalT, QueryT, DataT>::WarpCStep(CStep c, GlobalT* globalData, QueryT* query, PTR(DataT)* bufferData)
	{
		PRINT_INFO("Async C Step - Start - " + query->Info(*globalData));

		//
		int r = 0;

		//
		if (!globalData) r = -1;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		if (!query) r = -2;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		if (!bufferData) r = -3;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		if (!(*bufferData)) r = -4;
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		r = globalData->Check();
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		r = query->Check(*globalData);
		if (r != 0) { PRINT_INFO("Async A Step - End - " + std::to_string(r)); return r; }

		//
		r = c(*globalData, *query, *(*bufferData));
		PRINT_INFO("Async C Step - End - " + std::to_string(r));
		return r;
	}

	template<class GlobalT, class QueryT, class DataT>
	AsyncProcess<GlobalT, QueryT, DataT>::AsyncProcess(GlobalT& globalData, std::vector<QueryT>& queries, AStep a, BStep b, CStep c, std::size_t size_)
		: doubleBuffer(2), a_bufferPointer(false), size(size_)
	{
		doubleBuffer[a_bufferPointer].resize(size);
		doubleBuffer[!a_bufferPointer].resize(size);

		std::size_t a_gIdx = 0;
		std::size_t b_gIdx = 0;
		{
			std::size_t a_perGIdx = a_gIdx;
			std::size_t b_perGIdx = b_gIdx;

			std::vector<BufferT>& a_buffer = doubleBuffer[!a_bufferPointer];

			for (std::size_t a_locIdx = 0; (a_locIdx < size) && (a_gIdx < queries.size()); ++a_locIdx)
			{
				int a_status = WarpAStep(a, &globalData, &queries[a_gIdx], &a_buffer[a_locIdx].data);
				if (a_status != 0)
					THROW_EXCEPTION("A failed: " + std::to_string(a_status));
				a_gIdx++;
			}

			// Send queries
			/**
			for (std::size_t a_locIdx = 0; (a_locIdx < size) && (a_gIdx < queries.size()); ++a_locIdx)
			{
				a_buffer[a_locIdx].a_future = &std::async(std::launch::async, AsyncProcess::WarpAStep, a, &globalData, &queries[a_gIdx], &a_buffer[a_locIdx].data);
				a_gIdx++;
			}

			// Wait done
			a_gIdx = a_perGIdx;
			b_gIdx = b_perGIdx;
			for (std::size_t a_locIdx = 0; (a_locIdx < size) && (a_gIdx < queries.size()); ++a_locIdx)
			{
				if(a_buffer[a_locIdx].a_future == nullptr)
					THROW_EXCEPTION("a_future is not set");

				int a_status = a_buffer[a_locIdx].a_future->get();
				a_buffer[a_locIdx].a_future = nullptr;
				if (a_status != 0)
					THROW_EXCEPTION("A failed: " + std::to_string(a_status));
				a_gIdx++;
			}*/
		}

		for (; b_gIdx < queries.size();)
		{
			std::size_t a_perGIdx = a_gIdx;
			std::size_t b_perGIdx = b_gIdx;

			std::vector<BufferT>& a_buffer = doubleBuffer[a_bufferPointer];
			std::vector<BufferT>& b_buffer = doubleBuffer[!a_bufferPointer];

			// Send queries
			for (std::size_t b_locIdx = 0; (b_locIdx < size) && (b_gIdx < queries.size()); ++b_locIdx)
			{
				b_buffer[b_locIdx].b_future = &std::async(std::launch::async, AsyncProcess::WarpBStep, b, &globalData, &queries[b_gIdx], &b_buffer[b_locIdx].data);
				b_gIdx++;
			}
			for (std::size_t a_locIdx = 0; (a_locIdx < size) && (a_gIdx < queries.size()); ++a_locIdx)
			{
				a_buffer[a_locIdx].a_future = &std::async(std::launch::async, AsyncProcess::WarpAStep, a, &globalData, &queries[a_gIdx], &a_buffer[a_locIdx].data);
				a_gIdx++;
			}

			// Wait done
			a_gIdx = a_perGIdx;
			b_gIdx = b_perGIdx;
			for (std::size_t b_locIdx = 0; (b_locIdx < size) && (b_gIdx < queries.size()); ++b_locIdx)
			{
				if (b_buffer[b_locIdx].b_future == nullptr)
					THROW_EXCEPTION("b_future is not set");

				int b_status = b_buffer[b_locIdx].b_future->get();
				b_buffer[b_locIdx].b_future = nullptr;
				if (b_status != 0)
					THROW_EXCEPTION("B failed: " + std::to_string(b_status));

				int c_status = WarpCStep(c, &globalData, &queries[b_perGIdx + b_locIdx], &b_buffer[b_locIdx].data);
				if (c_status != 0)
					THROW_EXCEPTION("C failed: " + std::to_string(c_status));

				b_gIdx++;
			}
			for (std::size_t a_locIdx = 0; (a_locIdx < size) && (a_gIdx < queries.size()); ++a_locIdx)
			{
				if (a_buffer[a_locIdx].a_future == nullptr)
					THROW_EXCEPTION("a_future is not set");

				int a_status = a_buffer[a_locIdx].a_future->get();
				a_buffer[a_locIdx].a_future = nullptr;
				if (a_status != 0)
					THROW_EXCEPTION("A failed: " + std::to_string(a_status));
				a_gIdx++;
			}

			//
			a_bufferPointer = !a_bufferPointer;
		}
	}
}