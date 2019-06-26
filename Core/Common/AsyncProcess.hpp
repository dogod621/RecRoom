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
			std::vector<BufferT>& a_buffer = doubleBuffer[!a_bufferPointer];
			for (std::size_t a_locIdx = 0; (a_locIdx < size) && (a_gIdx < queries.size()); ++a_locIdx)
			{
				PRINT_INFO("Launch sync A - Start - ID: " + std::to_string(a_gIdx) + "/" + std::to_string(queries.size()));
				{
					int a_status = WarpAStep(a, &globalData, &queries[a_gIdx], &a_buffer[a_locIdx].data);
					if (a_status != 0)
						THROW_EXCEPTION("A failed: " + std::to_string(a_status));
				}
				PRINT_INFO("Launch sync A - End - ID: " + std::to_string(a_gIdx) + "/" + std::to_string(queries.size()));

				a_gIdx++;
			}
		}

		for (; b_gIdx < queries.size();)
		{
			// Send queries
			std::size_t b_perGIdx = b_gIdx;
			std::vector<BufferT>& b_buffer = doubleBuffer[!a_bufferPointer];
			for (std::size_t b_locIdx = 0; (b_locIdx < size) && (b_gIdx < queries.size()); ++b_locIdx)
			{
				PRINT_INFO("Launch async B - Start - ID: " + std::to_string(b_gIdx) + "/" + std::to_string(queries.size()));
				{
					b_buffer[b_locIdx].b_future = std::async(std::launch::async, AsyncProcess::WarpBStep, b, &globalData, &queries[b_gIdx], &b_buffer[b_locIdx].data);
				}
				PRINT_INFO("Launch async B - End - ID: " + std::to_string(b_gIdx) + "/" + std::to_string(queries.size()));
				b_gIdx++;
			}

			std::size_t a_perGIdx = a_gIdx;
			std::vector<BufferT>& a_buffer = doubleBuffer[a_bufferPointer];
			for (std::size_t a_locIdx = 0; (a_locIdx < size) && (a_gIdx < queries.size()); ++a_locIdx)
			{
				PRINT_INFO("Launch async A - Start - ID: " + std::to_string(a_gIdx) + "/" + std::to_string(queries.size()));
				{
					a_buffer[a_locIdx].a_future = std::async(std::launch::async, AsyncProcess::WarpAStep, a, &globalData, &queries[a_gIdx], &a_buffer[a_locIdx].data);
				}
				PRINT_INFO("Launch async A - End - ID: " + std::to_string(a_gIdx) + "/" + std::to_string(queries.size()));
				a_gIdx++;
			}

			// Wait done
			b_gIdx = b_perGIdx;
			for (std::size_t b_locIdx = 0; (b_locIdx < size) && (b_gIdx < queries.size()); ++b_locIdx)
			{
				//
				PRINT_INFO("Get async B - Start - ID: " + std::to_string(b_gIdx) + "/" + std::to_string(queries.size()));
				{
					int b_status = b_buffer[b_locIdx].b_future.get();
					if (b_status != 0)
						THROW_EXCEPTION("B failed: " + std::to_string(b_status));
				}
				PRINT_INFO("Get async B - End - ID: " + std::to_string(b_gIdx) + "/" + std::to_string(queries.size()));


				//
				PRINT_INFO("Launch sync C - Start - ID: " + std::to_string(b_gIdx) + "/" + std::to_string(queries.size()));
				{
					int c_status = WarpCStep(c, &globalData, &queries[b_perGIdx + b_locIdx], &b_buffer[b_locIdx].data);
					if (c_status != 0)
						THROW_EXCEPTION("C failed: " + std::to_string(c_status));
				}
				PRINT_INFO("Launch sync C - End - ID: " + std::to_string(b_gIdx) + "/" + std::to_string(queries.size()));

				//
				b_gIdx++;
			}

			a_gIdx = a_perGIdx;
			for (std::size_t a_locIdx = 0; (a_locIdx < size) && (a_gIdx < queries.size()); ++a_locIdx)
			{
				PRINT_INFO("Get async A - Start - ID: " + std::to_string(a_gIdx) + "/" + std::to_string(queries.size()));
				{
					int a_status = a_buffer[a_locIdx].a_future.get();
					if (a_status != 0)
						THROW_EXCEPTION("A failed: " + std::to_string(a_status));
				}
				PRINT_INFO("Get async A - End - ID: " + std::to_string(a_gIdx) + "/" + std::to_string(queries.size()));

				a_gIdx++;
			}

			//
			a_bufferPointer = !a_bufferPointer;
		}
	}
}