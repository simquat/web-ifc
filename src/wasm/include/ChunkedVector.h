#pragma once

#include <vector>

const int CHUNK_SIZE = 8 * 1024; // 8kb

template <typename T>
class Chunk
{
public:
	T* ptr;
	int index;
};

template <typename T>
class MemoryPool
{
public:

	~MemoryPool()
	{
		for (auto& c : chunks)
		{
			delete[] c.ptr;
		}

		chunks.clear();
	}

	inline Chunk<T> GetFreeChunk()
	{
		int freeIndex = -1;
		if (free.empty())
		{
			Chunk<T> c;
			c.index = chunks.size();
			c.ptr = new T[CHUNK_SIZE];
			chunks.push_back(c);

			freeIndex = chunks.size() - 1;
		}
		else
		{
			freeIndex = free.back();
			free.pop_back();
		}

		return chunks[freeIndex];
	}

	inline void Free(Chunk<T> t)
	{
		free.push_back(t.index);
	}

private:
	std::vector<Chunk<T>> chunks;
	std::vector<int> free;
};

template <typename T>
class PoolBackedChunkedVector
{
public:
	PoolBackedChunkedVector(MemoryPool<T>& p) :
		_pool(&p)
	{

	}

	~PoolBackedChunkedVector()
	{
		for (auto& c : _chunks)
		{
			_pool->Free(c);
		}
	}

	inline uint32_t size()
	{
		return _count;
	}

	inline T operator[](uint32_t index) const
	{
		uint32_t chunk = index / CHUNK_SIZE;
		uint32_t indexInChunk = index % CHUNK_SIZE;

		return _chunks[chunk].ptr[indexInChunk];
	}

	inline T& operator[](uint32_t index)
	{
		uint32_t chunk = index / CHUNK_SIZE;
		uint32_t indexInChunk = index % CHUNK_SIZE;

		return _chunks[chunk].ptr[indexInChunk];
	}

	inline void Set(uint32_t index, T val)
	{
		uint32_t chunk = index / CHUNK_SIZE;
		uint32_t indexInChunk = index % CHUNK_SIZE;

		if (chunk >= _chunks.size())
		{
			Grow();
		}

		_chunks[chunk].ptr[indexInChunk] = val;
	}

	inline void Push(T val)
	{
		Set(_count++, val);
	}

	inline void Grow()
	{
		_chunks.push_back(_pool->GetFreeChunk());
	}

private:
	MemoryPool<T>* _pool;
	std::vector<Chunk<T>> _chunks;
	uint32_t _count = 0;
};