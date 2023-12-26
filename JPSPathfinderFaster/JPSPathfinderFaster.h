#ifdef JPSPATHFINDERFASTER_EXPORTS
#define JPSPATHFINDERFASTER_API __declspec(dllexport)
#else
#define JPSPATHFINDERFASTER_API __declspec(dllimport)
#endif


#include <assert.h>


#define FLOAT_MAX 3.40282347E+38F
#define NOOP


typedef int int32;
typedef unsigned int uint32;
typedef unsigned long long int uint64;
typedef unsigned char uint8;

#define FORCEINLINE __forceinline


enum class PathfindResult : uint8
{
	Found = 0,
	NotFound = 1,
	StartOrEndPointOutOfBound = 2,
	PriorityQueuePoolOverflow = 3,
	PathResultPoolOverflow = 4,
	CloseListPoolOverflow = 5,
};

FORCEINLINE static int32 Abs32i(int32 InVal)
{
	return InVal < 0 ? (InVal * -1) : InVal;
}

FORCEINLINE static int32 Min32i(int32 lhs, int32 rhs)
{
	return lhs < rhs ? lhs : rhs;
}

struct Vector2Int
{
	int32 m_x, m_y;

	Vector2Int(int32 InX, int32 InY):
		m_x(InX), m_y(InY)
	{
		NOOP;
	}


	FORCEINLINE bool operator==(Vector2Int rhs) { return m_x == rhs.m_x && m_y == rhs.m_y; }
	FORCEINLINE bool isValid() const { return (m_x >= 0) && (m_y >= 0); }

	FORCEINLINE static float OctileDistance(const Vector2Int pos1,const Vector2Int pos2)
	{
		constexpr float DIAGONAL_DISTANCE_BASE = 1.4142135623730950f;

		int32 dx = Abs32i( pos1.m_x - pos2.m_x );
		int32 dy = Abs32i( pos1.m_y - pos2.m_y );
		
		return (dx + dy) + (DIAGONAL_DISTANCE_BASE - 2) * Min32i(dx, dy);
	}

	const static Vector2Int InvalidIdx;
};


struct PriorityQueuePair
{
	Vector2Int m_nodeIdx;
	float m_fCost;

	PriorityQueuePair(const Vector2Int InNodeIdx, const float InFCost) :
		m_nodeIdx(InNodeIdx), m_fCost(InFCost)
	{
		NOOP;
	}
};

struct PathFinderNode
{
	float m_gCost = 0;
	bool m_onCloseList = false;
	bool m_isBlock = false;
	Vector2Int m_paretnNode = Vector2Int(-1,-1);
}; 

struct PathfinderPriorityQueue
{
	PriorityQueuePair *m_priorityQueueData = nullptr;
	uint32 m_priorityQueueCapacity = 0;
	uint32 m_priorityQueueSize = 0;

	bool enqueue(const PriorityQueuePair dataToPush);
	Vector2Int dequeue();
	
	FORCEINLINE void clear() { m_priorityQueueSize = 0;  }
	FORCEINLINE bool isEmpty() { return m_priorityQueueSize == 0; }
	FORCEINLINE bool isFull() { return m_priorityQueueSize == m_priorityQueueCapacity; }
};

struct PathfinderClostList
{
	Vector2Int* m_closeListData;
	uint32 m_closeListCapacity;
	uint32 m_closeListSize;

	FORCEINLINE bool push_back(Vector2Int InVector)
	{
		m_closeListData[m_closeListSize++] = InVector; 
		if (m_closeListCapacity <= m_closeListSize) return false;
		return true;
	}

	FORCEINLINE void clear(){ m_closeListSize = 0; }

};

struct OutPathList
{
	Vector2Int* m_outPathListData;
	uint32 m_outPathListCapacity;
	uint32 m_outPathListSize;

	FORCEINLINE bool push_back(Vector2Int InVector)
	{
		if (m_outPathListCapacity <= m_outPathListSize) return false;
		m_outPathListData[m_outPathListSize++] = InVector;
		return true;
	}

	FORCEINLINE void clear() { m_outPathListSize = 0; }
};

struct JPSGridInfoToFindPath
{
	uint64 *m_gridScanningHorizontalBitmap; // for grid scannning
	uint32 m_gridScanningHorizontalBitmapCapacity;

	uint64 *m_gridScanningVerticalBitmap; // for grid scanning
	uint32 m_gridScanningVerticalBitmalCapacity;
	
	PathFinderNode *m_gridMapPathfinderInfo;
	uint32 m_gridMapHorizontalSize;
	uint32 m_gridMapVerticalSize;


	FORCEINLINE PathFinderNode& GetNodeAt(const Vector2Int InLocation) const
	{
		assert(InLocation.m_x < m_gridMapHorizontalSize);
		assert(InLocation.m_y < m_gridMapVerticalSize);

		return m_gridMapPathfinderInfo[m_gridMapHorizontalSize * InLocation.m_y + InLocation.m_x];
	}

	FORCEINLINE bool IsBlockAt(const Vector2Int InLocation) const
	{
		assert(InLocation.m_x < m_gridMapHorizontalSize);
		assert(InLocation.m_y < m_gridMapVerticalSize);

		return m_gridMapPathfinderInfo[m_gridMapHorizontalSize * InLocation.m_y + InLocation.m_x].m_isBlock;
	}
	
};


extern "C" JPSPATHFINDERFASTER_API PathfindResult __stdcall FindPathJPSFaster(
	JPSGridInfoToFindPath& InGridInfo,
	PathfinderPriorityQueue& PathFinderPriorityQueuePool,
	PathfinderClostList& PathFinderCloseListPool,
	Vector2Int InStart, Vector2Int InEnd,
	OutPathList& OutPath);
