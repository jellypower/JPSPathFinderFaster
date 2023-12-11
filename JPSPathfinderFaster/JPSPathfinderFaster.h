#include <assert.h>

#ifdef JPSPATHFINDERFASTER_EXPORTS
#define JPSPATHFINDERFASTER_API __declspec(dllexport)
#else
#define JPSPATHFINDERFASTER_API __declspec(dllimport)
#endif

#define BITMAP_UNIT_MAX 10
#define OUT_PATH_LEN_MAX 100
#define NOOP


typedef int int32;

typedef unsigned int uint32;
typedef unsigned long long int uint64;
typedef unsigned char uint8;

struct Vector2Int
{
	uint32 m_x, m_y;

	Vector2Int(uint32 InX, uint32 InY):
		m_x(InX), m_y(InY)
	{
		NOOP;
	}

	bool operator==(Vector2Int rhs) { return m_x == rhs.m_x && m_y == rhs.m_y; }
	bool isValid() { return (m_x != -1) && (m_y != -1); }

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
	float m_hCost = 0;
	Vector2Int m_paretnNode = Vector2Int(-1,-1);
	bool m_onCloseList = false;
	bool m_onOpenList = false;
	const inline float getFCost() const { return m_gCost + m_hCost; }
}; 

struct PathfinderPriorityQueue
{
	PriorityQueuePair *m_priorityQueueData = nullptr;
	uint32 m_priorityQueueCapacity = 0;
	uint32 m_priorityQueueSize = 0;

	void enqueue(const PriorityQueuePair dataToPush);
	Vector2Int dequeue();
	
	inline bool isEmpty() { m_priorityQueueSize == 0; }
	inline bool isFull() { m_priorityQueueSize == m_priorityQueueCapacity; }
};

struct PathfinderClostList
{
public:
	Vector2Int* m_closeListData;
	uint32 m_closeListCapacity;
	uint32 m_closeListSize;

	void push_back(Vector2Int InVector)
	{
		m_closeListData[m_closeListSize++] = InVector;
	}
	void clear()
	{
		m_closeListSize = 0;
	}

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


	inline PathFinderNode& GetNodeAt(const Vector2Int InLocation) const
	{
		assert(InLocation.m_x < m_gridMapHorizontalSize);
		assert(InLocation.m_y < m_gridMapVerticalSize);

		return m_gridMapPathfinderInfo[m_gridMapHorizontalSize * InLocation.m_y + InLocation.m_x];
	}
	
};


extern "C" JPSPATHFINDERFASTER_API bool __stdcall FindPathJPSFaster(
	JPSGridInfoToFindPath& InGridInfo,
	PathfinderPriorityQueue& InPathFinderPriorityQueuePool,
	PathfinderClostList& InPathFinderCloseListPool,
	Vector2Int InStart, Vector2Int InEnd,
	Vector2Int* OutPath, uint32& OutPathSize);