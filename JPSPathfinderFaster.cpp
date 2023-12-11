// JPSPathfinderFaster.cpp : DLL을 위해 내보낸 함수를 정의합니다.
//

#include "pch.h"
#include "framework.h"
#include "JPSPathfinderFaster.h"


Vector2Int const Vector2Int::InvalidIdx = Vector2Int(-1, -1);

inline int32 FindLeftmostSet(uint64 bitmap)
{
	uint32 result;

	if (_BitScanReverse64((unsigned long*)(&result), bitmap))
		return 63 - result;
	else
		return 64;

}

inline int32 FindRightmostSet(uint64 bitmap)
{
	uint32 result;

	if (_BitScanForward64((unsigned long*)&result, bitmap))
		return 63 - result;
	else
		return -1;
}

void PathfinderPriorityQueue::enqueue(const PriorityQueuePair dataToPush)
{
	assert(m_priorityQueueCapacity > m_priorityQueueSize);

	const uint32 pqLastIdx = m_priorityQueueSize++;
	m_priorityQueueData[pqLastIdx] = dataToPush;

	uint32 curHeapIdx = pqLastIdx;
	while (curHeapIdx > 0)
	{
		const uint32 parentHeapIdx = (curHeapIdx - 1) / 2;
		if (m_priorityQueueData[parentHeapIdx].m_fCost <= m_priorityQueueData[curHeapIdx].m_fCost)
			break;

		PriorityQueuePair temp = m_priorityQueueData[curHeapIdx];
		m_priorityQueueData[curHeapIdx] = m_priorityQueueData[parentHeapIdx];
		m_priorityQueueData[parentHeapIdx] = temp;

		curHeapIdx = parentHeapIdx;
	}

}

Vector2Int PathfinderPriorityQueue::dequeue()
{
	assert(m_priorityQueueSize > 0);

	--m_priorityQueueSize;
	const Vector2Int valueToReturn = m_priorityQueueData[0].m_nodeIdx;
	m_priorityQueueData[0] = m_priorityQueueData[m_priorityQueueSize];

	uint32 curHeapIdx = 0;
	while (true)
	{
		const uint32 leftChildHeapIdx = curHeapIdx * 2 + 1;
		const uint32 rightchildHeapIdx = curHeapIdx * 2 + 2;

		if (leftChildHeapIdx >= m_priorityQueueSize)
			break;

		int heapIdxToSwap = curHeapIdx;

		if (m_priorityQueueData[leftChildHeapIdx].m_fCost < m_priorityQueueData[curHeapIdx].m_fCost)
			heapIdxToSwap = leftChildHeapIdx;

		if (rightchildHeapIdx < m_priorityQueueSize &&
			m_priorityQueueData[rightchildHeapIdx].m_fCost < m_priorityQueueData[heapIdxToSwap].m_fCost)
			heapIdxToSwap = rightchildHeapIdx;

		if (heapIdxToSwap == curHeapIdx)
			break;

		PriorityQueuePair temp = m_priorityQueueData[curHeapIdx];
		m_priorityQueueData[curHeapIdx] = m_priorityQueueData[heapIdxToSwap];
		m_priorityQueueData[heapIdxToSwap] = temp;

		curHeapIdx = heapIdxToSwap;
	}

	return valueToReturn;
}

Vector2Int bitScanToRight(const JPSGridInfoToFindPath& InGridInfo, Vector2Int jumpStartNodeIdx, Vector2Int jumpTargetNodeIdx)
{
	const uint32 horizontalBitmalHorizontalSize = InGridInfo.m_gridMapHorizontalSize / 64;

	uint32 horizontalBitmapXIdx = jumpStartNodeIdx.m_x / 64u; 
	const uint32 horizontalBitmapYIdx = jumpStartNodeIdx.m_y;


	// scan jump points
	const uint32 bitmapMaskXIdx = jumpStartNodeIdx.m_x % 64u + 1;

	uint64 curBitmap = InGridInfo.m_gridScanningHorizontalBitmap[horizontalBitmapYIdx * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
	curBitmap = (curBitmap << bitmapMaskXIdx) >> bitmapMaskXIdx;
	uint64 curBitmapDown = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx - 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
	curBitmapDown = (curBitmapDown << bitmapMaskXIdx) >> bitmapMaskXIdx;
	uint64 curBitmapUp = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx + 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
	curBitmapUp = (curBitmapUp << bitmapMaskXIdx) >> bitmapMaskXIdx;

	uint64 forcedNodeBitmapDown = curBitmapDown & ~(curBitmapDown << 1);
	uint64 forcedNodeBitmapUp = curBitmapUp & ~(curBitmapUp << 1);

	uint64 bitscanResult = forcedNodeBitmapDown | curBitmap | forcedNodeBitmapUp;

	// if scan must stop ( return condition )
	if (bitscanResult != 0)
	{
		const int32 closestBlockXPos = horizontalBitmapXIdx * 64 + FindLeftmostSet(curBitmap);
		const int32 closestUpForcedXPos = horizontalBitmapXIdx * 64 + FindLeftmostSet(forcedNodeBitmapUp);
		const int32 closestDownForcedXPos = horizontalBitmapXIdx * 64 + FindLeftmostSet(forcedNodeBitmapDown);

		int32 xPosToJump;

		if (closestBlockXPos <= closestUpForcedXPos && closestBlockXPos <= closestDownForcedXPos)
			xPosToJump = closestBlockXPos;
		else if (closestUpForcedXPos > closestDownForcedXPos)
			xPosToJump = closestDownForcedXPos;
		else
			xPosToJump = closestUpForcedXPos;

		/*
		if (jumpStartNodeIdx.m_y == jumpTargetNodeIdx.m_y && xPosToJump > jumpTargetNodeIdx.m_x)
			xPosToJump = jumpTargetNodeIdx.m_x;
		// TODO: 이놈처럼 다른것들도 다 하기
		*/

		if (xPosToJump == closestBlockXPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(xPosToJump, horizontalBitmapYIdx);
	}

	// scan jump points
	bitscanResult = 0;
	while (bitscanResult == 0 && horizontalBitmapXIdx < horizontalBitmalHorizontalSize - 1)
	{
		++horizontalBitmapXIdx;

		curBitmap = InGridInfo.m_gridScanningHorizontalBitmap[horizontalBitmapYIdx * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
		curBitmapDown = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx - 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
		curBitmapUp = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx + 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];

		forcedNodeBitmapDown = curBitmapDown & ~(curBitmapDown << 1);
		forcedNodeBitmapUp = curBitmapUp & ~(curBitmapUp << 1);

		bitscanResult = forcedNodeBitmapDown | curBitmap | forcedNodeBitmapUp;
	}

	// if scan must stop ( return condition )
	if (bitscanResult != 0)
	{
		const int32 closestBlockXPos = horizontalBitmapXIdx * 64 + FindLeftmostSet(curBitmap);
		const int32 closestUpForcedXPos = horizontalBitmapXIdx * 64 + FindLeftmostSet(forcedNodeBitmapUp);
		const int32 closestDownForcedXPos = horizontalBitmapXIdx * 64 + FindLeftmostSet(forcedNodeBitmapDown);

		int32 xPosToJump;

		if (closestBlockXPos <= closestUpForcedXPos && closestBlockXPos <= closestDownForcedXPos)
			xPosToJump = closestBlockXPos;
		else if (closestUpForcedXPos > closestDownForcedXPos)
			xPosToJump = closestDownForcedXPos;
		else
			xPosToJump = closestUpForcedXPos;


		if (xPosToJump == closestBlockXPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(xPosToJump, horizontalBitmapYIdx);
	}

	assert(false);
	return Vector2Int::InvalidIdx;
}


Vector2Int bitScanToLeft(const JPSGridInfoToFindPath& InGridInfo, Vector2Int jumpStartNodeIdx, Vector2Int jumpTargetNodeIdx)
{
	const uint32 horizontalBitmalHorizontalSize = InGridInfo.m_gridMapHorizontalSize / 64;

	uint32 horizontalBitmapXIdx = jumpStartNodeIdx.m_x / 64u;
	const uint32 horizontalBitmapYIdx = jumpStartNodeIdx.m_y;


	// scan from StartPoint
	const uint32 bitmapMaskXIdx = 64 - jumpStartNodeIdx.m_x % 64u;

	uint64 curBitmap = InGridInfo.m_gridScanningHorizontalBitmap[horizontalBitmapYIdx * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
	curBitmap = (curBitmap >> bitmapMaskXIdx) << bitmapMaskXIdx;
	uint64 curBitmapDown = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx - 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
	curBitmapDown = (curBitmapDown >> bitmapMaskXIdx) << bitmapMaskXIdx;
	uint64 curBitmapUp = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx + 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
	curBitmapUp = (curBitmapUp >> bitmapMaskXIdx) << bitmapMaskXIdx;

	uint64 forcedNodeBitmapDown = curBitmapDown & ~(curBitmapDown >> 1);
	uint64 forcedNodeBitmapUp = curBitmapUp & ~(curBitmapUp >> 1);

	uint64 bitscanResult = forcedNodeBitmapDown | curBitmap | forcedNodeBitmapUp;

	// if scan must stop ( return condition )
	if (bitscanResult != 0)
	{
		const int32 closestBlockXPos = horizontalBitmapXIdx * 64 + FindRightmostSet(curBitmap);
		const int32 closestUpForcedXPos = horizontalBitmapXIdx * 64 + FindRightmostSet(forcedNodeBitmapUp);
		const int32 closestDownForcedXPos = horizontalBitmapXIdx * 64 + FindRightmostSet(forcedNodeBitmapDown);

		int32 xPosToJump;

		if (closestBlockXPos >= closestUpForcedXPos && closestBlockXPos >= closestDownForcedXPos)
			xPosToJump = closestBlockXPos;
		else if (closestUpForcedXPos < closestDownForcedXPos)
			xPosToJump = closestDownForcedXPos;
		else
			xPosToJump = closestUpForcedXPos;


		if (xPosToJump == closestBlockXPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(xPosToJump, horizontalBitmapYIdx);
	}

	bitscanResult = 0;
	while (bitscanResult == 0 && horizontalBitmapXIdx > 0)
	{
		--horizontalBitmapXIdx;

		curBitmap = InGridInfo.m_gridScanningHorizontalBitmap[horizontalBitmapYIdx * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
		curBitmapDown = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx - 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
		curBitmapUp = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx + 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];

		forcedNodeBitmapDown = curBitmapDown & ~(curBitmapDown >> 1);
		forcedNodeBitmapUp = curBitmapUp & ~(curBitmapUp >> 1);

		bitscanResult = forcedNodeBitmapDown | curBitmap | forcedNodeBitmapUp;
	}

	// if scan must stop ( return condition )
	if (bitscanResult != 0)
	{
		const int32 closestBlockXPos = horizontalBitmapXIdx * 64 + FindRightmostSet(curBitmap);
		const int32 closestUpForcedXPos = horizontalBitmapXIdx * 64 + FindRightmostSet(forcedNodeBitmapUp);
		const int32 closestDownForcedXPos = horizontalBitmapXIdx * 64 + FindRightmostSet(forcedNodeBitmapDown);

		int32 xPosToJump;

		if (closestBlockXPos >= closestUpForcedXPos && closestBlockXPos >= closestDownForcedXPos)
			xPosToJump = closestBlockXPos;
		else if (closestUpForcedXPos < closestDownForcedXPos)
			xPosToJump = closestDownForcedXPos;
		else
			xPosToJump = closestUpForcedXPos;


		if (xPosToJump == closestBlockXPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(xPosToJump, horizontalBitmapYIdx);
	}

	assert(false);
	return Vector2Int::InvalidIdx;
}


Vector2Int bitScanToUp(const JPSGridInfoToFindPath& InGridInfo, Vector2Int jumpStartNodeIdx, Vector2Int jumpTargetNodeIdx)
{
	const uint32 verticalBitmapHorizontalSize = InGridInfo.m_gridMapHorizontalSize;
	const uint32 verticalBitmapVerticalSize = InGridInfo.m_gridMapVerticalSize / 64;

	uint32 verticalBitmapYIdx = jumpStartNodeIdx.m_y / 64u;
	const uint32 verticalBitmapXIdx = jumpStartNodeIdx.m_x;


	// scan from StartPoint
	const uint32 bitmapMaskYIdx = jumpStartNodeIdx.m_y % 64u + 1;

	uint64 curBitmap = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + verticalBitmapXIdx];
	curBitmap = (curBitmap << bitmapMaskYIdx) >> bitmapMaskYIdx;
	uint64 curBitmapLeft = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx - 1)];
	curBitmapLeft = (curBitmapLeft << bitmapMaskYIdx) >> bitmapMaskYIdx;
	uint64 curBitmapRight = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx + 1)];
	curBitmapRight = (curBitmapRight << bitmapMaskYIdx) >> bitmapMaskYIdx;

	uint64 forcedNodeBitmapLeft = curBitmapLeft & ~(curBitmapLeft << 1);
	uint64 forcedNodeBitmapRight = curBitmapRight & ~(curBitmapRight << 1);

	uint64 bitscanResult = forcedNodeBitmapLeft | curBitmap | forcedNodeBitmapRight;

	// if scan must stop ( return condition )
	if (bitscanResult != 0)
	{
		const int32 closestBlockYPos = verticalBitmapYIdx * 64 + FindLeftmostSet(curBitmap);
		const int32 closestLeftForcedYPos = verticalBitmapYIdx * 64 + FindLeftmostSet(forcedNodeBitmapLeft);
		const int32 closestRightForcedYPos = verticalBitmapYIdx * 64 + FindLeftmostSet(forcedNodeBitmapRight);

		int32 yPosToJump;

		if (closestBlockYPos <= closestLeftForcedYPos && closestBlockYPos <= closestRightForcedYPos)
			yPosToJump = closestBlockYPos;
		else if (closestLeftForcedYPos < closestRightForcedYPos)
			yPosToJump = closestLeftForcedYPos;
		else
			yPosToJump = closestRightForcedYPos;


		if (yPosToJump == closestBlockYPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(verticalBitmapXIdx, yPosToJump);
	}

	bitscanResult = 0;
	while (bitscanResult == 0 && verticalBitmapYIdx < verticalBitmapVerticalSize - 1)
	{
		++verticalBitmapYIdx;

		curBitmap = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + verticalBitmapXIdx];
		curBitmapLeft = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx - 1)];
		curBitmapRight = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx + 1)];

		forcedNodeBitmapLeft = curBitmapLeft & ~(curBitmapLeft << 1);
		forcedNodeBitmapRight = curBitmapRight & ~(curBitmapRight << 1);

		bitscanResult = forcedNodeBitmapLeft | curBitmap | forcedNodeBitmapRight;
	}

	// if scan must stop ( return condition )
	if (bitscanResult != 0)
	{
		const int32 closestBlockYPos = verticalBitmapYIdx * 64 + FindLeftmostSet(curBitmap);
		const int32 closestLeftForcedYPos = verticalBitmapYIdx * 64 + FindLeftmostSet(forcedNodeBitmapLeft);
		const int32 closestRightForcedYPos = verticalBitmapYIdx * 64 + FindLeftmostSet(forcedNodeBitmapRight);

		int32 yPosToJump;

		if (closestBlockYPos <= closestLeftForcedYPos && closestBlockYPos <= closestRightForcedYPos)
			yPosToJump = closestBlockYPos;
		else if (closestLeftForcedYPos < closestRightForcedYPos)
			yPosToJump = closestLeftForcedYPos;
		else
			yPosToJump = closestRightForcedYPos;


		if (yPosToJump == closestBlockYPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(verticalBitmapXIdx, yPosToJump);
	}

	assert(false);
	return Vector2Int::InvalidIdx;
}

Vector2Int bitScanToDown(const JPSGridInfoToFindPath& InGridInfo, Vector2Int jumpStartNodeIdx, Vector2Int jumpTargetNodeIdx)
{
	const uint32 verticalBitmapHorizontalSize = InGridInfo.m_gridMapHorizontalSize;
	const uint32 verticalBitmapVerticalSize = InGridInfo.m_gridMapVerticalSize / 64;

	uint32 verticalBitmapYIdx = jumpStartNodeIdx.m_y / 64u;
	const uint32 verticalBitmapXIdx = jumpStartNodeIdx.m_x;


	// scan from StartPoint
	const uint32 bitmapMaskYIdx = 64 - jumpStartNodeIdx.m_y % 64u;

	uint64 curBitmap = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + verticalBitmapXIdx];
	curBitmap = (curBitmap >> bitmapMaskYIdx) << bitmapMaskYIdx;
	uint64 curBitmapLeft = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx - 1)];
	curBitmapLeft = (curBitmapLeft >> bitmapMaskYIdx) << bitmapMaskYIdx;
	uint64 curBitmapRight = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx + 1)];
	curBitmapRight = (curBitmapRight >> bitmapMaskYIdx) << bitmapMaskYIdx;

	uint64 forcedNodeBitmapLeft = curBitmapLeft & ~(curBitmapLeft >> 1);
	uint64 forcedNodeBitmapRight = curBitmapRight & ~(curBitmapRight >> 1);

	uint64 bitscanResult = forcedNodeBitmapLeft | curBitmap | forcedNodeBitmapRight;

	// if scan must stop ( return condition )
	if (bitscanResult != 0)
	{
		const int32 closestBlockYPos = verticalBitmapYIdx * 64 + FindRightmostSet(curBitmap);
		const int32 closestLeftForcedYPos = verticalBitmapYIdx * 64 + FindRightmostSet(forcedNodeBitmapLeft);
		const int32 closestRightForcedYPos = verticalBitmapYIdx * 64 + FindRightmostSet(forcedNodeBitmapRight);

		int32 yPosToJump;

		if (closestBlockYPos >= closestLeftForcedYPos && closestBlockYPos >= closestRightForcedYPos)
			yPosToJump = closestBlockYPos;
		else if (closestLeftForcedYPos < closestRightForcedYPos)
			yPosToJump = closestRightForcedYPos;
		else
			yPosToJump = closestLeftForcedYPos;


		if (yPosToJump == closestBlockYPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(verticalBitmapXIdx, yPosToJump);
	}

	bitscanResult = 0;
	while ( bitscanResult == 0 && verticalBitmapYIdx > 0 )
	{
		--verticalBitmapYIdx;

		curBitmap = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + verticalBitmapXIdx];
		curBitmapLeft = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx - 1)];
		curBitmapRight = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx + 1)];

		forcedNodeBitmapLeft = curBitmapLeft & ~(curBitmapLeft >> 1);
		forcedNodeBitmapRight = curBitmapRight & ~(curBitmapRight >> 1);

		bitscanResult = forcedNodeBitmapLeft | curBitmap | forcedNodeBitmapRight;
	}

	// if scan must stop ( return condition )
	if (bitscanResult != 0)
	{
		const int32 closestBlockYPos = verticalBitmapYIdx * 64 + FindRightmostSet(curBitmap);
		const int32 closestLeftForcedYPos = verticalBitmapYIdx * 64 + FindRightmostSet(forcedNodeBitmapLeft);
		const int32 closestRightForcedYPos = verticalBitmapYIdx * 64 + FindRightmostSet(forcedNodeBitmapRight);

		int32 yPosToJump;

		if (closestBlockYPos >= closestLeftForcedYPos && closestBlockYPos >= closestRightForcedYPos)
			yPosToJump = closestBlockYPos;
		else if (closestLeftForcedYPos < closestRightForcedYPos)
			yPosToJump = closestRightForcedYPos;
		else
			yPosToJump = closestLeftForcedYPos;


		if (yPosToJump == closestBlockYPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(verticalBitmapXIdx, yPosToJump);
	}

	assert(false);
	return Vector2Int::InvalidIdx;
}



bool __stdcall FindPathJPSFaster(
	JPSGridInfoToFindPath& InGridInfo,
	PathfinderPriorityQueue& InPathFinderPriorityQueuePool,
	PathfinderClostList& InPathFinderCloseListPool,
	Vector2Int InStart, Vector2Int InEnd,
	Vector2Int* OutPath, uint32& OutPathSize)
{
	OutPathSize = 0;

	if (InStart.m_x < 0 || InStart.m_x >= InGridInfo.m_gridMapHorizontalSize ||
		InStart.m_y < 0 || InStart.m_y >= InGridInfo.m_gridMapVerticalSize ||
		InEnd.m_x < 0 || InEnd.m_x >= InGridInfo.m_gridMapHorizontalSize ||
		InEnd.m_y < 0 || InEnd.m_y >= InGridInfo.m_gridMapVerticalSize)
	{
		return false;
	}

	constexpr uint64 BIT_BASE = static_cast<uint64>(1) << 63;
	

	OutPath[OutPathSize++] = bitScanToLeft(InGridInfo, InStart, InEnd);
	OutPath[OutPathSize++] = bitScanToRight(InGridInfo, InStart, InEnd);
	OutPath[OutPathSize++] = bitScanToUp(InGridInfo, InStart, InEnd);
	OutPath[OutPathSize++] = bitScanToDown(InGridInfo, InStart, InEnd);


	return true;
}

