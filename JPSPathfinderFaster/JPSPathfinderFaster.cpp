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

bool PathfinderPriorityQueue::enqueue(const PriorityQueuePair dataToPush)
{
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

	if (m_priorityQueueCapacity <= m_priorityQueueSize) return false;
	return true;
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

Vector2Int bitScanToRight(const JPSGridInfoToFindPath& InGridInfo, const Vector2Int& jumpStartNodeIdx, const Vector2Int& jumpTargetNodeIdx)
{
	const uint32 horizontalBitmalHorizontalSize = InGridInfo.m_gridMapHorizontalSize / 64;

	uint32 horizontalBitmapXIdx = jumpStartNodeIdx.m_x / 64u;
	const uint32 horizontalBitmapYIdx = jumpStartNodeIdx.m_y;


	// scan jump points
	const uint32 bitmapMaskXIdx = jumpStartNodeIdx.m_x % 64u + 1;

	uint64 curBitmap;
	uint64 curBitmapDown;
	uint64 curBitmapUp;
	uint64 forcedNodeBitmapDown;
	uint64 forcedNodeBitmapUp;
	uint64 bitscanResult;

	if (bitmapMaskXIdx < 64)
	{
		curBitmap = InGridInfo.m_gridScanningHorizontalBitmap[horizontalBitmapYIdx * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
		curBitmap = (curBitmap << bitmapMaskXIdx) >> bitmapMaskXIdx;
		curBitmapDown = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx - 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
		curBitmapDown = (curBitmapDown << bitmapMaskXIdx) >> bitmapMaskXIdx;
		curBitmapUp = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx + 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
		curBitmapUp = (curBitmapUp << bitmapMaskXIdx) >> bitmapMaskXIdx;

		forcedNodeBitmapDown = curBitmapDown & ~(curBitmapDown << 1);
		forcedNodeBitmapUp = curBitmapUp & ~(curBitmapUp << 1);

		bitscanResult = forcedNodeBitmapDown | curBitmap | forcedNodeBitmapUp;

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

			if (jumpStartNodeIdx.m_y == jumpTargetNodeIdx.m_y &&
				jumpStartNodeIdx.m_x < jumpTargetNodeIdx.m_x && jumpTargetNodeIdx.m_x < xPosToJump)
				xPosToJump = jumpTargetNodeIdx.m_x;

			if (xPosToJump == closestBlockXPos)
				return Vector2Int::InvalidIdx;
			else
				return Vector2Int(xPosToJump, horizontalBitmapYIdx);
		}
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

		if (jumpStartNodeIdx.m_y == jumpTargetNodeIdx.m_y &&
			jumpStartNodeIdx.m_x < jumpTargetNodeIdx.m_x && jumpTargetNodeIdx.m_x < xPosToJump)
			xPosToJump = jumpTargetNodeIdx.m_x;

		if (xPosToJump == closestBlockXPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(xPosToJump, horizontalBitmapYIdx);
	}

	assert(false);
	return Vector2Int::InvalidIdx;
}


Vector2Int bitScanToLeft(const JPSGridInfoToFindPath& InGridInfo, const Vector2Int& jumpStartNodeIdx, const Vector2Int& jumpTargetNodeIdx)
{
	const uint32 horizontalBitmalHorizontalSize = InGridInfo.m_gridMapHorizontalSize / 64;

	uint32 horizontalBitmapXIdx = jumpStartNodeIdx.m_x / 64u;
	const uint32 horizontalBitmapYIdx = jumpStartNodeIdx.m_y;


	// scan from StartPoint
	const uint32 bitmapMaskXIdx = 64 - jumpStartNodeIdx.m_x % 64u;

	uint64 curBitmap;
	uint64 curBitmapDown;
	uint64 curBitmapUp;
	uint64 forcedNodeBitmapDown;
	uint64 forcedNodeBitmapUp;
	uint64 bitscanResult;

	if (bitmapMaskXIdx < 64)
	{
		curBitmap = InGridInfo.m_gridScanningHorizontalBitmap[horizontalBitmapYIdx * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
		curBitmap = (curBitmap >> bitmapMaskXIdx) << bitmapMaskXIdx;
		curBitmapDown = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx - 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
		curBitmapDown = (curBitmapDown >> bitmapMaskXIdx) << bitmapMaskXIdx;
		curBitmapUp = InGridInfo.m_gridScanningHorizontalBitmap[(horizontalBitmapYIdx + 1) * horizontalBitmalHorizontalSize + horizontalBitmapXIdx];
		curBitmapUp = (curBitmapUp >> bitmapMaskXIdx) << bitmapMaskXIdx;

		forcedNodeBitmapDown = curBitmapDown & ~(curBitmapDown >> 1);
		forcedNodeBitmapUp = curBitmapUp & ~(curBitmapUp >> 1);

		bitscanResult = forcedNodeBitmapDown | curBitmap | forcedNodeBitmapUp;

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

			if (jumpStartNodeIdx.m_y == jumpTargetNodeIdx.m_y &&
				xPosToJump < jumpTargetNodeIdx.m_x && jumpTargetNodeIdx.m_x < jumpStartNodeIdx.m_x)
				xPosToJump = jumpTargetNodeIdx.m_x;

			if (xPosToJump == closestBlockXPos)
				return Vector2Int::InvalidIdx;
			else
				return Vector2Int(xPosToJump, horizontalBitmapYIdx);
		}
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

		if (jumpStartNodeIdx.m_y == jumpTargetNodeIdx.m_y &&
			xPosToJump < jumpTargetNodeIdx.m_x && jumpTargetNodeIdx.m_x < jumpStartNodeIdx.m_x)
			xPosToJump = jumpTargetNodeIdx.m_x;

		if (xPosToJump == closestBlockXPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(xPosToJump, horizontalBitmapYIdx);
	}

	assert(false);
	return Vector2Int::InvalidIdx;
}


Vector2Int bitScanToUp(const JPSGridInfoToFindPath& InGridInfo, const Vector2Int& jumpStartNodeIdx, const Vector2Int& jumpTargetNodeIdx)
{
	const uint32 verticalBitmapHorizontalSize = InGridInfo.m_gridMapHorizontalSize;
	const uint32 verticalBitmapVerticalSize = InGridInfo.m_gridMapVerticalSize / 64;

	uint32 verticalBitmapYIdx = jumpStartNodeIdx.m_y / 64u;
	const uint32 verticalBitmapXIdx = jumpStartNodeIdx.m_x;


	// scan from StartPoint
	const uint32 bitmapMaskYIdx = jumpStartNodeIdx.m_y % 64u + 1;

	uint64 curBitmap;
	uint64 curBitmapLeft;
	uint64 curBitmapRight;
	uint64 forcedNodeBitmapLeft;
	uint64 forcedNodeBitmapRight;
	uint64 bitscanResult;

	if (bitmapMaskYIdx < 64)
	{
		curBitmap = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + verticalBitmapXIdx];
		curBitmap = (curBitmap << bitmapMaskYIdx) >> bitmapMaskYIdx;
		curBitmapLeft = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx - 1)];
		curBitmapLeft = (curBitmapLeft << bitmapMaskYIdx) >> bitmapMaskYIdx;
		curBitmapRight = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx + 1)];
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

			if (jumpStartNodeIdx.m_x == jumpTargetNodeIdx.m_x &&
				jumpStartNodeIdx.m_y < jumpTargetNodeIdx.m_y && jumpTargetNodeIdx.m_y < yPosToJump)
				yPosToJump = jumpTargetNodeIdx.m_y;

			if (yPosToJump == closestBlockYPos)
				return Vector2Int::InvalidIdx;
			else
				return Vector2Int(verticalBitmapXIdx, yPosToJump);
		}
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

		if (jumpStartNodeIdx.m_x == jumpTargetNodeIdx.m_x &&
			jumpStartNodeIdx.m_y < jumpTargetNodeIdx.m_y && jumpTargetNodeIdx.m_y < yPosToJump)
			yPosToJump = jumpTargetNodeIdx.m_y;

		if (yPosToJump == closestBlockYPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(verticalBitmapXIdx, yPosToJump);
	}

	assert(false);
	return Vector2Int::InvalidIdx;
}

Vector2Int bitScanToDown(const JPSGridInfoToFindPath& InGridInfo, const Vector2Int& jumpStartNodeIdx, const Vector2Int& jumpTargetNodeIdx)
{
	const uint32 verticalBitmapHorizontalSize = InGridInfo.m_gridMapHorizontalSize;
	const uint32 verticalBitmapVerticalSize = InGridInfo.m_gridMapVerticalSize / 64;

	uint32 verticalBitmapYIdx = jumpStartNodeIdx.m_y / 64u;
	const uint32 verticalBitmapXIdx = jumpStartNodeIdx.m_x;


	// scan from StartPoint
	const uint32 bitmapMaskYIdx = 64 - jumpStartNodeIdx.m_y % 64u;

	uint64 curBitmap;
	uint64 curBitmapLeft;
	uint64 curBitmapRight;
	uint64 forcedNodeBitmapLeft;
	uint64 forcedNodeBitmapRight;
	uint64 bitscanResult;

	if (bitmapMaskYIdx < 64) {

		curBitmap = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + verticalBitmapXIdx];
		curBitmap = (curBitmap >> bitmapMaskYIdx) << bitmapMaskYIdx;
		curBitmapLeft = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx - 1)];
		curBitmapLeft = (curBitmapLeft >> bitmapMaskYIdx) << bitmapMaskYIdx;
		curBitmapRight = InGridInfo.m_gridScanningVerticalBitmap[verticalBitmapYIdx * verticalBitmapHorizontalSize + (verticalBitmapXIdx + 1)];
		curBitmapRight = (curBitmapRight >> bitmapMaskYIdx) << bitmapMaskYIdx;

		forcedNodeBitmapLeft = curBitmapLeft & ~(curBitmapLeft >> 1);
		forcedNodeBitmapRight = curBitmapRight & ~(curBitmapRight >> 1);

		bitscanResult = forcedNodeBitmapLeft | curBitmap | forcedNodeBitmapRight;

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

			if (jumpStartNodeIdx.m_x == jumpTargetNodeIdx.m_x &&
				yPosToJump < jumpTargetNodeIdx.m_y && jumpTargetNodeIdx.m_y < jumpStartNodeIdx.m_y)
				yPosToJump = jumpTargetNodeIdx.m_y;

			if (yPosToJump == closestBlockYPos)
				return Vector2Int::InvalidIdx;
			else
				return Vector2Int(verticalBitmapXIdx, yPosToJump);
		}
	}

	bitscanResult = 0;
	while (bitscanResult == 0 && verticalBitmapYIdx > 0)
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

		if (jumpStartNodeIdx.m_x == jumpTargetNodeIdx.m_x &&
			yPosToJump < jumpTargetNodeIdx.m_y && jumpTargetNodeIdx.m_y < jumpStartNodeIdx.m_y)
			yPosToJump = jumpTargetNodeIdx.m_y;

		if (yPosToJump == closestBlockYPos)
			return Vector2Int::InvalidIdx;
		else
			return Vector2Int(verticalBitmapXIdx, yPosToJump);
	}

	assert(false);
	return Vector2Int::InvalidIdx;
}

Vector2Int bitScanToRightUp(const JPSGridInfoToFindPath& InGridInfo, const Vector2Int& jumpStartNodeIdx, const Vector2Int& goalNodeIdx)
{
	Vector2Int curNodeIdx = jumpStartNodeIdx;

	while (true)
	{
		curNodeIdx.m_x += 1;
		curNodeIdx.m_y += 1;


		const int32 curXIdx = curNodeIdx.m_x;
		const int32 curYIdx = curNodeIdx.m_y;

		if (InGridInfo.IsBlockAt(curNodeIdx)) return Vector2Int::InvalidIdx;

		if (
			curNodeIdx == goalNodeIdx ||
			!InGridInfo.IsBlockAt(Vector2Int(curXIdx - 1, curYIdx + 1)) && InGridInfo.IsBlockAt(Vector2Int(curXIdx - 1, curYIdx)) ||
			!InGridInfo.IsBlockAt(Vector2Int(curXIdx + 1, curYIdx - 1)) && InGridInfo.IsBlockAt(Vector2Int(curXIdx, curYIdx - 1))
			)
		{
			return curNodeIdx;
		}

		Vector2Int jumpTarget = bitScanToRight(InGridInfo, curNodeIdx, goalNodeIdx);
		if (jumpTarget.isValid())
			return curNodeIdx;

		jumpTarget = bitScanToUp(InGridInfo, curNodeIdx, goalNodeIdx);
		if (jumpTarget.isValid())
			return curNodeIdx;
	}

}

Vector2Int bitScanToRightDown(const JPSGridInfoToFindPath& InGridInfo, const Vector2Int& jumpStartNodeIdx, const Vector2Int& goalNodeIdx)
{
	Vector2Int curNodeIdx = jumpStartNodeIdx;

	while (true)
	{
		curNodeIdx.m_x += 1;
		curNodeIdx.m_y -= 1;


		const int32 curXIdx = curNodeIdx.m_x;
		const int32 curYIdx = curNodeIdx.m_y;

		if (InGridInfo.IsBlockAt(curNodeIdx)) return Vector2Int::InvalidIdx;

		if (
			curNodeIdx == goalNodeIdx ||
			!InGridInfo.IsBlockAt(Vector2Int(curXIdx - 1, curYIdx - 1)) && InGridInfo.IsBlockAt(Vector2Int(curXIdx - 1, curYIdx)) ||
			!InGridInfo.IsBlockAt(Vector2Int(curXIdx + 1, curYIdx + 1)) && InGridInfo.IsBlockAt(Vector2Int(curXIdx, curYIdx + 1))
			)
		{
			return curNodeIdx;
		}

		Vector2Int jumpTarget = bitScanToRight(InGridInfo, curNodeIdx, goalNodeIdx);
		if (jumpTarget.isValid())
			return curNodeIdx;

		jumpTarget = bitScanToDown(InGridInfo, curNodeIdx, goalNodeIdx);
		if (jumpTarget.isValid())
			return curNodeIdx;
	}

}

Vector2Int bitScanToLeftUp(const JPSGridInfoToFindPath& InGridInfo, const Vector2Int& jumpStartNodeIdx, const Vector2Int& goalNodeIdx)
{
	Vector2Int curNodeIdx = jumpStartNodeIdx;

	while (true)
	{
		curNodeIdx.m_x -= 1;
		curNodeIdx.m_y += 1;

		const int32 curXIdx = curNodeIdx.m_x;
		const int32 curYIdx = curNodeIdx.m_y;

		if (InGridInfo.IsBlockAt(curNodeIdx)) return Vector2Int::InvalidIdx;

		if (
			curNodeIdx == goalNodeIdx ||
			!InGridInfo.IsBlockAt(Vector2Int(curXIdx + 1, curYIdx + 1)) && InGridInfo.IsBlockAt(Vector2Int(curXIdx + 1, curYIdx)) ||
			!InGridInfo.IsBlockAt(Vector2Int(curXIdx - 1, curYIdx - 1)) && InGridInfo.IsBlockAt(Vector2Int(curXIdx, curYIdx - 1))
			)
		{
			return curNodeIdx;
		}

		Vector2Int jumpTarget = bitScanToLeft(InGridInfo, curNodeIdx, goalNodeIdx);
		if (jumpTarget.isValid())
			return curNodeIdx;

		jumpTarget = bitScanToUp(InGridInfo, curNodeIdx, goalNodeIdx);
		if (jumpTarget.isValid())
			return curNodeIdx;
	}

}

Vector2Int bitScanToLeftDown(const JPSGridInfoToFindPath& InGridInfo, const Vector2Int& jumpStartNodeIdx, const Vector2Int& goalNodeIdx)
{
	Vector2Int curNodeIdx = jumpStartNodeIdx;

	while (true)
	{
		curNodeIdx.m_x -= 1;
		curNodeIdx.m_y -= 1;

		const int32 curXIdx = curNodeIdx.m_x;
		const int32 curYIdx = curNodeIdx.m_y;

		if (InGridInfo.IsBlockAt(curNodeIdx)) return Vector2Int::InvalidIdx;

		if (
			curNodeIdx == goalNodeIdx ||
			!InGridInfo.IsBlockAt(Vector2Int(curXIdx + 1, curYIdx - 1)) && InGridInfo.IsBlockAt(Vector2Int(curXIdx + 1, curYIdx)) ||
			!InGridInfo.IsBlockAt(Vector2Int(curXIdx - 1, curYIdx + 1)) && InGridInfo.IsBlockAt(Vector2Int(curXIdx, curYIdx + 1))
			)
		{
			return curNodeIdx;
		}

		Vector2Int jumpTarget = bitScanToLeft(InGridInfo, curNodeIdx, goalNodeIdx);
		if (jumpTarget.isValid())
			return curNodeIdx;

		jumpTarget = bitScanToDown(InGridInfo, curNodeIdx, goalNodeIdx);
		if (jumpTarget.isValid())
			return curNodeIdx;
	}

}

bool updateJumpPoint(
	JPSGridInfoToFindPath& InGridInfo,
	PathfinderPriorityQueue& PathFinderPriorityQueuePool,
	const Vector2Int& jumpStartNodeIdx, const Vector2Int& jumpEndNodeIdx, const Vector2Int& goalNodeIdx)
{
	if (jumpEndNodeIdx.isValid() == false) return true;

	const PathFinderNode& jumpStartNode = InGridInfo.GetNodeAt(jumpStartNodeIdx);
	PathFinderNode& jumpEndNode = InGridInfo.GetNodeAt(jumpEndNodeIdx);

	if (jumpEndNode.m_onCloseList) return true;

	const float hCost = Vector2Int::OctileDistance(jumpEndNodeIdx, goalNodeIdx);
	const float newGCost = jumpStartNode.m_gCost + Vector2Int::OctileDistance(jumpStartNodeIdx, jumpEndNodeIdx);

	if (jumpEndNode.m_gCost + hCost > newGCost + hCost)
	{
		jumpEndNode.m_paretnNode = jumpStartNodeIdx;
		jumpEndNode.m_gCost = newGCost;
		return PathFinderPriorityQueuePool.enqueue(PriorityQueuePair(jumpEndNodeIdx, newGCost + hCost));
	}

	return true;
}



PathfindResult __stdcall FindPathJPSFaster(
	JPSGridInfoToFindPath& InGridInfo,
	PathfinderPriorityQueue& PathFinderPriorityQueuePool,
	PathfinderClostList& PathFinderCloseListPool,
	Vector2Int InStart, Vector2Int InEnd,
	OutPathList& OutPath)
{
	if (InStart.m_x <= 0 || InStart.m_x >= (int32)InGridInfo.m_gridMapHorizontalSize - 1 ||
		InStart.m_y <= 0 || InStart.m_y >= (int32)InGridInfo.m_gridMapVerticalSize - 1 ||
		InEnd.m_x <= 0 || InEnd.m_x >= (int32)InGridInfo.m_gridMapHorizontalSize - 1 ||
		InEnd.m_y <= 0 || InEnd.m_y >= (int32)InGridInfo.m_gridMapVerticalSize - 1)
	{
		return PathfindResult::StartOrEndPointOutOfBound;
	}

	PathFinderCloseListPool.clear();
	PathFinderPriorityQueuePool.clear();
	OutPath.clear();

	if (InStart.isValid() == false || InEnd.isValid() == false) PathfindResult::StartOrEndPointOutOfBound;
	if (InStart == InEnd)
	{
		OutPath.push_back(InEnd);
		OutPath.push_back(InStart);
		return PathfindResult::Found;
	}

	PathFinderPriorityQueuePool.enqueue(PriorityQueuePair(InStart, Vector2Int::OctileDistance(InStart, InEnd)));
	InGridInfo.GetNodeAt(InStart).m_gCost = 0;

	bool isPathFound = false;
	bool isCloseListMax = false;
	bool isPriorityQueueMax = false;

	while (PathFinderPriorityQueuePool.isEmpty() == false)
	{
		Vector2Int nodeIdxToStartJump = PathFinderPriorityQueuePool.dequeue();
		PathFinderNode& NodeToStartJump = InGridInfo.GetNodeAt(nodeIdxToStartJump);

		if (nodeIdxToStartJump == InEnd)
		{
			isCloseListMax = PathFinderCloseListPool.push_back(nodeIdxToStartJump) == false;
			NodeToStartJump.m_onCloseList = true;
			isPathFound = true;
			break;
		}

		if (NodeToStartJump.m_onCloseList)
			continue;

		isCloseListMax = PathFinderCloseListPool.push_back(nodeIdxToStartJump) == false;
		NodeToStartJump.m_onCloseList = true;
		if (isCloseListMax)
			break;

		// jump process
		{

			isPriorityQueueMax =
				updateJumpPoint(InGridInfo, PathFinderPriorityQueuePool,
					nodeIdxToStartJump, bitScanToRight(InGridInfo, nodeIdxToStartJump, InEnd), InEnd) == false;
			if (isPriorityQueueMax) break;

			isPriorityQueueMax =
				updateJumpPoint(InGridInfo, PathFinderPriorityQueuePool,
					nodeIdxToStartJump, bitScanToLeft(InGridInfo, nodeIdxToStartJump, InEnd), InEnd) == false;
			if (isPriorityQueueMax) break;

			isPriorityQueueMax =
				updateJumpPoint(InGridInfo, PathFinderPriorityQueuePool,
					nodeIdxToStartJump, bitScanToUp(InGridInfo, nodeIdxToStartJump, InEnd), InEnd) == false;
			if (isPriorityQueueMax) break;

			isPriorityQueueMax =
				updateJumpPoint(InGridInfo, PathFinderPriorityQueuePool,
					nodeIdxToStartJump, bitScanToDown(InGridInfo, nodeIdxToStartJump, InEnd), InEnd) == false;
			if (isPriorityQueueMax) break;

			isPriorityQueueMax =
				updateJumpPoint(InGridInfo, PathFinderPriorityQueuePool,
					nodeIdxToStartJump, bitScanToRightUp(InGridInfo, nodeIdxToStartJump, InEnd), InEnd) == false;
			if (isPriorityQueueMax) break;

			isPriorityQueueMax =
				updateJumpPoint(InGridInfo, PathFinderPriorityQueuePool,
					nodeIdxToStartJump, bitScanToRightDown(InGridInfo, nodeIdxToStartJump, InEnd), InEnd) == false;
			if (isPriorityQueueMax) break;

			isPriorityQueueMax =
				updateJumpPoint(InGridInfo, PathFinderPriorityQueuePool,
					nodeIdxToStartJump, bitScanToLeftUp(InGridInfo, nodeIdxToStartJump, InEnd), InEnd) == false;
			if (isPriorityQueueMax) break;

			isPriorityQueueMax =
				updateJumpPoint(InGridInfo, PathFinderPriorityQueuePool,
					nodeIdxToStartJump, bitScanToLeftDown(InGridInfo, nodeIdxToStartJump, InEnd), InEnd) == false;
			if (isPriorityQueueMax) break;
		}

	}


	bool isOutPathMax = false;
	if (isPathFound)
	{
		isOutPathMax = (OutPath.push_back(InEnd) == false);
		
		Vector2Int childNodeIdx = InEnd;
		Vector2Int nodeIdx = InGridInfo.GetNodeAt(InEnd).m_paretnNode;
		Vector2Int parentNodeIdx = InGridInfo.GetNodeAt(nodeIdx).m_paretnNode;

		// exclude useless ways
		while (parentNodeIdx.isValid() && isOutPathMax == false)
		{
			enum class DiffType {
				HORIZONTAL,
				VETICAL,
				DIAGONAL
			};

			Vector2Int diff1 = Vector2Int(
				Abs32i(childNodeIdx.m_x - nodeIdx.m_x),
				Abs32i(childNodeIdx.m_y - nodeIdx.m_y));

			Vector2Int diff2 = Vector2Int(
				Abs32i(parentNodeIdx.m_x - nodeIdx.m_x),
				Abs32i(parentNodeIdx.m_y - nodeIdx.m_y));

			DiffType diff1DiffType;
			if (diff1.m_x >= 1 && diff1.m_y >= 1)
				diff1DiffType = DiffType::DIAGONAL;
			else if (diff1.m_x >= 1)
				diff1DiffType = DiffType::HORIZONTAL;
			else
				diff1DiffType = DiffType::VETICAL;

			DiffType diff2DiffType;
			if (diff2.m_x >= 1 && diff2.m_y >= 1)
				diff2DiffType = DiffType::DIAGONAL;
			else if (diff2.m_x >= 1)
				diff2DiffType = DiffType::HORIZONTAL;
			else
				diff2DiffType = DiffType::VETICAL;

			if (diff1DiffType != diff2DiffType)
				isOutPathMax = (OutPath.push_back(nodeIdx) == false);

			childNodeIdx = nodeIdx;
			nodeIdx = parentNodeIdx;
			parentNodeIdx = InGridInfo.GetNodeAt(parentNodeIdx).m_paretnNode;
		}

		isOutPathMax = (OutPath.push_back(InStart) == false);
		
	}

	// init pathfinder grid state
	for (int i = 0; i < PathFinderPriorityQueuePool.m_priorityQueueSize; i++)
	{
		PathFinderNode& node = InGridInfo.GetNodeAt(PathFinderPriorityQueuePool.m_priorityQueueData[i].m_nodeIdx);
		node.m_paretnNode = Vector2Int::InvalidIdx;
		node.m_gCost = FLOAT_MAX;
	}

	for (int i = 0; i < PathFinderCloseListPool.m_closeListSize; i++)
	{
		PathFinderNode& node = InGridInfo.GetNodeAt(PathFinderCloseListPool.m_closeListData[i]);
		node.m_onCloseList = false;
		node.m_paretnNode = Vector2Int::InvalidIdx;
		node.m_gCost = FLOAT_MAX;
	}

	if (isOutPathMax)
		return PathfindResult::PathResultPoolOverflow;
	else if (isPathFound)
		return PathfindResult::Found;
	else if (isPriorityQueueMax)
		return PathfindResult::PriorityQueuePoolOverflow;
	else if (isCloseListMax)
		return PathfindResult::CloseListPoolOverflow;
	else
		return PathfindResult::NotFound;
}

