-- ======================================================================
-- Copyright (c) 2012 RapidFire Studio Limited 
-- All Rights Reserved. 
-- http://www.rapidfirestudio.com

-- Permission is hereby granted, free of charge, to any person obtaining
-- a copy of this software and associated documentation files (the
-- "Software"), to deal in the Software without restriction, including
-- without limitation the rights to use, copy, modify, merge, publish,
-- distribute, sublicense, and/or sell copies of the Software, and to
-- permit persons to whom the Software is furnished to do so, subject to
-- the following conditions:

-- The above copyright notice and this permission notice shall be
-- included in all copies or substantial portions of the Software.

-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
-- EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
-- MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
-- IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
-- CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
-- TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
-- SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
-- ======================================================================

AStar = {}

local INF = 1 / 0

function euclideanDistance(nodeA, nodeB)
  return math.sqrt(
    math.pow(nodeB.x - nodeA.x, 2) +
      math.pow(nodeB.y - nodeA.y, 2) +
      math.pow(nodeB.z - nodeA.z, 2)
  )
end

function euclideanDistance2D(nodeA, nodeB)
  return math.sqrt(
    math.pow(nodeB.x - nodeA.x, 2) +
      math.pow(nodeB.y - nodeA.y, 2)
  )
end

function manhattanDistance(nodeA, nodeB)
  return (
    math.abs(nodeB.x - nodeA.x) +
      math.abs(nodeB.y - nodeA.y) +
      math.abs(nodeB.z - nodeA.z)
  )
end

function manhattanDistance2d(nodeA, nodeB)
  return (
    math.abs(nodeB.x - nodeA.x) +
      math.abs(nodeB.y - nodeA.y)
  )
end

--local RIDE_SPEED = (100 + 100) / 100
local FLIGHT_SPEED = (100 + 310) / 100
-- local SWIM_SPEED =

local function distanceBetween(nodeA, nodeB)
  local distance = euclideanDistance(nodeA, nodeB)
  if nodeA.isInAir and nodeB.isInAir then
    distance = distance / FLIGHT_SPEED
    --elseif nodeA.isOutdoors and nodeB.isOutdoors then
    --  distance = distance / RIDE_SPEED
  end
  return distance
end

local estimateCostWithHeuristic = distanceBetween

local function unwind(path, node)
  -- log('unwind')
  -- log(node)
  if node.pathIndex then
    local subPath = Movement.retrievePath(node.pathIndex)
    -- log('reused path with length: ' .. #subPath)
    path = Array.concat(subPath, path)
  else
    table.insert(path, 1, node)
  end
  return path
end

local function unwindPath(path, cameFrom, nodeTo)
  -- log('nodeTo')
  -- log(nodeTo)
  local node = cameFrom[nodeTo]
  -- log('node')
  -- log(node)
  if node then
    path = unwind(path, node)
    return unwindPath(path, cameFrom, path[1])
  else
    return path
  end
end

local function areClose(a, b, toleranceDistance)
  toleranceDistance = toleranceDistance or 3 -- 0.25
  return euclideanDistance(a, b) <= toleranceDistance
end

local function hasFound(a, b, pathBetweenPoints, toleranceDistance)
  return pathBetweenPoints or areClose(a, b, toleranceDistance)
end

local function lessThan(a, b)
  return a < b
end

function AStar.canPathBeMoved(path)
  if #path <= 1 then
    return true
  end

  for index = 2, #path do
    local from = path[index - 1]
    local to = path[index]
    local canMoveFromPointToPoint = Movement.canBeMovedFromPointToPoint(from, to)
    print('canMoveFromPointToPoint', index - 1, index, canMoveFromPointToPoint)
    if not canMoveFromPointToPoint then
      return false
    end
  end

  return true
end

-- TODO: Passing continentID via from and to.
-- TODO: Make sure that from.continentID and to.continentID are equal (as long as findPath only supports pathfinding in one continent).
function findPath(from, to, retrieveNeighborNodes, a, yielder, toleranceDistance)
  -- log('findPath')
  toleranceDistance = toleranceDistance or 0

  print(1)
  if not GMR.IsMeshLoaded() then
    print(2)
    GMR.LoadMeshFiles()
    print(3)
  end
  print(4)

  aStarPoints = {}

  local startTime = debugprofilestop()
  local numberOfNeighborProcessed = 0

  local points = PointToValueMap:new()

  local function reusePoint(point)
    local storedPoint = points:retrieveValue(point)
    if storedPoint then
      return storedPoint
    else
      points:setValue(point, point)
      return point
    end
  end

  from = reusePoint(from)
  to = reusePoint(to)

  local closedSet = PointToValueMap:new()
  local openSet = BinaryHeap.minUnique(lessThan)
  local cameFrom = {}

  local gScore = {}
  gScore[from] = 0
  local fScore = gScore[from] + estimateCostWithHeuristic(from, to)
  openSet:insert(fScore, from)

  while openSet:peekValue() ~= nil do
    local current = reusePoint(openSet:pop())

    -- aStarPoints = { current }
    if aStarPoints then
      table.insert(aStarPoints, current)
      if #aStarPoints > 50 then
        aStarPoints = Array.slice(aStarPoints, #aStarPoints - 50)
      end
    end

    local pathBetweenPoints = HWT.FindPath(Movement.retrieveContinentID(), current.x, current.y, current.z, to.x, to.y, to.z, false, 1024, 0, toleranceDistance, false)
    if pathBetweenPoints then
      pathBetweenPoints = Movement.convertGMRPathToPath(pathBetweenPoints)
      if Array.all(pathBetweenPoints, Movement.isPositionInRangeForTraceLineChecks) and not AStar.canPathBeMoved(pathBetweenPoints) then
        pathBetweenPoints = nil
      end
    end
    local hasFound2 = hasFound(current, to, pathBetweenPoints, toleranceDistance)
    if hasFound2 then
      local path = {}
      path = unwind(path, current)
      local numberOfPointsFromGMR
      if pathBetweenPoints then
        -- log('#pathBetweenPoints', #pathBetweenPoints)
        local points = pathBetweenPoints
        if not (
          Movement.havePointsSameCoordinates(current, points[1]) and
            Movement.havePointsSameCoordinates(current, points[#points])
        ) then
          if Movement.havePointsSameCoordinates(current, points[1]) then
            points = Array.slice(points, 2)
          end
          Array.append(path, points)
          numberOfPointsFromGMR = #points
        end
      else
        numberOfPointsFromGMR = 0
      end
      path = unwindPath(path, cameFrom, path[1])

      -- log('number of neighbors / second', numberOfNeighborProcessed / ((debugprofilestop() - startTime) / 1000))

      local subPathWhichHasBeenGeneratedWithMovementPoints = Array.slice(path, 1, #path - numberOfPointsFromGMR)

      return path, subPathWhichHasBeenGeneratedWithMovementPoints
    end

    closedSet:setValue(current, true)

    local neighbors = Array.map(retrieveNeighborNodes(current), reusePoint)
    for index = 1, Array.length(neighbors) do
      local neighbor = neighbors[index]
      if not closedSet:retrieveValue(neighbor) then
        -- table.insert(aStarPoints, neighbor)
        -- log('neighbor', neighbor)
        local tentativeGScore = gScore[current] + distanceBetween(current, neighbor)
        local neighborValueInOpenSet = openSet:valueByPayload(neighbor)
        local isNeighborInOpenSet = neighborValueInOpenSet ~= nil
        if not isNeighborInOpenSet or tentativeGScore < gScore[neighbor] then
          cameFrom[neighbor] = current
          gScore[neighbor] = tentativeGScore
          local fScore = gScore[neighbor] + estimateCostWithHeuristic(neighbor, to)
          if isNeighborInOpenSet then
            if neighborValueInOpenSet ~= fScore then
              openSet:update(neighbor, fScore)
            end
          else
            openSet:insert(fScore, neighbor)
          end
        end

        numberOfNeighborProcessed = numberOfNeighborProcessed + 1
      end
    end

    if a.shouldStop() then
      -- log('adjklasjdkl', (maximumTimeToRun and debugprofilestop() - startTime >= maximumTimeToRun), a.shouldStop())
      -- log('number of neighbors / second', numberOfNeighborProcessed / ((debugprofilestop() - startTime) / 1000))
      return nil, nil
    end

    if yielder.hasRanOutOfTime() then
      yielder.yield()
    end
  end

  -- log('number of neighbors / second', numberOfNeighborProcessed / ((debugprofilestop() - startTime) / 1000))
  return nil, nil
end
