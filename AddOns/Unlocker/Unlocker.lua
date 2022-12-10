Unlocker = {}

Unlocker.QuestGiverStatuses = {
  DailyQuest = 9,
  Quest = 10
}

function Unlocker.retrieveQuestGiverStatus(object)
  return HWT and HWT.ObjectQuestGiverStatus(object)
end

local objectIDsToQuests2 = {
  [165505] = { 29548 }
}

function Unlocker.ObjectIsQuestObjective(object)
  local objectID = HWT.ObjectId(object)
  local quests = objectIDsToQuests2[objectID]
  if quests then
    return Array.any(quests, Compatibility.QuestLog.isOnQuest)
  else
    return HWT.ObjectIsQuestObjective(object, false)
  end
end

local objectIDsToQuests = {
  -- seems to crash the game
  [209436] = {
    [29619] = {
      [1] = true
    }
  },
  [209463] = {
    [29627] = {
      [1] = true
    }
  },
  [203972] = {
    [26230] = {
      [2] = true
    }
  },
  --[] = {
  --  [] = {
  --    [1] = true
  --  }
  --},
  -- optional quest objective
  [165505] = {
    [29548] = {
      [1] = true
    }
  },
}

function Unlocker.ObjectQuests(object)
  local objectID = HWT.ObjectId(object)
  local quests = objectIDsToQuests[objectID]
  if quests then
    local questIDs = Set.create()
    for questID, objectives in pairs(quests) do
      if GMR.IsQuestActive(questID) then
        for objectiveIndex in pairs(objectives) do
          if not GMR.Questing.IsObjectiveCompleted(questID, objectiveIndex) then
            Set.add(questIDs, questID)
            break
          end
        end
      end
    end
    return Set.toList(questIDs)
  elseif Unlocker.ObjectIsQuestObjective(object) then
    local gameObjectType = HWT.GameObjectType(object)
    if gameObjectType == 4 or gameObjectType == 3 then -- for some containers and quest giver objects the game seems to crash when ObjectQuests is called with their pointer
      return {}
    else
      return { HWT.ObjectQuests(object) }
    end
  else
    return {}
  end
end
