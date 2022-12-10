:: It seems required to run this script as administrator because that seems required from symlinking.

set "path=C:\Program Files (x86)\World of Warcraft"

if exist "%path%\_retail_" (
	if not exist "%path%\_retail_\Interface" mkdir "%path%\_retail_\Interface"
	if not exist "%path%\_retail_\Interface\AddOns" mkdir "%path%\_retail_\Interface\AddOns"
	mklink /D "%path%\_retail_\Interface\AddOns\ActionSequenceDoer" "%~dp0\AddOns\ActionSequenceDoer"
	mklink /D "%path%\_retail_\Interface\AddOns\Array" "%~dp0\AddOns\Array"
	mklink /D "%path%\_retail_\Interface\AddOns\AStar" "%~dp0\AddOns\AStar"
	mklink /D "%path%\_retail_\Interface\AddOns\BinaryHeap" "%~dp0\AddOns\BinaryHeap"
	mklink /D "%path%\_retail_\Interface\AddOns\Boolean" "%~dp0\AddOns\Boolean"
	mklink /D "%path%\_retail_\Interface\AddOns\Coroutine" "%~dp0\AddOns\Coroutine"
	mklink /D "%path%\_retail_\Interface\AddOns\Events" "%~dp0\AddOns\Events"
	mklink /D "%path%\_retail_\Interface\AddOns\Function" "%~dp0\AddOns\Function"
	mklink /D "%path%\_retail_\Interface\AddOns\Math" "%~dp0\AddOns\Math"
	mklink /D "%path%\_retail_\Interface\AddOns\Movement" "%~dp0\AddOns\Movement"
	mklink /D "%path%\_retail_\Interface\AddOns\Object" "%~dp0\AddOns\Object"
	mklink /D "%path%\_retail_\Interface\AddOns\OffMeshHandler" "%~dp0\AddOns\OffMeshHandler"
	mklink /D "%path%\_retail_\Interface\AddOns\Set" "%~dp0\AddOns\Set"
	mklink /D "%path%\_retail_\Interface\AddOns\Vector" "%~dp0\AddOns\Vector"
	mklink /D "%path%\_retail_\Interface\AddOns\Yielder" "%~dp0\AddOns\Yielder"
	mklink /D "%path%\_retail_\Interface\AddOns\GMR" "%~dp0\AddOns\GMR"
	mklink /D "%path%\_retail_\Interface\AddOns\HWT" "%~dp0\AddOns\HWT"
	mklink /D "%path%\_retail_\Interface\AddOns\Unlocker" "%~dp0\AddOns\Unlocker"
	mklink /D "%path%\_retail_\Interface\AddOns\Core" "%~dp0\AddOns\Core"
	mklink /D "%path%\_retail_\Interface\AddOns\Serialization" "%~dp0\AddOns\Serialization"
	mklink /D "%path%\_retail_\Interface\AddOns\APIDumper" "%~dp0\AddOns\APIDumper"
	mklink /D "%path%\_retail_\Interface\AddOns\Bot" "%~dp0\AddOns\Bot"
	mklink /D "%path%\_retail_\Interface\AddOns\AutoGear" "%~dp0\AddOns\AutoGear"
	mklink /D "%path%\_retail_\Interface\AddOns\Tooltips" "%~dp0\AddOns\Tooltips"
	mklink /D "%path%\_retail_\Interface\AddOns\Questing" "%~dp0\AddOns\Questing"
	mklink /D "%path%\_retail_\Interface\AddOns\APICallLogging" "%~dp0\AddOns\APICallLogging"
	mklink /D "%path%\_retail_\Interface\AddOns\Hooking" "%~dp0\AddOns\Hooking"
	mklink /D "%path%\_retail_\Interface\AddOns\Conditionals" "%~dp0\AddOns\Conditionals"
	mklink /D "%path%\_retail_\Interface\AddOns\HWTRetriever" "%~dp0\AddOns\HWTRetriever"
	mklink /D "%path%\_retail_\Interface\AddOns\Draw" "%~dp0\AddOns\Draw"
	mklink /D "%path%\_retail_\Interface\AddOns\Scheduling" "%~dp0\AddOns\Scheduling"
	mklink /D "%path%\_retail_\Interface\AddOns\String" "%~dp0\AddOns\String"
	mklink /D "%path%\_retail_\Interface\AddOns\Float" "%~dp0\AddOns\Float"
	:: mklink /D "%path%\_retail_\Interface\AddOns\DisableGMR" "%~dp0\AddOns\DisableGMR"
	mklink /D "%path%\_retail_\Interface\AddOns\!Fixes" "%~dp0\retail\AddOns\!Fixes"
)

if exist "%path%\_classic_" (
	if not exist "%path%\_classic_\Interface" mkdir "%path%\_classic_\Interface"
	if not exist "%path%\_classic_\Interface\AddOns" mkdir "%path%\_classic_\Interface\AddOns"
	mklink /D "%path%\_classic_\Interface\AddOns\ActionSequenceDoer" "%~dp0\AddOns\ActionSequenceDoer"
	mklink /D "%path%\_classic_\Interface\AddOns\Array" "%~dp0\AddOns\Array"
	mklink /D "%path%\_classic_\Interface\AddOns\AStar" "%~dp0\AddOns\AStar"
	mklink /D "%path%\_classic_\Interface\AddOns\BinaryHeap" "%~dp0\AddOns\BinaryHeap"
	mklink /D "%path%\_classic_\Interface\AddOns\Boolean" "%~dp0\AddOns\Boolean"
	mklink /D "%path%\_classic_\Interface\AddOns\Coroutine" "%~dp0\AddOns\Coroutine"
	mklink /D "%path%\_classic_\Interface\AddOns\Events" "%~dp0\AddOns\Events"
	mklink /D "%path%\_classic_\Interface\AddOns\Function" "%~dp0\AddOns\Function"
	mklink /D "%path%\_classic_\Interface\AddOns\Math" "%~dp0\AddOns\Math"
	mklink /D "%path%\_classic_\Interface\AddOns\Movement" "%~dp0\AddOns\Movement"
	mklink /D "%path%\_classic_\Interface\AddOns\Object" "%~dp0\AddOns\Object"
	mklink /D "%path%\_classic_\Interface\AddOns\OffMeshHandler" "%~dp0\AddOns\OffMeshHandler"
	mklink /D "%path%\_classic_\Interface\AddOns\Set" "%~dp0\AddOns\Set"
	mklink /D "%path%\_classic_\Interface\AddOns\Vector" "%~dp0\AddOns\Vector"
	mklink /D "%path%\_classic_\Interface\AddOns\Yielder" "%~dp0\AddOns\Yielder"
	mklink /D "%path%\_classic_\Interface\AddOns\GMR" "%~dp0\AddOns\GMR"
	mklink /D "%path%\_classic_\Interface\AddOns\HWT" "%~dp0\AddOns\HWT"
	mklink /D "%path%\_classic_\Interface\AddOns\Unlocker" "%~dp0\AddOns\Unlocker"
	mklink /D "%path%\_classic_\Interface\AddOns\Core" "%~dp0\AddOns\Core"
	mklink /D "%path%\_classic_\Interface\AddOns\Serialization" "%~dp0\AddOns\Serialization"
	mklink /D "%path%\_classic_\Interface\AddOns\APIDumper" "%~dp0\AddOns\APIDumper"
	mklink /D "%path%\_classic_\Interface\AddOns\Bot" "%~dp0\AddOns\Bot"
	mklink /D "%path%\_classic_\Interface\AddOns\Tooltips" "%~dp0\AddOns\Tooltips"
	mklink /D "%path%\_classic_\Interface\AddOns\Questing" "%~dp0\AddOns\Questing"
	mklink /D "%path%\_classic_\Interface\AddOns\APICallLogging" "%~dp0\AddOns\APICallLogging"
	mklink /D "%path%\_classic_\Interface\AddOns\Hooking" "%~dp0\AddOns\Hooking"
	mklink /D "%path%\_classic_\Interface\AddOns\Conditionals" "%~dp0\AddOns\Conditionals"
	mklink /D "%path%\_classic_\Interface\AddOns\HWTRetriever" "%~dp0\AddOns\HWTRetriever"
	mklink /D "%path%\_classic_\Interface\AddOns\Draw" "%~dp0\AddOns\Draw"
	mklink /D "%path%\_classic_\Interface\AddOns\Scheduling" "%~dp0\AddOns\Scheduling"
	mklink /D "%path%\_classic_\Interface\AddOns\String" "%~dp0\AddOns\String"
	mklink /D "%path%\_classic_\Interface\AddOns\Float" "%~dp0\AddOns\Float"
	:: mklink /D "%path%\_classic_\Interface\AddOns\DisableGMR" "%~dp0\AddOns\DisableGMR"
)

if exist "%path%\_classic_era_" (
	if not exist "%path%\_classic_era_\Interface" mkdir "%path%\_classic_era_\Interface"
	if not exist "%path%\_classic_era_\Interface\AddOns" mkdir "%path%\_classic_era_\Interface\AddOns"
	mklink /D "%path%\_classic_era_\Interface\AddOns\ActionSequenceDoer" "%~dp0\AddOns\ActionSequenceDoer"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Array" "%~dp0\AddOns\Array"
	mklink /D "%path%\_classic_era_\Interface\AddOns\AStar" "%~dp0\AddOns\AStar"
	mklink /D "%path%\_classic_era_\Interface\AddOns\BinaryHeap" "%~dp0\AddOns\BinaryHeap"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Boolean" "%~dp0\AddOns\Boolean"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Coroutine" "%~dp0\AddOns\Coroutine"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Events" "%~dp0\AddOns\Events"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Function" "%~dp0\AddOns\Function"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Math" "%~dp0\AddOns\Math"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Movement" "%~dp0\AddOns\Movement"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Object" "%~dp0\AddOns\Object"
	mklink /D "%path%\_classic_era_\Interface\AddOns\OffMeshHandler" "%~dp0\AddOns\OffMeshHandler"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Set" "%~dp0\AddOns\Set"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Vector" "%~dp0\AddOns\Vector"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Yielder" "%~dp0\AddOns\Yielder"
	mklink /D "%path%\_classic_era_\Interface\AddOns\GMR" "%~dp0\AddOns\GMR"
	mklink /D "%path%\_classic_era_\Interface\AddOns\HWT" "%~dp0\AddOns\HWT"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Unlocker" "%~dp0\AddOns\Unlocker"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Core" "%~dp0\AddOns\Core"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Serialization" "%~dp0\AddOns\Serialization"
	mklink /D "%path%\_classic_era_\Interface\AddOns\APIDumper" "%~dp0\AddOns\APIDumper"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Bot" "%~dp0\AddOns\Bot"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Tooltips" "%~dp0\AddOns\Tooltips"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Questing" "%~dp0\AddOns\Questing"
	mklink /D "%path%\_classic_era_\Interface\AddOns\APICallLogging" "%~dp0\AddOns\APICallLogging"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Hooking" "%~dp0\AddOns\Hooking"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Conditionals" "%~dp0\AddOns\Conditionals"
	mklink /D "%path%\_classic_era_\Interface\AddOns\HWTRetriever" "%~dp0\AddOns\HWTRetriever"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Draw" "%~dp0\AddOns\Draw"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Scheduling" "%~dp0\AddOns\Scheduling"
	mklink /D "%path%\_classic_era_\Interface\AddOns\String" "%~dp0\AddOns\String"
	mklink /D "%path%\_classic_era_\Interface\AddOns\Float" "%~dp0\AddOns\Float"
	:: mklink /D "%path%\_classic_era_\Interface\AddOns\DisableGMR" "%~dp0\AddOns\DisableGMR"
)