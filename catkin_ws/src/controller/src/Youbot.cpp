#include "Youbot.h"

FSM(Youbot){
FSM_STATES
{
    AnalyzeScan,
    Drive,
    TurnToTarget,
    PickUp,
    Initialize,
    DriveAround,
    Stop,
    Detection,
    AnalyzeLoad,
    DriveToTruck,
    DriveToContainer,
    PlaceToTruck,
    PlaceToContainer
}
FSM_START(Initialize);
FSM_BGN
{
    FSM_STATE(Initialize)
    {
        FSM_CALL_TASK(Initialize)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/DONE", FSM_NEXT(AnalyzeScan))
        }
    }
    FSM_STATE(AnalyzeScan)
    {
        FSM_CALL_TASK(AnalyzeScan)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/NOTHING_FOUND", FSM_NEXT(DriveAround))
            FSM_ON_EVENT("/TARGET_FOUND", FSM_NEXT(Drive))
        }
    }
    FSM_STATE(Drive)
    {
        FSM_CALL_TASK(Drive)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/TARGET_REACHED", FSM_NEXT(TurnToTarget))
        }
    }
    FSM_STATE(DriveAround)
    {
        FSM_CALL_TASK(DriveAround)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/NEW_OBJECT", FSM_NEXT(AnalyzeScan))
            FSM_ON_EVENT("/TIMEOUT", FSM_NEXT(Stop))
        }
    }
    FSM_STATE(TurnToTarget)
    {
        FSM_CALL_TASK(TurnToTarget)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/TURNED", FSM_NEXT(Detection))
        }
    }
    FSM_STATE(Detection)
    {
        FSM_CALL_TASK(Detection)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/DANGER", FSM_NEXT(AnalyzeScan))
            FSM_ON_EVENT("/SAFE", FSM_NEXT(PickUp))
        }
    }
    FSM_STATE(PickUp)
    {
        FSM_CALL_TASK(PickUp)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/FREE_SPACE", FSM_NEXT(AnalyzeScan))
            FSM_ON_EVENT("/FULL", FSM_NEXT(AnalyzeLoad))
        }
    }
    FSM_STATE(AnalyzeLoad)
    {
        FSM_CALL_TASK(AnalyzeLoad)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/BOTH", FSM_NEXT(DriveToContainer))
            FSM_ON_EVENT("/TRUCK_ONLY", FSM_NEXT(DriveToTruck))
            FSM_ON_EVENT("/CONTAINER_ONLY", FSM_NEXT(DriveToContainer))
            FSM_ON_EVENT("/EMPTY", FSM_NEXT(AnalyzeScan))
        }
    }
    FSM_STATE(DriveToTruck)
    {
        FSM_CALL_TASK(DriveToTruck)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/TRUCK_REACHED", FSM_NEXT(PlaceToTruck))
        }
    }
    FSM_STATE(DriveToContainer)
    {
        FSM_CALL_TASK(DriveToContainer)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/CONTAINER_REACHED", FSM_NEXT(PlaceToContainer))
        }
    }
    FSM_STATE(PlaceToTruck)
    {
        FSM_CALL_TASK(PlaceToTruck)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/PLACED", FSM_NEXT(AnalyzeLoad))
        }
    }
    FSM_STATE(PlaceToContainer)
    {
        FSM_CALL_TASK(PlaceToContainer)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/PLACED", FSM_NEXT(AnalyzeLoad))
        }
    }
    FSM_STATE(Stop)
    {
        FSM_CALL_TASK(Stop)
        FSM_TRANSITIONS
        {

        }
    }
}
FSM_END
}