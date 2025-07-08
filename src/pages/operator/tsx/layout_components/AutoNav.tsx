import React, { useEffect, useState, useCallback, Dispatch, SetStateAction } from 'react'

import { Canvas } from "../static_components/Canvas";
import { Map } from './Map';
import { ComponentType, MapDefinition } from '../utils/component_definitions';
import { SharedState } from './CustomizableComponent';
import FooterAutoNav from './FooterAutoNav';
import { mapFunctionProvider } from 'operator/tsx/index';
import { OccupancyGrid } from '../static_components/OccupancyGrid';
import { underMapFunctionProvider } from 'operator/tsx/index';
import { UnderMapButton } from '../function_providers/UnderMapFunctionProvider';
import {
    ROSOccupancyGrid,
    ROSPose,
    className,
} from 'shared/util';

interface AutoNavProps {
    sharedState: SharedState;
    isAutoNavHidden: boolean;
    isAutoNavHiddenSet: Dispatch<SetStateAction<boolean>>;
}

enum MapFunction {
    GetMap,
    GetPose,
    MoveBase,
    GoalReached,
}

export interface AutoNavFunctions {
    SelectGoal: (toggle: boolean) => void;
    CancelGoal: () => void;
    DeleteGoal: (goalId: number) => void;
    SaveGoal: (locationName: string) => void;
    LoadGoal: (goalID: number) => void;
    GetPose: () => ROSLIB.Transform;
    GetSavedPoseNames: () => string[];
    GetSavedPoseTypes: () => string[];
    GetSavedPoses: () => ROSLIB.Transform[];
    DisplayPoseMarkers: (
        toggle: boolean,
        poses: ROSLIB.Transform[],
        poseNames: string[],
        poseTypes: string[],
    ) => void;
    DisplayGoalMarker: (pose: ROSLIB.Vector3) => void;
    NavigateToAruco: (goalID: number) => void;
    Play: () => void;
    RemoveGoalMarker: () => void;
    GoalReached: () => Promise<boolean>;
}

interface MapFunctions {
    GetMap: ROSOccupancyGrid;
    GetPose: () => ROSLIB.Transform;
    MoveBase: (pose: ROSPose) => void;
    GoalReached: () => boolean;
    SelectGoal: () => boolean;
    SetSelectGoal: (selectGoal: boolean) => void;
}


const AutoNav: React.FC<AutoNavProps> = ({
    sharedState,
    isAutoNavHidden,
    isAutoNavHiddenSet,
}) => {
    const [occupancyGrid, occupancyGridSet] = useState<OccupancyGrid | undefined>();
    const functs: AutoNavFunctions = {
        SelectGoal: underMapFunctionProvider.provideFunctions(
            UnderMapButton.SelectGoal,
        ) as (toggle: boolean) => void,
        CancelGoal: underMapFunctionProvider.provideFunctions(
            UnderMapButton.CancelGoal,
        ) as () => void,
        DeleteGoal: underMapFunctionProvider.provideFunctions(
            UnderMapButton.DeleteGoal,
        ) as (goalId: number) => void,
        SaveGoal: underMapFunctionProvider.provideFunctions(
            UnderMapButton.SaveGoal,
        ) as (locationName: string) => void,
        LoadGoal: underMapFunctionProvider.provideFunctions(
            UnderMapButton.LoadGoal,
        ) as (goalID: number) => void,
        GetPose: underMapFunctionProvider.provideFunctions(
            UnderMapButton.GetPose,
        ) as () => ROSLIB.Transform,
        GetSavedPoseNames: underMapFunctionProvider.provideFunctions(
            UnderMapButton.GetSavedPoseNames,
        ) as () => string[],
        GetSavedPoseTypes: underMapFunctionProvider.provideFunctions(
            UnderMapButton.GetSavedPoseTypes,
        ) as () => string[],
        GetSavedPoses: underMapFunctionProvider.provideFunctions(
            UnderMapButton.GetSavedPoses,
        ) as () => ROSLIB.Transform[],
        DisplayPoseMarkers: (
            toggle: boolean,
            poses: ROSLIB.Transform[],
            poseNames: string[],
            poseTypes: string[],
        ) => {
            return occupancyGrid!.displayPoseMarkers(
                toggle,
                poses,
                poseNames,
                poseTypes,
            );
        },
        DisplayGoalMarker: (pose: ROSLIB.Vector3) =>
            occupancyGrid!.createGoalMarker(pose.x, pose.y, true),
        NavigateToAruco: underMapFunctionProvider.provideFunctions(
            UnderMapButton.NavigateToAruco,
        ) as (goalID: number) => void,
        Play: () => occupancyGrid!.play(),
        RemoveGoalMarker: () => occupancyGrid!.removeGoalMarker(),
        GoalReached: underMapFunctionProvider.provideFunctions(
            UnderMapButton.GoalReached,
        ) as () => Promise<boolean>,
    };

    const [poses, posesSet] = useState<string[]>(
        functs.GetSavedPoseNames(),
    );
    const handleSelectGoal = useCallback((selectGoal1: boolean) => {
        selectGoal1Set(selectGoal1);
        mapFn.SelectGoal = (): boolean => {
            return selectGoal1;
        };
    }, []);
    const mapFn: MapFunctions = {
        GetMap: mapFunctionProvider.provideFunctions(
            MapFunction.GetMap,
        ) as ROSOccupancyGrid,
        GetPose: mapFunctionProvider.provideFunctions(
            MapFunction.GetPose,
        ) as () => ROSLIB.Transform,
        MoveBase: mapFunctionProvider.provideFunctions(
            MapFunction.MoveBase,
        ) as (pose: ROSPose) => void,
        GoalReached: mapFunctionProvider.provideFunctions(
            MapFunction.GoalReached,
        ) as () => boolean,
        SelectGoal: (): boolean => {
            return selectGoal1;
        },
        SetSelectGoal: (selectGoal1: boolean) => {
            handleSelectGoal(selectGoal1);
        },
    };
    const [isModalAddLocationVisible, isModalAddLocationVisibleSet] = useState<boolean>(false);
    const [isModalLocationsMenuVisible, isModalLocationsMenuVisibleSet] = useState<boolean>(false);

    // Navigation goal selection state
    const [selectGoal1, selectGoal1Set] = useState<boolean>(false);

    // For the second goal selection, we use a different state variable
    const [selectGoal2, selectGoal2Set] = useState<boolean>(false);
    const [displayGoals, displayGoalsSet] = useState<boolean>(false);

    useEffect(() => {
        let map = mapFn.GetMap;
        let width = map ? map.info.width : 60;
        let height = map ? map.info.height : 100;
        var canvas = new Canvas({
            divID: "map",
            className: "mapCanvas",
            width: width * 5, // Scale width to avoid blurriness when making map larger
            height: height * 5, // Scale height to avoid blurriness when making map larger
        });
        var occupancyGridMap = new OccupancyGrid({
            functs: mapFn,
            rootObject: canvas.scene!,
        });
        canvas.scaleToDimensions(
            occupancyGridMap.width,
            occupancyGridMap.height,
        );
        occupancyGridSet(occupancyGridMap);
    }, []);

    return (
        <div className='auto-nav'>
            <Map
                {...{
                    path: "",
                    definition: {
                        type: ComponentType.Map,
                        selectGoal: false,
                    } as MapDefinition,
                    sharedState: sharedState,
                }}
            />
            <FooterAutoNav
                isAutoNavHiddenSet={isAutoNavHiddenSet}
                handleSelectGoal={handleSelectGoal}
                selectGoal2={selectGoal2}
                selectGoal2Set={selectGoal2Set}
                functs={functs}
                poses={poses}
                posesSet={posesSet}
                displayGoals={displayGoals}
                isModalAddLocationVisible={isModalAddLocationVisible}
                isModalAddLocationVisibleSet={isModalAddLocationVisibleSet}
                isModalLocationsMenuVisible={isModalLocationsMenuVisible}
                isModalLocationsMenuVisibleSet={isModalLocationsMenuVisibleSet}
            />
        </div>
    )
}

export default AutoNav