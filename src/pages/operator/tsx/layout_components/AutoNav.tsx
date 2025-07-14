import React, { useEffect, useState, useCallback, Dispatch, SetStateAction } from 'react'

import { Canvas } from "../static_components/Canvas";
import { Map } from './Map';
import { ComponentType, MapDefinition } from '../utils/component_definitions';
import genUUID from '../utils/genUUID';
import { SharedState } from './CustomizableComponent';
import FooterAutoNav from './FooterAutoNav';
import Toasts, { Toast } from './Toasts';
import { mapFunctionProvider } from 'operator/tsx/index';
import { OccupancyGrid } from '../static_components/OccupancyGrid';
import { underMapFunctionProvider } from 'operator/tsx/index';
import { UnderMapButton } from '../function_providers/UnderMapFunctionProvider';
import {
    ROSOccupancyGrid,
    ROSPose,
    ROSPoint,
    className,
} from 'shared/util';

interface AutoNavProps {
    sharedState: SharedState;
    swipeableViewsIdx: number;
    swipeableViewsIdxSet: Dispatch<SetStateAction<number>>;
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
    DeleteMapPose: (poseName: string) => void;
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
    RenamePose: (poseNameOld: string, poseNameNew: string) => void;
}

export interface MapFunctions {
    GetMap: ROSOccupancyGrid;
    GetPose: () => ROSLIB.Transform;
    MoveBase: (pose: ROSPose) => void;
    GoalReached: () => boolean;
    SelectGoal: () => boolean;
    SetSelectGoal: (selectGoal: boolean) => void;
}

/**
 * AutoNav component provides the autonomous navigation interface for the operator.
 * It manages map display, pose/goal management, and footer controls for navigation.
 *
 * Props:
 *  - sharedState: SharedState object for global operator state
 */
const AutoNav: React.FC<AutoNavProps> = ({
    sharedState,
    swipeableViewsIdx,
    swipeableViewsIdxSet,
}) => {

    const [selectedLocationMenuItemIdx, selectedLocationMenuItemIdxSet] = useState<number | -1>(-1);
    const [goalPosition, goalPositionSet] = useState<ROSPoint | undefined>(undefined);

    // Toast notifications
    const [toasts, toastsSet] = useState<Toast[]>([]);

    // OccupancyGrid instance for map and marker operations
    const [occupancyGrid, occupancyGridSet] = useState<OccupancyGrid>();

    // Subscribe to goal position updates from the OccupancyGrid
    useEffect(() => {
        const callback = (pos: ROSPoint | undefined) => {
            goalPositionSet(pos);
        };
        const unsubscribeOnUnmount = occupancyGrid?.goalPositionSubscribe(callback);
        return () => {
            // Make sure to unsubscribe when <AutoNav> component unmounts
            if (unsubscribeOnUnmount) unsubscribeOnUnmount();
        };
    }, [occupancyGrid]);

    // Function to add a toast notification
    const addToast = (
        type: 'success' | 'error' | 'info',
        message: string,
        duration?: number
    ) => {
        const id = genUUID();
        toastsSet((prevToasts) => (
            [...prevToasts, { id, type, message, duration }]
        ));
    };

    /**
     * All navigation-related functions, provided by underMapFunctionProvider.
     * Many of these are ROS actions or service calls.
     * Some functions require occupancyGrid to be set.
     */
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
        DeleteMapPose: underMapFunctionProvider.provideFunctions(
            UnderMapButton.DeleteMapPose,
        ) as (poseName: string) => void,
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

        /**
         * Display pose markers on the map. Requires occupancyGrid to be set.
         */
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
        /**
         * Display a goal marker on the map at the given pose.
         */
        DisplayGoalMarker: (pose: ROSLIB.Vector3) =>
            occupancyGrid!.createGoalMarker(pose.x, pose.y, true),
        NavigateToAruco: underMapFunctionProvider.provideFunctions(
            UnderMapButton.NavigateToAruco,
        ) as (goalID: number) => void,
        /**
         * Play the current navigation sequence (if supported by occupancyGrid).
         */
        Play: () => occupancyGrid!.play(),
        RemoveGoalMarker: () => occupancyGrid!.removeGoalMarker(),
        GoalReached: underMapFunctionProvider.provideFunctions(
            UnderMapButton.GoalReached,
        ) as () => Promise<boolean>,
        RenamePose: underMapFunctionProvider.provideFunctions(
            UnderMapButton.RenamePose,
        ) as (poseNameOld: string, poseNameNew: string) => void,
    };

    underMapFunctionProvider.setMapPoseCallback((pose: ROSLIB.Vector3) => {
        functs.DisplayGoalMarker(pose);
        functs.NavigateToAruco(selectedLocationMenuItemIdx);
        isCurrentlyMovingSet(true);
        isSelectingGoalSet(false);
        functs
            .GoalReached()
            .then((goalReached) => isCurrentlyMovingSet(false));
    })

    /**
     * Callback to update the goal selection state and update mapFn.SelectGoal.
     */
    const handleSelectGoal = (isSelectingGoal: boolean) => {
        isSelectingGoalSet(isSelectingGoal);
        mapFn.SelectGoal = (): boolean => {
            return isSelectingGoal;
        };
    };

    /**
     * Map-related functions for interacting with the map and robot pose.
     * These are provided by mapFunctionProvider.
     */
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
        /**
         * Returns whether a goal is currently being selected.
         */
        SelectGoal: (): boolean => {
            return isSelectingGoal;
        },
        /**
         * Sets the goal selection state.
         */
        SetSelectGoal: (isSelectingGoal: boolean) => {
            handleSelectGoal(isSelectingGoal);
        },
    };

    // Modal visibility state for adding a location
    const [isModalAddLocationVisible, isModalAddLocationVisibleSet] = useState<boolean>(false);
    // Modal visibility state for locations menu
    const [isModalLocationsMenuVisible, isModalLocationsMenuVisibleSet] = useState<boolean>(false);
    // Whether to display all goal markers on the map
    const [displayGoals, displayGoalsSet] = useState<boolean>(false);
    // Navigation goal selection state (true if selecting a goal).
    const [isSelectingGoal, isSelectingGoalSet] = useState<boolean>(true);
    // Whether the robot is currently auto-navigating
    const [isCurrentlyMoving, isCurrentlyMovingSet] = useState<boolean>(false);

    /**
     * On mount, create the canvas and OccupancyGrid for the map.
     * This sets up the map rendering and interaction logic.
     */
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
        var occupancyGrid = new OccupancyGrid({
            functs: mapFn,
            rootObject: canvas.scene!,
        });
        canvas.scaleToDimensions(
            occupancyGrid.width,
            occupancyGrid.height,
        );
        occupancyGridSet(occupancyGrid);
    }, []);

    useEffect(() => {
        console.log('occupancyGrid', occupancyGrid)
    }, [occupancyGrid]);

    // Show toast when user navigates to AutoNav
    useEffect(() => {
        if (swipeableViewsIdx === 1) addToast('info', 'Click on the map to navigate');
    }, [swipeableViewsIdx])

    return (
        <div className='auto-nav'>
            {/* Map display for navigation */}
            <div
                style={{ padding: '10px 0px' }}
            >
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
            </div>
            {/* Footer controls for navigation and pose management */}
            <FooterAutoNav
                handleSelectGoal={handleSelectGoal}
                functs={functs}
                isModalAddLocationVisible={isModalAddLocationVisible}
                isModalAddLocationVisibleSet={isModalAddLocationVisibleSet}
                isModalLocationsMenuVisible={isModalLocationsMenuVisible}
                isModalLocationsMenuVisibleSet={isModalLocationsMenuVisibleSet}
                isCurrentlyMoving={isCurrentlyMoving}
                isCurrentlyMovingSet={isCurrentlyMovingSet}
                isSelectingGoal={isSelectingGoal}
                isSelectingGoalSet={isSelectingGoalSet}
                selectedLocationMenuItemIdx={selectedLocationMenuItemIdx}
                swipeableViewsIdxSet={swipeableViewsIdxSet}
                goalPosition={goalPosition}
                addToast={addToast}
            />
            <Toasts toasts={toasts} toastsSet={toastsSet} />
        </div>
    );
};

export default AutoNav;