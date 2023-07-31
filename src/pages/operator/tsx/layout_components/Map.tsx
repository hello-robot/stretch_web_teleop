import React from "react";
import 'latest-createjs';
import { CustomizableComponentProps, isSelected } from "./CustomizableComponent";
import { MapDefinition } from "../utils/component_definitions";
import { mapFunctionProvider, occupancyGrid } from "operator/tsx/index";
import "operator/css/Map.css"
import { Canvas } from "../static_components/Canvas";
import { OccupancyGrid } from "../static_components/OccupancyGrid";
import { AMCLPose, ROSOccupancyGrid, ROSPose, className } from "shared/util";
import ROSLIB from "roslib";
import { UnderMapButton } from "../function_providers/UnderMapFunctionProvider";
import { underMapFunctionProvider } from "operator/tsx/index";
import { CheckToggleButton } from "../basic_components/CheckToggleButton";
import { useState } from "react";
import { Dropdown } from "../basic_components/Dropdown";
import { PopupModal } from "../basic_components/PopupModal";
import { Tooltip } from "operator/tsx/static_components/Tooltip"

export enum MapFunction {
    GetMap,
    GetPose,
    MoveBase,
    GoalReached,
}

export interface MapFunctions {
    GetMap: ROSOccupancyGrid
    GetPose: () => ROSLIB.Transform
    MoveBase: (pose: ROSPose) => void
    GoalReached: () => boolean
    SelectGoal: () => boolean
}

export interface UnderMapFunctions {
    SelectGoal: (toggle: boolean) => void
    CancelGoal: () => void
    DeleteGoal: (goalId: number) => void
    SaveGoal: (name: string) => void
    LoadGoal: (goalID: number) => void
    GetPose: () => ROSLIB.Transform
    GetSavedPoseNames: () => string[]
    GetSavedPoseTypes: () => string[]
    GetSavedPoses: () => ROSLIB.Transform[]
    DisplayPoseMarkers: (toggle: boolean, poses: ROSLIB.Transform[], poseNames: string[], poseTypes: string[]) => void
    DisplayGoalMarker: (pose: ROSLIB.Vector3) => void
}

export const Map = (props: CustomizableComponentProps) => {
    const definition = props.definition as MapDefinition
    const [active, setActive] = React.useState<boolean>(false);
    const [occupancyGrid, setOccupanyGrid] = React.useState<OccupancyGrid>()
    const { customizing } = props.sharedState;
    const selected = isSelected(props);

    // Constrain the width or height when the stream gets too large
    React.useEffect(() => {
        var canvas = new Canvas({
            divID: 'map',
            className: 'mapCanvas',
            width: mapFn.GetMap.info.width * 5,  // Scale width to avoid blurriness when making map larger
            height: mapFn.GetMap.info.height * 5 // Scale height to avoid blurriness when making map larger
        });
        var occupancyGridMap = new OccupancyGrid({
            functs: mapFn,
            rootObject: canvas.scene!
        })
        canvas.scaleToDimensions(occupancyGridMap.width, occupancyGridMap.height);
        setOccupanyGrid(occupancyGridMap)
    }, []);

    function handleSelect(event: React.MouseEvent<HTMLDivElement>) {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    }

    let mapFn: MapFunctions = {
        GetMap: mapFunctionProvider.provideFunctions(MapFunction.GetMap) as ROSOccupancyGrid,
        GetPose: mapFunctionProvider.provideFunctions(MapFunction.GetPose) as () => ROSLIB.Transform,
        MoveBase: mapFunctionProvider.provideFunctions(MapFunction.MoveBase) as (pose: ROSPose) => void,
        GoalReached: mapFunctionProvider.provideFunctions(MapFunction.GoalReached) as () => boolean,
        SelectGoal: (): boolean => { console.log(definition.selectGoal!); return definition.selectGoal! }
    }

    let underMapFn: UnderMapFunctions = {
        SelectGoal: underMapFunctionProvider.provideFunctions(UnderMapButton.SelectGoal) as () => void,
        CancelGoal: underMapFunctionProvider.provideFunctions(UnderMapButton.CancelGoal) as () => void,
        DeleteGoal: underMapFunctionProvider.provideFunctions(UnderMapButton.DeleteGoal) as (goalID: number) => void,
        SaveGoal: underMapFunctionProvider.provideFunctions(UnderMapButton.SaveGoal) as (name: string) => void,
        LoadGoal: underMapFunctionProvider.provideFunctions(UnderMapButton.LoadGoal) as () => void,
        GetPose: underMapFunctionProvider.provideFunctions(UnderMapButton.GetPose) as () => ROSLIB.Transform,
        GetSavedPoseNames: underMapFunctionProvider.provideFunctions(UnderMapButton.GetSavedPoseNames) as () => string[],
        GetSavedPoseTypes: underMapFunctionProvider.provideFunctions(UnderMapButton.GetSavedPoseTypes) as () => string[],
        GetSavedPoses: underMapFunctionProvider.provideFunctions(UnderMapButton.GetSavedPoses) as () => ROSLIB.Transform[],
        DisplayPoseMarkers: (toggle: boolean, poses: ROSLIB.Transform[], poseNames: string[], poseTypes: string[]) => {
            return occupancyGrid!.displayPoseMarkers(toggle, poses, poseNames, poseTypes)
        },
        DisplayGoalMarker: (pose: ROSLIB.Vector3) => occupancyGrid!.createGoalMarker(pose.x, pose.y, true)
    }

    return (
        <div className="map-container">
            <h4 className="title">Map</h4>
            <div id="map" className={className("map", { customizing, selected, active })} onClick={handleSelect}></div>
            <div className="under-video-area">
                <UnderMapButtons definition={definition} functs={underMapFn} />
            </div>
        </div>
    )
}

/**
 * Buttons to display under the map.
 */
const UnderMapButtons = (props: { definition: MapDefinition, functs: UnderMapFunctions }) => {
    const [poses, setPoses] = useState<string[]>(props.functs.GetSavedPoseNames())
    const [selectedIdx, setSelectedIdx] = React.useState<number>();
    const [selectGoal, setSelectGoal] = React.useState<boolean>(false)
    const [displayGoals, setDisplayGoals] = React.useState<boolean>(false)
    const [showSavePoseModal, setShowSavePoseModal] = useState<boolean>(false);

    const SavePoseModal = (props: {
        functs: UnderMapFunctions,
        setShow: (show: boolean) => void,
        show: boolean
    }) => {
        const [name, setName] = React.useState<string>("");
        function handleAccept() {
            if (name.length > 0) {
                if (!poses.includes(name)) {
                    setPoses(poses => [...poses, name])
                }
                props.functs.SaveGoal(name)
                props.functs.DisplayPoseMarkers(
                    displayGoals, 
                    props.functs.GetSavedPoses(),
                    props.functs.GetSavedPoseNames(),
                    props.functs.GetSavedPoseTypes()
                )
            }
            setName("");
        }

        return (
            <PopupModal
                setShow={props.setShow}
                show={props.show}
                onAccept={handleAccept}
                id="save-pose-modal"
                acceptButtonText="Save"
                acceptDisabled={name.length < 1}
            >
                <label htmlFor="new-pose-name"><b>Save Current Pose on Map</b></label>
                <hr />
                <div className="pose-name">
                    <label>Pose Name</label>
                    <input autoFocus type="text" id="new-pose-name" name="new-option-name"
                        value={name} onChange={(e) => setName(e.target.value)}
                        placeholder="Enter name"
                    />
                </div>
            </PopupModal>
        )
    }

    function formatNamesandTypes(names: string[], types: string[]): React.JSX.Element[]{
        let elements: React.JSX.Element[] = []
        names.map((name, index) => {
            elements.push(<p><em>{types[index]}</em> {name}</p>)
        })
        return elements
    }

    return (
        <React.Fragment>
            <div className="under-video-area">
                <CheckToggleButton
                    checked={selectGoal}
                    onClick={() => {
                        props.definition.selectGoal = !selectGoal;
                        setSelectGoal(!selectGoal)
                    }}
                    label="Select Goal"
                />
                <Tooltip text="Save goal" position="top">
                    <button className="save-btn" onClick={() => setShowSavePoseModal(true)}>
                        Save
                        <span className="material-icons">
                            save
                        </span>
                    </button>
                </Tooltip>
                <Tooltip text="Cancel goal" position="top">
                    <button className="delete-btn" onClick={props.functs.CancelGoal}>
                        Cancel
                        <span className="material-icons">
                            cancel
                        </span>
                    </button>
                </Tooltip>
            </div>
            <div className="under-video-area">
                <CheckToggleButton
                    checked={displayGoals}
                    onClick={() => {
                        // props.definition.displayGoals = !displayGoals;
                        setDisplayGoals(!displayGoals)
                        props.functs.DisplayPoseMarkers(
                            !displayGoals, 
                            props.functs.GetSavedPoses(),
                            props.functs.GetSavedPoseNames(),
                            props.functs.GetSavedPoseTypes()
                        )
                    }}
                    label="Display Goals"
                />
                <Dropdown
                    onChange={setSelectedIdx}
                    selectedIndex={selectedIdx}
                    possibleOptions={formatNamesandTypes(props.functs.GetSavedPoseNames(), props.functs.GetSavedPoseTypes())}
                    placeholderText="Select a goal..."
                    placement="top"
                />
                <Tooltip text="Load goal" position="top">
                    <button className="play-btn" onClick={
                        () => {
                            if (selectedIdx != undefined) {
                                let pose: ROSLIB.Vector3 = props.functs.LoadGoal(selectedIdx)!
                                props.functs.DisplayGoalMarker(pose)
                            }
                        }
                    }>
                        Play 
                        <span className="material-icons">
                            play_circle
                        </span>
                    </button>
                </Tooltip>
                <Tooltip text="Delete goal" position="top">
                    <button className="delete-btn" onClick={() => {
                        if (selectedIdx != undefined) props.functs.DeleteGoal(selectedIdx)
                        setPoses(props.functs.GetSavedPoseNames())
                        props.functs.DisplayPoseMarkers(
                            !displayGoals, 
                            props.functs.GetSavedPoses(),
                            props.functs.GetSavedPoseNames(),
                            props.functs.GetSavedPoseTypes()
                        )
                        setSelectedIdx(undefined)
                    }}>
                        Delete 
                        <span className="material-icons">
                            delete_forever
                        </span>
                    </button>
                </Tooltip>
            </div>
            <SavePoseModal
                functs={props.functs}
                setShow={setShowSavePoseModal}
                show={showSavePoseModal}
            />
        </React.Fragment>
    )
}
