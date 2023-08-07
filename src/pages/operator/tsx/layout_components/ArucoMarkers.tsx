import React, { useState, useEffect } from "react";
import { PopupModal } from "../basic_components/PopupModal";
import { arucoMarkerFunctionProvider } from "operator/tsx/index";
import { Dropdown } from "../basic_components/Dropdown";
import "operator/css/ArucoMarkers.css"
import "operator/css/basic_components.css"
import { ArucoNavigationState, className } from "shared/util";
import { Tooltip } from "../static_components/Tooltip";

/** All the possible button functions */
export enum ArucoMarkersFunction {
    SaveMarker,
    NavigateToMarker,
    DeleteMarker,
    SavedMarkerNames,
    SaveRelativePose,
    Cancel
}

// export interface ArucoNavigationState {
//     state: ArucoNavigationResult | string,
//     alertType: string
// }

export interface ArucoMarkersFunctions {
    SaveMarker: (markerID: string, name: string, size: string) => ArucoNavigationState,
    NavigateToMarker: (markerID: number) => ArucoNavigationState,
    SavedMarkerNames: () => string[],
    DeleteMarker: (markerIndex: number) => ArucoNavigationState,
    SaveRelativePose: (markerIndex: number, saveToMap: boolean) => Promise<ArucoNavigationState>,
    Cancel: () => void
}

export const ArucoMarkers = (props: { 
    setArucoNavigationState: (state: ArucoNavigationState) => void,
    hideLabels: boolean
}) => {
    let functions: ArucoMarkersFunctions = {
        SaveMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SaveMarker) as (markerID: string, name: string, size: string) => ArucoNavigationState,
        NavigateToMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.NavigateToMarker) as (markerID: number) => ArucoNavigationState,
        SavedMarkerNames: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SavedMarkerNames) as () => string[],
        DeleteMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.DeleteMarker) as (markerIndex: number) => ArucoNavigationState,
        SaveRelativePose: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SaveRelativePose) as (markerIndex: number, saveToMap: boolean) => Promise<ArucoNavigationState>,
        Cancel: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.Cancel) as () => void
    }

    const [markers, setMarkers] = useState<string[]>(functions.SavedMarkerNames());
    const [selectedIdx, setSelectedIdx] = React.useState<number>();
    const [showSaveMarkerModal, setShowSaveMarkerModal] = useState<boolean>(false);
    const [showSavePoseModal, setShowSavePoseModal] = useState<boolean>(false);
    const [result, setResult] = React.useState<ArucoNavigationState>()

    let timeout: NodeJS.Timeout;

    const SaveMarkerModal = (props: {
        setArucoNavigationState: (state: ArucoNavigationState) => void
        setShow: (show: boolean) => void,
        show: boolean
    }) => {
        const [markerID, setMarkerID] = React.useState<string>("")
        const [markerSize, setMarkerSize] = React.useState<string>("")
        const [name, setName] = React.useState<string>("");

        function handleAccept() {
            if (name.length > 0) {
                if (!markers.includes(name)) {
                    setMarkers(markers => [...markers, name])
                }
                if (!markerID) throw 'Marker ID undefined'
                let state = functions.SaveMarker(markerID, name, markerSize)
                props.setArucoNavigationState(state);
            }
            setName("");
            setMarkerID("");
            setMarkerSize("")
        }

        return (
            <PopupModal
                setShow={props.setShow}
                show={props.show}
                onAccept={handleAccept}
                id="save-marker-modal"
                acceptButtonText="Save"
                acceptDisabled={name.length < 1}
            >
                <label htmlFor="new-marker-name"><b>Save Marker</b></label>
                <hr />
                <div className="marker-name">
                    <label>Marker Name</label>
                    <input autoFocus type="text" id="new-marker-name" name="new-option-name"
                        value={name} onChange={(e) => setName(e.target.value)}
                        placeholder="Enter Marker Name"
                    />
                </div>
                <div className="marker-id">
                    <label>Marker ID</label>
                    <input autoFocus type="number" id="new-marker-id" name="new-option-id"
                        value={markerID} onChange={(e) => setMarkerID(e.target.value)}
                        placeholder="Enter Marker ID"
                    />
                </div>
                <div className="marker-size">
                    <label>Marker Size</label>
                    <input autoFocus type="number" id="new-marker-size" name="new-option-size"
                        value={markerSize} onChange={(e) => setMarkerSize(e.target.value)}
                        placeholder="Enter Marker Size (mm)"
                    />
                </div>
            </PopupModal>
        )
    }

    const SavePoseModal = (props: {
        setArucoNavigationState: (state: ArucoNavigationState) => void,
        setShow: (show: boolean) => void,
        show: boolean
    }) => {
        const [selectedMarkerID, setSelectedMarkerID] = React.useState<number>()
        const [saveToMap, setSaveToMap] = React.useState<boolean>(false)

        function handleAccept() {
            if (!selectedMarkerID) throw 'Marker ID undefined'
            const promise = functions.SaveRelativePose(selectedMarkerID, saveToMap)
            promise.then((response) => {
                props.setArucoNavigationState(response)
            })
            setSelectedMarkerID(undefined);
            setSaveToMap(false);
        }

        return (
            <PopupModal
                setShow={props.setShow}
                show={props.show}
                onAccept={handleAccept}
                id="save-pose-modal"
                acceptButtonText="Save"
                acceptDisabled={!selectedMarkerID}
            >
                <label htmlFor="save-pose"><b>Save Relative Pose</b></label>
                <hr />
                <Dropdown
                    onChange={setSelectedMarkerID}
                    selectedIndex={selectedMarkerID}
                    possibleOptions={markers}
                    placeholderText="Select a marker..."
                    placement="bottom"
                />
                <ul className="checkbox">
                    <li>
                        <input type="checkbox" id="save-to-map" name="save-to-map" 
                            value="Map" onChange={(e) => setSaveToMap(e.target.checked)}
                        />
                        <label>Save Marker to Map</label>
                    </li>
                </ul>
            </PopupModal>
        )
    }

    return (
        <React.Fragment>
            <div className="aruco-markers-container">Aruco Marker Navigator</div>
            <div className="aruco-markers-container">
                <Dropdown
                    onChange={setSelectedIdx}
                    selectedIndex={selectedIdx}
                    possibleOptions={markers}
                    placeholderText="Select a marker..."
                    placement="bottom"
                />
                <Tooltip text="Play" position="top">
                    <button className="play-btn" onClick={() => {
                        if (selectedIdx != undefined) {
                            let state = functions.NavigateToMarker(selectedIdx)
                            if (state) props.setArucoNavigationState(state)
                        }
                    }
                    }>
                        <span hidden={props.hideLabels}>Play</span> 
                        <span className="material-icons">play_circle</span>
                    </button>
                </Tooltip>
                <Tooltip text="Cancel" position="top">
                    <button className="delete-btn" onClick={() => functions.Cancel()}>
                        <span hidden={props.hideLabels}>Cancel</span>
                        <span className="material-icons">cancel</span>
                    </button>
                </Tooltip>
                <Tooltip text="Save Marker" position="top">
                    <button className="save-btn" onClick={() => setShowSaveMarkerModal(true)}>
                        <span hidden={props.hideLabels}>Save Marker</span>
                        <span className="material-icons">save</span><span className="material-icons">qr_code_2</span>
                    </button>
                </Tooltip>
                <Tooltip text="Delete" position="top">
                    <button className="delete-btn" onClick={() => {
                        if (selectedIdx != undefined) {
                            props.setArucoNavigationState(functions.DeleteMarker(selectedIdx))
                            setMarkers(functions.SavedMarkerNames())
                            setSelectedIdx(undefined)
                        }
                    }
                    }>
                        <span hidden={props.hideLabels}>Delete Marker</span>
                        <span className="material-icons">delete_forever</span>
                    </button>
                </Tooltip>
                <Tooltip text="Save Pose" position="top">
                    <button className="save-btn" onClick={() => setShowSavePoseModal(true)}>
                        <span hidden={props.hideLabels}>Save Pose</span>
                        <span className="material-icons">save</span><span className="material-icons">location_on</span>
                    </button>
                </Tooltip>
            </div>
            <SaveMarkerModal
                setArucoNavigationState={props.setArucoNavigationState}
                setShow={setShowSaveMarkerModal}
                show={showSaveMarkerModal}
            />
            <SavePoseModal
                setArucoNavigationState={props.setArucoNavigationState}
                setShow={setShowSavePoseModal}
                show={showSavePoseModal}
            />
        </React.Fragment>
    )
}