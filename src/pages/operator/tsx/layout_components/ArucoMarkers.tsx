import React, { useState, useEffect } from "react";
import { PopupModal } from "../basic_components/PopupModal";
import { arucoMarkerFunctionProvider } from "operator/tsx/index";
import { Dropdown } from "../basic_components/Dropdown";
import "operator/css/ArucoMarkers.css"
import "operator/css/basic_components.css"
import { Alert } from "../basic_components/Alert";
import { ArucoNavigationResult } from "../function_providers/ArucoMarkerFunctionProvider";
import { ArucoNavigationState, className } from "shared/util";

/** All the possible button functions */
export enum ArucoMarkersFunction {
    SaveMarker,
    NavigateToMarker,
    DeleteMarker,
    SavedMarkerNames,
    SaveRelativePose,
}

// export interface ArucoNavigationState {
//     state: ArucoNavigationResult | string,
//     alertType: string
// }

export interface ArucoMarkersFunctions {
    SaveMarker: (markerID: string, name: string) => ArucoNavigationState,
    NavigateToMarker: (markerID: number) => ArucoNavigationState,
    SavedMarkerNames: () => string[],
    DeleteMarker: (markerIndex: number) => ArucoNavigationState,
    SaveRelativePose: (markerIndex: number, saveToMap: boolean) => Promise<ArucoNavigationState>,
}

export const ArucoMarkers = (props: { setArucoNavigationState: (state: ArucoNavigationState) => void }) => {
    let functions: ArucoMarkersFunctions = {
        SaveMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SaveMarker) as (markerID: string, name: string) => ArucoNavigationState,
        NavigateToMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.NavigateToMarker) as (markerID: number) => ArucoNavigationState,
        SavedMarkerNames: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SavedMarkerNames) as () => string[],
        DeleteMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.DeleteMarker) as (markerIndex: number) => ArucoNavigationState,
        SaveRelativePose: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SaveRelativePose) as (markerIndex: number, saveToMap: boolean) => Promise<ArucoNavigationState>,
    }

    const [markers, setMarkers] = useState<string[]>(functions.SavedMarkerNames());
    const [selectedIdx, setSelectedIdx] = React.useState<number>();
    const [showSaveMarkerModal, setShowSaveMarkerModal] = useState<boolean>(false);
    const [showSavePoseModal, setShowSavePoseModal] = useState<boolean>(false);
    const [result, setResult] = React.useState<ArucoNavigationState>()

    let timeout: NodeJS.Timeout;

    // useEffect(() => {
    //     let state = props.arucoNavigationState
    //     if (state && state.state !== "") setResult({ result: state.state, alert: state.alertType })
    // }, [props.arucoNavigationState])

    // useEffect(() => {
    //     if (result) {
    //         console.log(result)
    //         document.getElementById("operator-aruco-markers")!.style.height = "10rem"
    //         if (timeout) clearTimeout(timeout)
    //         if (result.alert != "info") {
    //             timeout = setTimeout(() => {
    //                 document.getElementById("operator-aruco-markers")!.style.height = "6rem"
    //                 setResult(undefined)
    //             }, 5000)
    //         }
    //     }
    // }, [result]);

    const SaveMarkerModal = (props: {
        setArucoNavigationState: (state: ArucoNavigationState) => void
        setShow: (show: boolean) => void,
        show: boolean
    }) => {
        const [markerID, setMarkerID] = React.useState<string>("")
        const [name, setName] = React.useState<string>("");

        function handleAccept() {
            if (name.length > 0) {
                if (!markers.includes(name)) {
                    setMarkers(markers => [...markers, name])
                }
                if (!markerID) throw 'Marker ID undefined'
                let state = functions.SaveMarker(markerID, name)
                props.setArucoNavigationState(state);
            }
            setName("");
            setMarkerID("");
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
                <button className="play-btn" onClick={() => {
                    if (selectedIdx != undefined) {
                        let state = functions.NavigateToMarker(selectedIdx)
                        if (state) props.setArucoNavigationState(state)
                        // const promise = functions.NavigateToMarker(selectedIdx)
                        // promise.then((response) => {
                        //     setResult(response)

                        //     // setTimeout(() => {
                        //     //     document.getElementById("operator-aruco-markers")!.style.height = "6rem"
                        //     //     setResult("")
                        //     // }, 5000)
                        // })
                    }
                }
                }>
                    Play <span className="material-icons">play_circle</span>
                </button>
                <button className="save-btn" onClick={() => setShowSaveMarkerModal(true)}>
                    Save Marker<span className="material-icons">save</span>
                </button>
                <button className="delete-btn" onClick={() => {
                    if (selectedIdx != undefined) {
                        props.setArucoNavigationState(functions.DeleteMarker(selectedIdx))
                        setMarkers(functions.SavedMarkerNames())
                        setSelectedIdx(undefined)
                    }
                }
                }>
                    Delete <span className="material-icons">delete_forever</span>
                </button>
                {/* <button className="save-btn" onClick={() => {
                    if (selectedIdx != undefined) {
                        const promise = functions.SaveRelativePose(selectedIdx)
                        promise.then((response) => {
                            setResult(response)
                        })
                    }
                }}>
                    Save Pose <span className="material-icons">save</span>
                </button> */}
                <button className="save-btn" onClick={() => setShowSavePoseModal(true)}>
                    Save Pose<span className="material-icons">save</span>
                </button>
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
            {/* {result &&
                <div className="operator-collision-alerts">
                    <div className={className('operator-alert', { fadeIn: result !== undefined, fadeOut: result == undefined })}>
                        <Alert type={result.alert} message={result.result} />
                    </div>
                </div>
            } */}
        </React.Fragment>
    )
}