import React, { useState } from "react";
import { PopupModal } from "../basic_components/PopupModal";
import { arucoMarkerFunctionProvider } from "operator/tsx/index";
import { Dropdown } from "../basic_components/Dropdown";
import "operator/css/ArucoMarkers.css"
import "operator/css/basic_components.css"
import { Alert } from "../basic_components/Alert";
import { className } from "shared/util";

enum ArucoNavigationResult {
    COMPLETE = "navigation complete",
    FAIL = 'navigation failed',
    MARKER_FAIL = "marker not found"
}

/** All the possible button functions */
export enum ArucoMarkersFunction {
    SaveMarker,
    NavigateToMarker,
    DeleteMarker,
    SavedMarkerNames,
    SaveRelativePose
}

export interface ArucoMarkersFunctions {
    SaveMarker: (markerID: string, name: string) => void,
    NavigateToMarker: (markerID: number) => Promise<string>,
    SavedMarkerNames: () => string[],
    DeleteMarker: (markerIndex: number) => void,
    SaveRelativePose: (markerIndex: number) => void
}

export const ArucoMarkers = () => {
    let functions: ArucoMarkersFunctions = {
        SaveMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SaveMarker) as (markerID: string, name: string) => void,
        NavigateToMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.NavigateToMarker) as (markerID: number) => Promise<string>,
        SavedMarkerNames: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SavedMarkerNames) as () => string[],
        DeleteMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.DeleteMarker) as (markerIndex: number) => void,
        SaveRelativePose: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SaveRelativePose) as (markerIndex: number) => void
    }

    const [markers, setMarkers] = useState<string[]>(functions.SavedMarkerNames());
    const [selectedIdx, setSelectedIdx] = React.useState<number>();
    const [showSaveMarkerModal, setShowSaveMarkerModal] = useState<boolean>(false);
    const [result, setResult] = React.useState<string>("")
    const [expand, setExpand] = React.useState<boolean>(false)

    const SaveMarkerModal = (props: {
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
                functions.SaveMarker(markerID, name);
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

    return (
        <React.Fragment>
            <div className="aruco-markers-container">Aruco Marker Navigator</div>
            <div className="aruco-markers-container">
                <Dropdown
                    onChange={setSelectedIdx}
                    selectedIndex={selectedIdx}
                    possibleOptions={markers}
                    placeholderText="Select a marker..."
                />
                <button className="play-btn" onClick={() => {
                    if (selectedIdx != undefined) {
                        const promise = functions.NavigateToMarker(selectedIdx)
                        promise.then((response) => { 
                            setResult(response) 
                            document.getElementById("operator-aruco-markers")!.style.height = "10rem"
                            setTimeout(() => {
                                document.getElementById("operator-aruco-markers")!.style.height = "6rem"
                                setResult("")
                            }, 5000)
                        })
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
                        functions.DeleteMarker(selectedIdx)
                        setMarkers(functions.SavedMarkerNames())
                        setSelectedIdx(undefined)
                    }
                }
                }>
                    Delete <span className="material-icons">delete_forever</span>
                </button>
                <button className="save-btn" onClick={() => {
                    if (selectedIdx != undefined) functions.SaveRelativePose(selectedIdx)
                }}>
                    Save Pose <span className="material-icons">save</span>
                </button>
                
            </div>
            <SaveMarkerModal
                setShow={setShowSaveMarkerModal}
                show={showSaveMarkerModal}
            />
            { result == ArucoNavigationResult.COMPLETE 
                ? <Alert type="success" message="Navigation Complete!"/>
                : <></>
            }
            { result == ArucoNavigationResult.FAIL 
                ? <Alert type="error" message="Navigation Failed!"/>
                : <></>
            }
            { result == ArucoNavigationResult.MARKER_FAIL 
                ? <Alert type="error" message="Could not find Aruco Marker. You may be too far away, try moving Stretch closer."/>
                : <></>
            }
        </React.Fragment>
    )
}