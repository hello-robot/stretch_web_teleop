import React, { useState } from "react";
import { PopupModal } from "../basic_components/PopupModal";
import { arucoMarkerFunctionProvider } from "operator/tsx/index";
import { Dropdown } from "../basic_components/Dropdown";
import "operator/css/ArucoMarkers.css"
import "operator/css/basic_components.css"

/** All the possible button functions */
export enum ArucoMarkersFunction {
    SaveMarker,
    NavigateToMarker,
    DeleteMarker,
    SavedMarkerNames,
}

export interface ArucoMarkersFunctions {
    SaveMarker: (markerID: string, name: string) => void,
    NavigateToMarker: (markerID: number) => void,
    SavedMarkerNames: () => string[],
    DeleteMarker: (markerIndex: number) => void
}

export const ArucoMarkers = () => {
    let functions: ArucoMarkersFunctions = {
        SaveMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SaveMarker) as (markerID: string, name: string) => void,
        NavigateToMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.NavigateToMarker) as (markerID: number) => void,
        SavedMarkerNames: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.SavedMarkerNames) as () => string[],
        DeleteMarker: arucoMarkerFunctionProvider.provideFunctions(ArucoMarkersFunction.DeleteMarker) as (markerIndex: number) => void,
    }

    const [markers, setMarkers] = useState<string[]>(functions.SavedMarkerNames());
    const [selectedIdx, setSelectedIdx] = React.useState<number>();
    const [showSaveMarkerModal, setShowSaveMarkerModal] = useState<boolean>(false);

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
            <div id="aruco-markers-container">Aruco Marker Navigator</div>
            <div id="aruco-markers-container">
                <Dropdown
                    onChange={setSelectedIdx}
                    selectedIndex={selectedIdx}
                    possibleOptions={markers}
                    placeholderText="Select a marker..."
                />
                <button className="play-btn" onClick={() => {
                    if (selectedIdx != undefined) {
                        functions.NavigateToMarker(selectedIdx)
                    }
                }
                }>
                    Play <span className="material-icons">play_circle</span>
                </button>
                <button className="save-btn" onClick={() => setShowSaveMarkerModal(true)}>
                    Save <span className="material-icons">save</span>
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
            </div>
            <SaveMarkerModal
                setShow={setShowSaveMarkerModal}
                show={showSaveMarkerModal}
            />
        </React.Fragment>
    )
}