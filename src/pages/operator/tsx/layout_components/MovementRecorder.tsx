import React, { useEffect, useState } from "react";
import { PopupModal } from "../basic_components/PopupModal";
import { movementRecorderFunctionProvider } from "operator/tsx/index";
import { Dropdown } from "../basic_components/Dropdown";
import { Tooltip } from "../static_components/Tooltip";
import "operator/css/MovementRecorder.css";
import "operator/css/basic_components.css";
import { isMobile } from "react-device-detect";
import { RadioFunctions, RadioGroup } from "../basic_components/RadioGroup";

/** All the possible button functions */
export enum MovementRecorderFunction {
    Record,
    SaveRecording,
    StopRecording,
    SavedRecordingNames,
    DeleteRecording,
    LoadRecording,
    Cancel,
    DeleteRecordingName,
    LoadRecordingName,
}

export interface MovementRecorderFunctions {
    Record: () => void;
    SaveRecording: (name: string) => void;
    StopRecording: () => void;
    SavedRecordingNames: () => string[];
    DeleteRecording: (recordingID: number) => void;
    LoadRecording: (recordingID: number) => void;
}

export const MovementRecorder = (props: {
    hideLabels: boolean;
    globalRecord?: boolean;
    isRecording?: boolean;
}) => {
    let functions: MovementRecorderFunctions = {
        Record: movementRecorderFunctionProvider.provideFunctions(
            MovementRecorderFunction.Record,
        ) as () => void,
        SaveRecording: movementRecorderFunctionProvider.provideFunctions(
            MovementRecorderFunction.SaveRecording,
        ) as (name: string) => void,
        StopRecording: movementRecorderFunctionProvider.provideFunctions(
            MovementRecorderFunction.StopRecording,
        ) as () => void,
        SavedRecordingNames: movementRecorderFunctionProvider.provideFunctions(
            MovementRecorderFunction.SavedRecordingNames,
        ) as () => string[],
        DeleteRecording: movementRecorderFunctionProvider.provideFunctions(
            MovementRecorderFunction.DeleteRecording,
        ) as (recordingID: number) => void,
        LoadRecording: movementRecorderFunctionProvider.provideFunctions(
            MovementRecorderFunction.LoadRecording,
        ) as (recordingID: number) => void,
    };

    let radioFuncts: RadioFunctions = {
        Delete: movementRecorderFunctionProvider.provideFunctions(
            MovementRecorderFunction.DeleteRecordingName,
        ) as (name: string) => void,
        GetLabels: functions.SavedRecordingNames,
        SelectedLabel: (label: string) =>
            setSelectedIdx(functions.SavedRecordingNames().indexOf(label)),
    };

    const [recordings, setRecordings] = useState<string[]>(
        functions.SavedRecordingNames(),
    );
    const [selectedIdx, setSelectedIdx] = React.useState<number>();
    const [showSaveRecordingModal, setShowSaveRecordingModal] =
        useState<boolean>(false);
    const [isRecording, setIsRecording] = React.useState<boolean>(
        props.isRecording ? props.isRecording : false,
    );

    const SaveRecordingModal = (props: {
        setShow: (show: boolean) => void;
        show: boolean;
    }) => {
        const [name, setName] = React.useState<string>("");
        function handleAccept() {
            if (name.length > 0) {
                if (!recordings.includes(name)) {
                    setRecordings((recordings) => [...recordings, name]);
                }
                functions.SaveRecording(name);
            }
            setName("");
        }

        return (
            <PopupModal
                setShow={props.setShow}
                show={props.show}
                onAccept={handleAccept}
                onCancel={() => functions.StopRecording()}
                id="save-recording-modal"
                acceptButtonText="Save"
                acceptDisabled={name.length < 1}
                size={isMobile ? "small" : "large"}
                mobile={isMobile}
            >
                {/* <label htmlFor="new-recoding-name"><b>Save Recording</b></label>
                <hr/> */}
                <div className="recording-name">
                    {/* <label>Recording Name</label> */}
                    <input
                        autoFocus
                        type="text"
                        id="new-recording-name"
                        name="new-option-name"
                        value={name}
                        onChange={(e) => setName(e.target.value)}
                        placeholder="Enter name of movement"
                    />
                </div>
            </PopupModal>
        );
    };

    useEffect(() => {
        if (props.isRecording == undefined) {
            return;
        } else if (props.isRecording) {
            functions.Record();
        } else {
            setShowSaveRecordingModal(true);
        }
    }, [props.isRecording]);

    if (props.globalRecord !== undefined && !props.globalRecord)
        return (
            <SaveRecordingModal
                setShow={setShowSaveRecordingModal}
                show={showSaveRecordingModal}
            />
        );

    return !isMobile ? (
        <React.Fragment>
            <div id="movement-recorder-container">Movement Recorder</div>
            <div id="movement-recorder-container">
                <Dropdown
                    onChange={setSelectedIdx}
                    selectedIndex={selectedIdx}
                    possibleOptions={recordings}
                    placeholderText="Select a recording..."
                    placement="bottom"
                />
                <Tooltip text="Play movement" position="top">
                    <button
                        className="play-btn btn-label"
                        onClick={() => {
                            if (selectedIdx != undefined) {
                                functions.LoadRecording(selectedIdx);
                            }
                        }}
                    >
                        <span hidden={props.hideLabels}>Play</span>
                        <span className="material-icons">play_circle</span>
                    </button>
                </Tooltip>
                <Tooltip
                    text={!isRecording ? "Record movement" : "Save movement"}
                    position="top"
                >
                    <button
                        className="save-btn btn-label"
                        onClick={() => {
                            if (!isRecording) {
                                setIsRecording(true);
                                functions.Record();
                            } else {
                                setIsRecording(false);
                                setShowSaveRecordingModal(true);
                            }
                        }}
                    >
                        {!isRecording ? (
                            <i hidden={props.hideLabels}>Record</i>
                        ) : (
                            <i hidden={props.hideLabels}>Save</i>
                        )}
                        {!isRecording ? (
                            <span className="material-icons">
                                radio_button_checked
                            </span>
                        ) : (
                            <span className="material-icons">save</span>
                        )}
                    </button>
                </Tooltip>
                <Tooltip text="Delete recording" position="top">
                    <button
                        className="delete-btn btn-label"
                        onClick={() => {
                            if (selectedIdx != undefined) {
                                functions.DeleteRecording(selectedIdx);
                            }
                            setRecordings(functions.SavedRecordingNames());
                            setSelectedIdx(undefined);
                        }}
                    >
                        <span hidden={props.hideLabels}>Delete</span>
                        <span className="material-icons">delete_forever</span>
                    </button>
                </Tooltip>
            </div>
            <SaveRecordingModal
                setShow={setShowSaveRecordingModal}
                show={showSaveRecordingModal}
            />
        </React.Fragment>
    ) : (
        <React.Fragment>
            <RadioGroup functs={radioFuncts} />
            <div className="global-btns">
                {/* <div className="mobile-movement-save-btn" onClick={() => {
                        if (!isRecording) {
                            setIsRecording(true)
                            functions.Record()
                            if (props.onRecordingChange) props.onRecordingChange(true)
                        } else {
                            setIsRecording(false)
                            setShowSaveRecordingModal(true)
                            if (props.onRecordingChange) props.onRecordingChange(false)
                        }
                    }
                }>
                    {!isRecording
                        ? <span className="material-icons">radio_button_checked</span>
                        : <span className="material-icons">save</span>
                    }
                    {!isRecording ? <i>Record</i> : <i>Save</i> }
                </div> */}
                <div
                    className="mobile-movement-play-btn"
                    onClick={() => {
                        if (selectedIdx != undefined && selectedIdx > -1) {
                            functions.LoadRecording(selectedIdx);
                        }
                    }}
                >
                    <span className="material-icons">play_circle</span>
                    <i>Play</i>
                </div>
            </div>
            <SaveRecordingModal
                setShow={setShowSaveRecordingModal}
                show={showSaveRecordingModal}
            />
        </React.Fragment>
    );
};
