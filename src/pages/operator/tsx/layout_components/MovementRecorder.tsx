import React, { useMemo, useEffect, useState, useRef } from "react";
import { PopupModal } from "../basic_components/PopupModal";
import Flex from "../basic_components/Flex";
import { movementRecorderFunctionProvider } from "operator/tsx/index";
import { Dropdown } from "../basic_components/Dropdown";
import { Tooltip } from "../static_components/Tooltip";
import "operator/css/MovementRecorder.css";
import "operator/css/basic_components.css";
import { isBrowser, isTablet } from "react-device-detect";
import { RadioFunctions, RadioGroup } from "../basic_components/RadioGroup";
import PlayCircle from "@mui/icons-material/PlayCircle";
import RadioButtonCheckedIcon from "@mui/icons-material/RadioButtonChecked";
import DeleteForeverIcon from "@mui/icons-material/DeleteForever";
import SaveIcon from "@mui/icons-material/Save";

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
    Record: (
        head: boolean,
        arm: boolean,
        lift: boolean,
        wrist_roll: boolean,
        wrist_pitch: boolean,
        wrist_yaw: boolean,
        gripper: boolean,
    ) => void;
    SaveRecording: (name: string) => void;
    StopRecording: () => void;
    SavedRecordingNames: () => string[];
    DeleteRecording: (recordingID: number) => void;
    LoadRecording: (recordingID: number) => void;
}

const ButtonRecord = (props: {
    showRecordingStartButton: boolean;
    showRecordingStartButtonSet: React.Dispatch<React.SetStateAction<boolean>>;
    isRecording: boolean;
    setIsRecording: React.Dispatch<React.SetStateAction<boolean>>;
}) => {
    if (!props.showRecordingStartButton && !props.isRecording) {
        return (
            <button
                onPointerDown={() => props.showRecordingStartButtonSet(true)}>
                Record 🔴
            </button>
        );
    }
    else if (props.showRecordingStartButton && !props.isRecording) {
        return (
            <button
                onPointerDown={() => props.showRecordingStartButtonSet(false)}
            >
                Start 🔴
            </button>
        );
    }
    else if (props.isRecording) {
        return (
            <button
                onPointerDown={() => alert('hiiii')}
            >
                Stop 🔴
            </button>
        );
    }
}

const ButtonFilter = (props: {
    isFilterActivated: boolean;
    isFilterActivatedSet: React.Dispatch<React.SetStateAction<boolean>>;
    filterQuery: string;
    filterQuerySet: React.Dispatch<React.SetStateAction<string>>;
}) => {

    // Reference to focus the input when filter is activated
    const inputRef = useRef<HTMLInputElement>(null);

    // Effect to focus the input when filter is activated
    useEffect(() => {
        if (props.isFilterActivated && inputRef.current) {
            requestAnimationFrame(() => {
                inputRef.current?.focus();
            });
        }
    }, [props.isFilterActivated]);

    if (!props.isFilterActivated) {
        return (
            <button
                onPointerDown={() => {
                    props.isFilterActivatedSet(!props.isFilterActivated)
                }}>
                🔎
            </button>
        );
    } else return (
        <div className="button-filter">
            <input
                ref={inputRef}
                type="text"
                placeholder="Type to filter..."
                value={props.filterQuery}
                onChange={(e) => props.filterQuerySet(e.target.value)}
            />
        </div>
    );
}

export const MovementRecorder = (props: {
    hideLabels: boolean;
    globalRecord?: boolean;
    isRecording?: boolean;
}) => {
    let functions: MovementRecorderFunctions = {
        Record: movementRecorderFunctionProvider.provideFunctions(
            MovementRecorderFunction.Record,
        ) as (
            head: boolean,
            arm: boolean,
            lift: boolean,
            wrist_roll: boolean,
            wrist_pitch: boolean,
            wrist_yaw: boolean,
            gripper: boolean,
        ) => void,
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
    const [showJointSelectionModal, setShowJointSelectionModal] =
        useState<boolean>(false);
    const [showSaveRecordingModal, setShowSaveRecordingModal] =
        useState<boolean>(false);
    const [isRecording, setIsRecording] = React.useState<boolean>(
        props.isRecording ? props.isRecording : false,
    );
    const [isFilterActivated, isFilterActivatedSet] =
        useState<boolean>(false);
    const [showRecordingStartButton, showRecordingStartButtonSet] =
        useState<boolean>(false);
    const [filterQuery, filterQuerySet] = useState<string>('');
    const dumpToInitialState = () => {
        filterQuerySet('');
        isFilterActivatedSet(false);
        setIsRecording(false);
        setShowJointSelectionModal(false);
        setShowSaveRecordingModal(false);
        showRecordingStartButtonSet(false);
    }

    const recordingsListRef = useRef(null);

    // track whether we can scroll up / down
    const [canScrollUp, setCanScrollUp] = useState(false);
    const [canScrollDown, setCanScrollDown] = useState(false);

    useEffect(() => {
        const node = recordingsListRef.current;
        if (!node) return;
        const handleScroll = () => {
            setCanScrollUp(node.scrollTop > 0);
            setCanScrollDown(node.scrollTop + node.clientHeight < node.scrollHeight - 1);
        };
        handleScroll();
        node.addEventListener("scroll", handleScroll);
        return () => node.removeEventListener("scroll", handleScroll);
    }, [filterQuery]);

    const scrollUp = () => {
        if (recordingsListRef.current) {
            const scrollDistance = recordingsListRef.current.clientHeight * 0.9;
            recordingsListRef.current.scrollBy({
                top: -scrollDistance,
                behavior: "smooth",
            });
        }
    };

    const scrollDown = () => {
        if (recordingsListRef.current) {
            const scrollDistance = recordingsListRef.current.clientHeight * 0.9;
            recordingsListRef.current.scrollBy({
                top: scrollDistance,
                behavior: "smooth",
            });
        }
    };

    const JointSelectionModal = (props: {
        setShow: (show: boolean) => void;
        show: boolean;
        setIsRecording: (isRecording: boolean) => void;
    }) => {
        const [head, setHead] = React.useState<boolean>(false);
        const [arm, setArm] = React.useState<boolean>(false);
        const [lift, setLift] = React.useState<boolean>(false);
        const [wristRoll, setWristRoll] = React.useState<boolean>(false);
        const [wristPitch, setWristPitch] = React.useState<boolean>(false);
        const [wristYaw, setWristYaw] = React.useState<boolean>(false);
        const [gripper, setGripper] = React.useState<boolean>(false);

        function handleAccept() {
            functions.Record(
                head,
                arm,
                lift,
                wristRoll,
                wristPitch,
                wristYaw,
                gripper,
            );
            setHead(false);
            setArm(false);
            setLift(false);
            setWristRoll(false);
            setWristPitch(false);
            setWristYaw(false);
            setGripper(false);
        }

        return (
            <PopupModal
                setShow={props.setShow}
                show={props.show}
                onAccept={handleAccept}
                onCancel={() => {
                    props.setIsRecording(false);
                    props.setShow(false);
                    functions.StopRecording();
                }}
                id="save-recording-modal"
                acceptButtonText="Save"
                size={!isBrowser && !isTablet ? "small" : "large"}
                mobile={!isBrowser && !isTablet}
            >
                {/* <label htmlFor="new-recoding-name"><b>Save Recording</b></label>
                <hr/> */}
                <div className="joint-checkbox">
                    <label>Select the joints to record:</label>
                </div>
                <ul className="checkbox">
                    <li>
                        <input
                            type="checkbox"
                            id="head"
                            name="save-head-pose"
                            value="Head"
                            defaultChecked={head}
                            onChange={(e) => setHead(e.target.checked)}
                        />
                        <label>Head</label>
                    </li>
                    <li>
                        <input
                            type="checkbox"
                            id="arm"
                            name="save-arm-pose"
                            value="Arm"
                            defaultChecked={arm}
                            onChange={(e) => setArm(e.target.checked)}
                        />
                        <label>Arm</label>
                    </li>
                    <li>
                        <input
                            type="checkbox"
                            id="lift"
                            name="save-lift-pose"
                            value="Lift"
                            defaultChecked={lift}
                            onChange={(e) => setLift(e.target.checked)}
                        />
                        <label>Lift</label>
                    </li>
                    <li>
                        <input
                            type="checkbox"
                            id="wristRoll"
                            name="save-wrist-roll-pose"
                            value="Wrist Roll"
                            defaultChecked={wristRoll}
                            onChange={(e) => setWristRoll(e.target.checked)}
                        />
                        <label>Wrist Twist</label>
                    </li>
                    <li>
                        <input
                            type="checkbox"
                            id="wristPitch"
                            name="save-wrist-pitch-pose"
                            value="Wrist Pitch"
                            defaultChecked={wristPitch}
                            onChange={(e) => setWristPitch(e.target.checked)}
                        />
                        <label>Wrist Bend</label>
                    </li>
                    <li>
                        <input
                            type="checkbox"
                            id="wristYaw"
                            name="save-wrist-yaw-pose"
                            value="Wrist Yaw"
                            defaultChecked={wristYaw}
                            onChange={(e) => setWristYaw(e.target.checked)}
                        />
                        <label>Wrist Rotate</label>
                    </li>
                    <li>
                        <input
                            type="checkbox"
                            id="gripper"
                            name="save-gripper-pose"
                            value="Gripper"
                            defaultChecked={gripper}
                            onChange={(e) => setGripper(e.target.checked)}
                        />
                        <label>Gripper</label>
                    </li>
                </ul>
            </PopupModal>
        );
    };

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
                size={!isBrowser && !isTablet ? "small" : "large"}
                mobile={!isBrowser && !isTablet}
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


    // Strict filter results based on query
    const recordingsFiltered = useMemo(() => {
        if (!filterQuery) return recordings;
        const filterQueryLower = filterQuery.toLowerCase().trim();
        return recordings.filter(recording =>
            recording.toLowerCase().includes(filterQueryLower)
        );
    }, [filterQuery, recordings]);

    useEffect(() => {
        if (!isBrowser && !isTablet) {
            if (props.isRecording == undefined) {
                return;
            } else if (props.isRecording) {
                setIsRecording(true);
                setShowJointSelectionModal(true);
            } else {
                setIsRecording(false);
                setShowSaveRecordingModal(true);
            }
        }
    }, [props.isRecording]);

    if (props.globalRecord !== undefined && !props.globalRecord)
        return (
            <SaveRecordingModal
                setShow={setShowSaveRecordingModal}
                show={showSaveRecordingModal}
            />
        );

    return isBrowser || isTablet ? (
        <React.Fragment>
            <div id="movement-recorder-container">
                <div ref={recordingsListRef} className="recordings-list">
                    {
                        Array(5)
                            .fill(0)
                            .flatMap(() => recordingsFiltered)
                            .map((recording, idx) => {
                                return (
                                    <div
                                        key={recording + idx}
                                        className="recording-item"
                                    >
                                        <div>{recording}</div>
                                        <Flex gap={5}>
                                            <button onPointerDown={() => {
                                                // We need the actual index in "recordings" array, not the filtered one
                                                const idxTrue = recordings.indexOf(recording);
                                                functions.LoadRecording(idxTrue)
                                            }}>
                                                ▸
                                            </button>
                                            <button>✐</button>
                                            <button>␡</button>
                                        </Flex>
                                    </div>
                                )
                            })
                    }

                </div>
                <div className="articulation-points-list">

                </div>
                {/* <Tooltip
                    text={!isRecording ? "Record movement" : "Save movement"}
                    position="top"
                >
                    <button
                        className="save-btn btn-label"
                        onClick={() => {
                            if (!isRecording) {
                                setIsRecording(true);
                                setShowJointSelectionModal(true);
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
                            <RadioButtonCheckedIcon />
                        ) : (
                            <SaveIcon />
                        )}
                    </button>
                </Tooltip> */}
                {/* <Tooltip text="Delete recording" position="top">
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
                        <DeleteForeverIcon />
                    </button>
                </Tooltip> */}
                {/* Footer */}
                <div className="footer">
                    <Flex gap={5} align="center">
                        {isFilterActivated || showRecordingStartButton
                            ? <button onPointerDown={dumpToInitialState}>◂</button>
                            : null
                        }
                        {!isFilterActivated
                            ? <ButtonRecord
                                showRecordingStartButton={showRecordingStartButton}
                                showRecordingStartButtonSet={showRecordingStartButtonSet}
                                isRecording={isRecording}
                                setIsRecording={setIsRecording}
                            />
                            : null
                        }
                        {!showRecordingStartButton
                            ? <ButtonFilter
                                isFilterActivated={isFilterActivated}
                                isFilterActivatedSet={isFilterActivatedSet}
                                filterQuery={filterQuery}
                                filterQuerySet={filterQuerySet}
                            />
                            : null}
                    </Flex>
                    <Flex gap={5} align="center">
                        <button onPointerDown={scrollUp} disabled={!canScrollUp}>▲</button>
                        <button onPointerDown={scrollDown} disabled={!canScrollDown}>▼</button>
                    </Flex>
                </div>
            </div>
            <JointSelectionModal
                setShow={setShowJointSelectionModal}
                show={showJointSelectionModal}
                setIsRecording={setIsRecording}
            />
            <SaveRecordingModal
                setShow={setShowSaveRecordingModal}
                show={showSaveRecordingModal}
            />
        </React.Fragment>
    ) : (
        <React.Fragment>
            <RadioGroup
                functs={radioFuncts}
            />
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
                    <PlayCircle />
                    <i>Play</i>
                </div>
            </div>
            <JointSelectionModal
                setShow={setShowJointSelectionModal}
                show={showJointSelectionModal}
                setIsRecording={setIsRecording}
            />
            <SaveRecordingModal
                setShow={setShowSaveRecordingModal}
                show={showSaveRecordingModal}
            />
        </React.Fragment>
    );
};
