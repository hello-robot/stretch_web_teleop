import React, { useMemo, useEffect, useCallback, useState, useRef } from "react";
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
    RenameRecording,
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
    RenameRecording: (recordingID: number, recordingNameNew: string) => void;
}



/****************************
 * Record/Start/Stop <button> *
 ****************************/

const ButtonRecord = (props: {
    showRecordingStartButton: boolean;
    showRecordingStartButtonSet: React.Dispatch<React.SetStateAction<boolean>>;
    isRecording: boolean;
    isRecordingSet: React.Dispatch<React.SetStateAction<boolean>>;
    isOneJointSelected: boolean;
    startRecording: () => void;
    stopRecording: () => void;
    saveRecording: (name: string) => void;
    setRecordings: React.Dispatch<React.SetStateAction<string[]>>;
    deselectAllJoints: () => void;
    isNamingModalVisibleSet: (arg0: boolean) => void;
    recordingNameSet: React.Dispatch<React.SetStateAction<string>>;
}) => {
    /////////////////////////
    //// "Record" button ////
    /////////////////////////
    if (
        !props.showRecordingStartButton
        && !props.isRecording
    ) {
        return (
            <button
                onPointerDown={() => props.showRecordingStartButtonSet(true)}>
                Record 🔴
            </button>
        );
    }
    //////////////////////// 
    //// "Start" button ////
    ////////////////////////
    else if (
        props.showRecordingStartButton
        && !props.isRecording
    ) {
        return (
            <button
                onPointerDown={props.startRecording}
                disabled={!props.isOneJointSelected}
            >
                Start 🔴
            </button>
        );
    }
    /////////////////////// 
    //// "Stop" button ////
    ///////////////////////
    else if (props.isRecording) {
        return (
            <button
                onPointerDown={() => {
                    const tempName = new Date().toLocaleString('en-US', {
                        month: 'numeric',
                        day: 'numeric',
                        hour: 'numeric',
                        minute: '2-digit',
                        hour12: true
                    });
                    props.recordingNameSet(tempName);
                    props.isNamingModalVisibleSet(true)
                }}
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
    const refInput = useRef<HTMLInputElement>(null);

    // Effect to focus the input when filter is activated
    useEffect(() => {
        if (props.isFilterActivated && refInput.current) {
            requestAnimationFrame(() => {
                refInput.current?.focus();
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
                ref={refInput}
                type="text"
                placeholder="Type to filter..."
                value={props.filterQuery}
                onFocus={(e) => e.target.select()}
                onChange={(e) => props.filterQuerySet(e.target.value)}
            />
        </div>
    );
}

interface RecordingItemProps {
    recordingName: string;
    idxFixed: number;
    functions: {
        LoadRecording: (idx: number) => void;
        RenameRecording: (idx: number, newName: string) => void;
        DeleteRecording: (idx: number) => void;
        SavedRecordingNames: () => string[];
    };
    setRecordings: React.Dispatch<React.SetStateAction<string[]>>;
    scrollToTop?: () => void;
}

const RecordingItem: React.FC<RecordingItemProps> = ({
    recordingName,
    idxFixed,
    functions,
    setRecordings,
    scrollToTop,
}: RecordingItemProps) => {
    const [valueTextArea, valueTextAreaSet] = useState<string>(recordingName);
    const refTextArea = useRef<HTMLTextAreaElement>(null);
    const recordingsRefresh = useCallback(() => {
        setRecordings(functions.SavedRecordingNames());
    }, []);
    const [isEditing, isEditingSet] = useState<boolean>(false);
    useEffect(() => {
        if (isEditing) {
            requestAnimationFrame(() => {
                refTextArea.current?.focus();
                refTextArea.current?.select();
            });
        }
    }, [isEditing]);
    // Adjust height of the textarea based on its content
    useEffect(() => {
        const adjustHeight = () => {
            const domNode = refTextArea.current;
            if (domNode) {
                domNode.style.height = '30px';
                domNode.style.height = domNode.scrollHeight + 'px';
            }
        };
        
        adjustHeight();
        
        // Also adjust when window resizes
        window.addEventListener('resize', adjustHeight);
        
        return () => {
            window.removeEventListener('resize', adjustHeight);
        };
    }, [valueTextArea]);

    return (
        <div
            className="recording-item"
            key={recordingName}
        >
            <textarea
                // type="text"
                ref={refTextArea}
                className="recording-name-text-area"
                value={valueTextArea || recordingName}
                placeholder={recordingName}
                onChange={(e) => {
                    valueTextAreaSet(e.target.value)
                }}
                disabled={!isEditing}
                onBlur={(e) => {

                    const recordingNameNew = e.target.value.trim() || recordingName;

                    functions.RenameRecording(idxFixed, recordingNameNew)
                    recordingsRefresh();
                    isEditingSet(false);
                    scrollToTop();
                }}
            />
            <Flex gap={5}>
                <button onPointerDown={() => {
                    functions.LoadRecording(idxFixed)
                }}>
                    ▸
                </button>
                <button
                    onPointerDown={() => {
                        isEditingSet(true);
                    }}
                >✐</button>
                <button
                    onPointerDown={() => {
                        functions.DeleteRecording(idxFixed);
                        recordingsRefresh();
                    }}
                >␡</button>
            </Flex>
        </div>
    );
};

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
        RenameRecording: movementRecorderFunctionProvider.provideFunctions(
            MovementRecorderFunction.RenameRecording,
        ) as (recordingID: number, recordingNameNew: string) => void,
    };

    let radioFuncts: RadioFunctions = {
        Delete: movementRecorderFunctionProvider.provideFunctions(
            MovementRecorderFunction.DeleteRecordingName,
        ) as (name: string) => void,
        GetLabels: functions.SavedRecordingNames,
        SelectedLabel: (label: string) =>
            setSelectedIdx(functions.SavedRecordingNames().indexOf(label)),
    };



    // TODO: previous abstraction
    const [selectedIdx, setSelectedIdx] = React.useState<number>();
    const [showJointSelectionModal, setShowJointSelectionModal] =
        useState<boolean>(false);
    const [showSaveRecordingModal, setShowSaveRecordingModal] =
        useState<boolean>(false);
    // TODO: previous abstraction






    /*******************
     * Joint selection *
     *******************/

    const [head, setHead] = React.useState<boolean>(false);
    const [arm, setArm] = React.useState<boolean>(false);
    const [lift, setLift] = React.useState<boolean>(false);
    const [wristRoll, setWristRoll] = React.useState<boolean>(false);
    const [wristPitch, setWristPitch] = React.useState<boolean>(false);
    const [wristYaw, setWristYaw] = React.useState<boolean>(false);
    const [gripper, setGripper] = React.useState<boolean>(false);
    const [isOneJointSelected, isOneJointSelectedSet] = React.useState<boolean>(false);
    const selectAllJoints = useCallback(() => {
        setHead(true);
        setArm(true);
        setLift(true);
        setWristRoll(true);
        setWristPitch(true);
        setWristYaw(true);
        setGripper(true);
    }, []);

    const deselectAllJoints = useCallback(() => {
        setHead(false);
        setArm(false);
        setLift(false);
        setWristRoll(false);
        setWristPitch(false);
        setWristYaw(false);
        setGripper(false);
    }, []);
    // Effect to check if at least one joint is selected
    useEffect(() => {
        isOneJointSelectedSet(
            head
            || arm
            || lift
            || wristRoll
            || wristPitch
            || wristYaw
            || gripper
        );
    }, [head, arm, lift, wristRoll, wristPitch, wristYaw, gripper]);



    /*************
     * Recording *
     *************/

    const [recordings, setRecordings] = useState<string[]>(
        functions.SavedRecordingNames(),
    );
    const [isRecording, isRecordingSet] = React.useState<boolean>(false);
    const [showRecordingStartButton, showRecordingStartButtonSet] =
        useState<boolean>(false);
    const [isNamingModalVisible, isNamingModalVisibleSet] = React.useState<boolean>(false);
    const [recordingName, recordingNameSet] = React.useState<string>('');

    const startRecording = useCallback(() => {
        showRecordingStartButtonSet(false)
        isRecordingSet(true);
        functions.Record(
            head,
            arm,
            lift,
            wristRoll,
            wristPitch,
            wristYaw,
            gripper,
        );
    }, [head, arm, lift, wristRoll, wristPitch, wristYaw, gripper,]);


    /*******************************
     * Filter state & side-effects *
     *******************************/

    const [isFilterActivated, isFilterActivatedSet] =
        useState<boolean>(false);
    const [filterQuery, filterQuerySet] = useState<string>('');
    const recordingsFiltered = useMemo(() => {
        if (!filterQuery) return recordings;
        const filterQueryLower = filterQuery.toLowerCase().trim();
        return recordings.filter(recording =>
            recording.toLowerCase().includes(filterQueryLower)
        );
    }, [filterQuery, recordings]);



    /********************************************
     * Callback to go back to the drawing board *
     ********************************************/

    const dumpToInitialState = useCallback(() => {
        filterQuerySet('');
        isFilterActivatedSet(false);
        showRecordingStartButtonSet(false);
        deselectAllJoints();
        isRecordingSet(false);
        functions.StopRecording();
        isNamingModalVisibleSet(false);
    }, []);



    /****************************
     * Step-scrolling <buttons> *
     ****************************/

    // Refs
    const refRecordingsList = useRef(null);
    const refJointsList = useRef(null);

    // Track whether we can scroll up/down
    const [canScrollUp, setCanScrollUp] = useState(false);
    const [canScrollDown, setCanScrollDown] = useState(false);

    // Current DOM node to scroll
    const [domNodeScroll, domNodeScrollSet] = useState<HTMLElement | null>(null);
    useEffect(() => {
        const domNodeScroll = !showRecordingStartButton && !isRecording
            ? refRecordingsList.current
            : refJointsList.current;
        domNodeScrollSet(domNodeScroll);
    }, [showRecordingStartButton, isRecording]);

    // Event listeners when scroll and resize
    useEffect(() => {
        if (!domNodeScroll) return;

        const handleScroll = () => {
            setCanScrollUp(domNodeScroll.scrollTop > 0);
            setCanScrollDown(
                domNodeScroll.scrollTop + domNodeScroll.clientHeight <
                domNodeScroll.scrollHeight - 1
            );
        };

        // run once on mount
        handleScroll();

        // listen to both scroll and resize
        domNodeScroll.addEventListener("scroll", handleScroll);
        window.addEventListener("resize", handleScroll);

        return () => {
            domNodeScroll.removeEventListener("scroll", handleScroll);
            window.removeEventListener("resize", handleScroll);
        };
    }, [domNodeScroll, filterQuery, isRecording, showRecordingStartButton, isNamingModalVisible]);
    const scrollUp = useCallback(() => {
        if (domNodeScroll) {
            const scrollDistance = domNodeScroll.clientHeight * 0.9;
            domNodeScroll.scrollBy({
                top: -scrollDistance,
                behavior: "smooth",
            });
        }
    }, [domNodeScroll]);
    const scrollDown = useCallback(() => {
        if (domNodeScroll) {
            const scrollDistance = domNodeScroll.clientHeight * 0.9;
            domNodeScroll.scrollBy({
                top: scrollDistance,
                behavior: "smooth",
            });
        }
    }, [domNodeScroll]);
    const scrollToTop = useCallback(() => {
        domNodeScroll.scrollTo({
            top: 0,
            behavior: "smooth",
        });
    }, [domNodeScroll]);



    /****************
     * Naming Modal *
     ****************/

    // ref for <input>
    const refInputRecordingName = useRef<HTMLInputElement>(null);

    // Auto-select text inside of <input>
    // when naming modal is visible
    useEffect(() => {
        if (isNamingModalVisible) {
            requestAnimationFrame(() => { refInputRecordingName.current.select() });
        }
    }, [isNamingModalVisible]);

    // TODO: previous abstraction
    const JointSelectionModal = (props: {
        setShow: (show: boolean) => void;
        show: boolean;
        isRecordingSet: (isRecording: boolean) => void;
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
                    props.isRecordingSet(false);
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
    useEffect(() => {
        if (!isBrowser && !isTablet) {
            if (props.isRecording == undefined) {
                return;
            } else if (props.isRecording) {
                isRecordingSet(true);
                setShowJointSelectionModal(true);
            } else {
                isRecordingSet(false);
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
    // TODO: previous abstraction



    return isBrowser || isTablet ? (
        <React.Fragment>
            <div id="movement-recorder-container">
                <div ref={refRecordingsList} className="recordings-list">
                    {
                        // sort by newest recordings...
                        [...recordingsFiltered].reverse().map((recordingName, idx) => {
                            const idxFixed = recordings.indexOf(recordingName);
                            return (
                                <RecordingItem
                                    key={recordingName + idx}
                                    recordingName={recordingName}
                                    idxFixed={idxFixed}
                                    functions={functions}
                                    setRecordings={setRecordings}
                                    scrollToTop={scrollToTop}
                                />
                            );
                        })
                    }

                </div>
                {showRecordingStartButton || isRecording
                    ? (
                        <div className="joints-list" ref={refJointsList}>
                            <Flex>
                                <button
                                    onPointerDown={!isOneJointSelected ? selectAllJoints : deselectAllJoints}
                                    disabled={isRecording}
                                >
                                    {!isOneJointSelected ? "Select All" : "Deselect All"}
                                </button>
                            </Flex>
                            <ul className="checkbox">
                                <li>
                                    <input
                                        type="checkbox"
                                        id="head"
                                        name="save-head-pose"
                                        value="Head"
                                        checked={head}
                                        onChange={(e) => setHead(e.target.checked)}
                                        disabled={isRecording}
                                    />
                                    <label>Head</label>
                                </li>
                                <li>
                                    <input
                                        type="checkbox"
                                        id="arm"
                                        name="save-arm-pose"
                                        value="Arm"
                                        checked={arm}
                                        onChange={(e) => setArm(e.target.checked)}
                                        disabled={isRecording}
                                    />
                                    <label>Arm</label>
                                </li>
                                <li>
                                    <input
                                        type="checkbox"
                                        id="lift"
                                        name="save-lift-pose"
                                        value="Lift"
                                        checked={lift}
                                        onChange={(e) => setLift(e.target.checked)}
                                        disabled={isRecording}
                                    />
                                    <label>Lift</label>
                                </li>
                                <li>
                                    <input
                                        type="checkbox"
                                        id="wristRoll"
                                        name="save-wrist-roll-pose"
                                        value="Wrist Roll"
                                        checked={wristRoll}
                                        onChange={(e) => setWristRoll(e.target.checked)}
                                        disabled={isRecording}
                                    />
                                    <label>Wrist Twist</label>
                                </li>
                                <li>
                                    <input
                                        type="checkbox"
                                        id="wristPitch"
                                        name="save-wrist-pitch-pose"
                                        value="Wrist Pitch"
                                        checked={wristPitch}
                                        onChange={(e) => setWristPitch(e.target.checked)}
                                        disabled={isRecording}
                                    />
                                    <label>Wrist Bend</label>
                                </li>
                                <li>
                                    <input
                                        type="checkbox"
                                        id="wristYaw"
                                        name="save-wrist-yaw-pose"
                                        value="Wrist Yaw"
                                        checked={wristYaw}
                                        onChange={(e) => setWristYaw(e.target.checked)}
                                        disabled={isRecording}
                                    />
                                    <label>Wrist Rotate</label>
                                </li>
                                <li>
                                    <input
                                        type="checkbox"
                                        id="gripper"
                                        name="save-gripper-pose"
                                        value="Gripper"
                                        checked={gripper}
                                        onChange={(e) => setGripper(e.target.checked)}
                                        disabled={isRecording}
                                    />
                                    <label>Gripper</label>
                                </li>
                            </ul>
                        </div>
                    )
                    : null}

                {isNamingModalVisible
                    ? (
                        <Flex className="naming-modal" direction="column" gap={10}>
                            <div>Recording name:</div>
                            <Flex>
                                <input
                                    type="text"
                                    ref={refInputRecordingName}
                                    value={recordingName}
                                    onFocus={(e) => e.target.select()}
                                    onChange={(e) => recordingNameSet(e.target.value)}
                                />
                            </Flex>
                            <Flex gap={5}>
                                <button
                                    onPointerDown={() => {
                                        setRecordings((recordings) => [...recordings, recordingName]);
                                        functions.SaveRecording(recordingName);
                                        showRecordingStartButtonSet(false)
                                        isRecordingSet(false);
                                        deselectAllJoints();
                                        isNamingModalVisibleSet(false);
                                    }}
                                    disabled={recordingName.length < 1 || recordings.includes(recordingName)}
                                >
                                    ✐ Save
                                </button>
                                <button
                                    onPointerDown={dumpToInitialState}
                                >
                                    Cancel
                                </button>
                            </Flex>
                        </Flex>
                    )
                    : null}
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
                {!isNamingModalVisible
                    ? (<div className="footer">
                        <Flex gap={5} align="center">
                            {isFilterActivated || showRecordingStartButton || (isRecording && showRecordingStartButton)
                                ? <button onPointerDown={dumpToInitialState}>◂</button>
                                : null
                            }
                            {!isFilterActivated
                                ? <ButtonRecord
                                    showRecordingStartButton={showRecordingStartButton}
                                    showRecordingStartButtonSet={showRecordingStartButtonSet}
                                    isRecording={isRecording}
                                    isRecordingSet={isRecordingSet}
                                    isOneJointSelected={isOneJointSelected}
                                    startRecording={startRecording}
                                    stopRecording={functions.StopRecording}
                                    saveRecording={functions.SaveRecording}
                                    setRecordings={setRecordings}
                                    deselectAllJoints={deselectAllJoints}
                                    isNamingModalVisibleSet={isNamingModalVisibleSet}
                                    recordingNameSet={recordingNameSet}
                                />
                                : null
                            }
                            {!showRecordingStartButton && !isRecording
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
                    )
                    : null}
            </div>
            <JointSelectionModal
                setShow={setShowJointSelectionModal}
                show={showJointSelectionModal}
                isRecordingSet={isRecordingSet}
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
                            isRecordingSet(true)
                            functions.Record()
                            if (props.onRecordingChange) props.onRecordingChange(true)
                        } else {
                            isRecordingSet(false)
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
                isRecordingSet={isRecordingSet}
            />
            <SaveRecordingModal
                setShow={setShowSaveRecordingModal}
                show={showSaveRecordingModal}
            />
        </React.Fragment>
    );
};
