import React, { useMemo, useEffect, useCallback, useState, useRef } from "react";
import Flex from "../basic_components/Flex";
import { movementRecorderFunctionProvider } from "operator/tsx/index";
import "operator/css/MovementRecorder.css";
import "operator/css/basic_components.css";
import PlayCircle from "@mui/icons-material/PlayCircle";
import DeleteIcon from '@mui/icons-material/Delete';
import EditIcon from '@mui/icons-material/Edit';
import StopCircleIcon from '@mui/icons-material/StopCircle';
import RadioButtonCheckedIcon from '@mui/icons-material/RadioButtonChecked';
import NotStartedIcon from '@mui/icons-material/NotStarted';
import SearchIcon from '@mui/icons-material/Search';
import KeyboardArrowLeftIcon from '@mui/icons-material/KeyboardArrowLeft';
import KeyboardArrowUpIcon from '@mui/icons-material/KeyboardArrowUp';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';
import LocalFloristIcon from '@mui/icons-material/LocalFlorist';


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
                onPointerDown={() => props.showRecordingStartButtonSet(true)}
                className="button-record button-record-record"
            >
                Record <RadioButtonCheckedIcon />
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
                className="button-record button-record-start"
            >
                Start <NotStartedIcon />
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
                className="button-record button-record-stop"
            >
                Stop <StopCircleIcon />
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
                }}
                className="button-filter"
            >
                <SearchIcon />
            </button>
        );
    } else return (
        <div>
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
    const [isAskingConfirmationBeforeDelete, isAskingConfirmationBeforeDeleteSet] = useState<boolean>(false);
    // Focus <textarea>, and select value inside <textarea> when focused
    useEffect(() => {
        if (isEditing) {
            requestAnimationFrame(() => {
                refTextArea.current?.focus();
                refTextArea.current?.select();
            });
        }
    }, [isEditing]);
    // Adjust height of the textarea based on its content
    // This is to account for recording names that are really
    // long and need to be able to wrap
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
    const updateRecordingName = useCallback(() => {
        if (refTextArea?.current?.value.trim() === recordingName) {
            isEditingSet(false)
        } else {
            const recordingNameNew = refTextArea?.current?.value.trim() || recordingName;

            functions.RenameRecording(idxFixed, recordingNameNew)
            recordingsRefresh();
            isEditingSet(false);
            scrollToTop();
        }
    }, [valueTextArea, idxFixed, recordingName])
    return (
        <div
            className="recording-item"
            key={recordingName}
        >
            <textarea
                ref={refTextArea}
                className="recording-name-text-area"
                value={valueTextArea || recordingName}
                placeholder={recordingName}
                onChange={(e) => {
                    valueTextAreaSet(e.target.value)
                }}
                disabled={!isEditing}
                onBlur={updateRecordingName}
            />
            <Flex gap={4} className="recording-item-buttons">
                <button
                    onPointerDown={() => {
                        functions.LoadRecording(idxFixed)
                    }}
                    className={
                        `button-playback
                        ${!isAskingConfirmationBeforeDelete ? "visible" : "hidden"}`
                    }
                >
                    <PlayCircle />
                </button>
                <button
                    onPointerDown={() => {
                        isEditingSet(true);
                    }}
                    className={`
                        button-edit
                        ${!isAskingConfirmationBeforeDelete ? "visible" : "hidden"}
                        ${isEditing ? 'editing' : ''}
                    `}
                ><EditIcon className="button-edit-icon" /></button>
                <Flex className="button-delete-recording-wrapper">
                    <button
                        onPointerDown={() => { isAskingConfirmationBeforeDeleteSet(false) }}
                        disabled={!isAskingConfirmationBeforeDelete}
                        className={`button-cancel-deletion ${isAskingConfirmationBeforeDelete ? "visible" : "hidden"}`}
                    >
                        <KeyboardArrowLeftIcon />
                    </button>
                    <div className={`helper-text ${isAskingConfirmationBeforeDelete ? "visible" : "hidden"}`}>Are you sure?</div>
                    <button
                        onPointerDown={() => {
                            if (isAskingConfirmationBeforeDelete) {
                                functions.DeleteRecording(idxFixed);
                                recordingsRefresh();
                            }
                            else if (!isAskingConfirmationBeforeDelete) {
                                isAskingConfirmationBeforeDeleteSet(true)
                            }
                        }}
                        className={`button-delete ${isAskingConfirmationBeforeDelete ? " pulse" : " "}`}
                    >
                        <DeleteIcon
                            className="button-delete-icon"
                        />
                    </button>
                </Flex>
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

    // For Arm & Lift
    const armLiftAllChecked = arm && lift;
    const armLiftNoneChecked = !arm && !lift;
    const armLiftIndeterminate = !armLiftAllChecked && !armLiftNoneChecked;

    const armLiftRef = useRef<HTMLInputElement>(null);
    useEffect(() => {
        if (armLiftRef.current) {
            armLiftRef.current.indeterminate = armLiftIndeterminate;
        }
    }, [armLiftIndeterminate]);

    const handleArmLiftParentChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        setArm(e.target.checked);
        setLift(e.target.checked);
    };

    // For Wrist & Gripper
    const wristGripperAllChecked = wristRoll && wristPitch && wristYaw && gripper;
    const wristGripperNoneChecked = !wristRoll && !wristPitch && !wristYaw && !gripper;
    const wristGripperIndeterminate = !wristGripperAllChecked && !wristGripperNoneChecked;

    const wristGripperRef = useRef<HTMLInputElement>(null);
    useEffect(() => {
        if (wristGripperRef.current) {
            wristGripperRef.current.indeterminate = wristGripperIndeterminate;
        }
    }, [wristGripperIndeterminate]);

    const handleWristGripperParentChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        setWristRoll(e.target.checked);
        setWristPitch(e.target.checked);
        setWristYaw(e.target.checked);
        setGripper(e.target.checked);
    };



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


    return (
        <React.Fragment>
            <div id="movement-recorder-container">
                <div ref={refRecordingsList} className="recordings-list">
                    {recordings.length === 0
                        ? (
                            <div className="helper-text-empty-state">
                                <div><LocalFloristIcon fontSize="large" /></div>
                                <div>You haven't made any recordings yet.</div>
                            </div>
                        )
                        : recordingsFiltered.length
                            // sort by newest recordings...
                            ? [...recordingsFiltered].reverse().map((recordingName, idx) => {
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
                            : (
                                <div className="helper-text-empty-state">
                                    <div><SearchIcon fontSize="large" /></div>
                                    <div>No recordings</div>
                                </div>
                            )
                    }

                </div>
                {showRecordingStartButton || isRecording
                    ? (
                        <div className="joints-list" ref={refJointsList}>
                            <div className="heading">Select Joints to Record</div>
                            <div className="subheading">At least 1 joint needs to be selected to begin recording</div>
                            <Flex>
                                <button
                                    onPointerDown={!isOneJointSelected ? selectAllJoints : deselectAllJoints}
                                    disabled={isRecording}
                                    className="button-select-all"
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
                                    <label htmlFor="head">Head</label>
                                </li>
                                <li>
                                    <input
                                        type="checkbox"
                                        id="arm-lift"
                                        ref={armLiftRef}
                                        checked={armLiftAllChecked}
                                        onChange={handleArmLiftParentChange}
                                        disabled={isRecording}
                                    />
                                    <label htmlFor="arm-lift">Arm & Lift</label>
                                    <ul className="checkbox nested">
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
                                            <label htmlFor="arm">Arm</label>
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
                                            <label htmlFor="lift">Lift</label>
                                        </li>
                                    </ul>
                                </li>
                                <li>
                                    <input
                                        type="checkbox"
                                        id="wrist-gripper"
                                        ref={wristGripperRef}
                                        checked={wristGripperAllChecked}
                                        onChange={handleWristGripperParentChange}
                                        disabled={isRecording}
                                    />
                                    <label htmlFor="wrist-gripper">Wrist & Gripper</label>
                                    <ul className="checkbox nested">
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
                                            <label htmlFor="wristRoll">Wrist Twist</label>
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
                                            <label htmlFor="wristPitch">Wrist Bend</label>
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
                                            <label htmlFor="wristYaw">Wrist Rotate</label>
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
                                            <label htmlFor="gripper">Gripper</label>
                                        </li>
                                    </ul>
                                </li>
                            </ul>
                        </div>
                    )
                    : null}

                {isNamingModalVisible
                    ? (
                        <Flex className="naming-modal" direction="column" gap={10}>
                            <div className="heading">Recording Name</div>
                            <Flex style={{ width: '100%' }}>
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
                                    <NotStartedIcon /> Save Recording
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

                {/* Footer */}
                {!isNamingModalVisible
                    ? (<div className="footer">
                        <Flex gap={5} align="center">
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
                            {isFilterActivated || showRecordingStartButton || (isRecording && showRecordingStartButton)
                                ? <button className="button-cancel" onPointerDown={dumpToInitialState}><KeyboardArrowLeftIcon /></button>
                                : null
                            }
                            {recordings.length && !showRecordingStartButton && !isRecording
                                ? <ButtonFilter
                                    isFilterActivated={isFilterActivated}
                                    isFilterActivatedSet={isFilterActivatedSet}
                                    filterQuery={filterQuery}
                                    filterQuerySet={filterQuerySet}
                                />
                                : null}
                        </Flex>
                        <Flex gap={5} align="center" className="button-scroll-wrapper">
                            <button className="button-scroll" onPointerDown={scrollUp} disabled={!canScrollUp}><KeyboardArrowUpIcon /></button>
                            <button className="button-scroll" onPointerDown={scrollDown} disabled={!canScrollDown}><KeyboardArrowDownIcon /></button>
                        </Flex>
                    </div>
                    )
                    : null}
            </div>
        </React.Fragment>
    );
};
