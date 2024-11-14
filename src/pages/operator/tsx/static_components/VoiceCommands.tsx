import React, { useState, useRef, useEffect } from "react";
import SpeechRecognition, { useSpeechRecognition } from "react-speech-recognition";
import { voiceFunctionProvider } from "operator/tsx/index";
import MicOff from "@mui/icons-material/MicOff";
import Mic from "@mui/icons-material/Mic";
import Info from "@mui/icons-material/Info";
import RadioButtonChecked from "@mui/icons-material/RadioButtonChecked" 
import "operator/css/basic_components.css"
import "operator/css/VoiceCommands.css"
import { ActionMode, LayoutDefinition } from "../utils/component_definitions";
import { UnderVideoButton } from "../function_providers/UnderVideoFunctionProvider";

/** All the possible button functions */
export enum VoiceCommandFunction {
    // Button Pad Functions
    BaseForward,
    BaseReverse,
    BaseRotateRight,
    BaseRotateLeft,
    ArmLift,
    ArmLower,
    ArmExtend,
    ArmRetract,
    GripperOpen,
    GripperClose,
    WristRotateIn,
    WristRotateOut,
    WristPitchUp,
    WristPitchDown,
    WristRollLeft,
    WristRollRight,
    TiltUp,
    TiltDown,
    PanLeft,
    PanRight,
    // Under video functions
    FollowGripper,
    PredictiveDisplay,
    RealsenseDepthSensing,
    DetectObjects,
    SelectDetectedObject,
    SelectObject,
    HorizontalGrasp,
    VerticalGrasp,
    LookAhead,
    LookAtGripper,
    LookAtBase,
    CenterWrist,
    // Global functions
    Stop,
    SetSpeed,
    SetActionMode,
    FeedingMode,
    DefaultMode,
    SelectBite,
    GoTo,
    Cancel,
    StowArm
}

/** Defining keyword and associated callback. */
export type VoiceCommandFunctions = {
    command: string,
    callback: (speed?: string) => void
}
export type VoiceCommandsProps = {
     /**
     * Callback function when a new speed is selected.
     * @param newScale the new selected speed
     */
      onUpdateVelocityScale: (newScale: number) => void;

    /**
     * Callback function when a new action mode is selected.
     * @param newActionMode the new selected action mode
     */
      onUpdateActionMode: (newActionMode: ActionMode) => void;

    /**
     * Callback function to toggle buttons under the camera views.
     * @param toggle boolean indicating whether to enable or disable the button
     */
    onToggleUnderVideoButtons: (toggle: boolean, button: UnderVideoButton) => void;
}

export const VoiceCommands = (props: VoiceCommandsProps) => {
    const { transcript, resetTranscript } = useSpeechRecognition({ commands: createCommands() });
    const [isListening, setIsListening] = useState(false);
    const [display, setDisplay] = useState("Microphone off");
    const [showModal, setShowModal] = useState(false);
    const microphoneRef = useRef<HTMLButtonElement>(null);

    function createCommands(): VoiceCommandFunctions[] {
        let functions: VoiceCommandFunction[] = [
            VoiceCommandFunction.BaseForward,
            VoiceCommandFunction.BaseReverse,
            VoiceCommandFunction.BaseRotateRight,
            VoiceCommandFunction.BaseRotateLeft,
            VoiceCommandFunction.ArmLift,
            VoiceCommandFunction.ArmLower,
            VoiceCommandFunction.ArmExtend,
            VoiceCommandFunction.ArmRetract,
            VoiceCommandFunction.GripperOpen,
            VoiceCommandFunction.GripperClose,
            VoiceCommandFunction.WristRotateIn,
            VoiceCommandFunction.WristRotateOut,
            VoiceCommandFunction.WristPitchUp,
            VoiceCommandFunction.WristPitchDown,
            VoiceCommandFunction.WristRollLeft,
            VoiceCommandFunction.WristRollRight,
            VoiceCommandFunction.TiltUp,
            VoiceCommandFunction.TiltDown,
            VoiceCommandFunction.PanLeft,
            VoiceCommandFunction.PanRight,
            VoiceCommandFunction.FollowGripper,
            VoiceCommandFunction.PredictiveDisplay,
            VoiceCommandFunction.RealsenseDepthSensing,
            VoiceCommandFunction.DetectObjects,
            VoiceCommandFunction.SelectDetectedObject,
            VoiceCommandFunction.SelectObject,
            VoiceCommandFunction.HorizontalGrasp,
            VoiceCommandFunction.VerticalGrasp,
            VoiceCommandFunction.Stop,
            VoiceCommandFunction.SetSpeed,
            VoiceCommandFunction.SetActionMode,
            VoiceCommandFunction.LookAhead,
            VoiceCommandFunction.LookAtBase,
            VoiceCommandFunction.LookAtGripper,
            VoiceCommandFunction.CenterWrist,
            VoiceCommandFunction.FeedingMode,
            VoiceCommandFunction.DefaultMode,
            VoiceCommandFunction.SelectBite,
            VoiceCommandFunction.GoTo,
            VoiceCommandFunction.Cancel,
            VoiceCommandFunction.StowArm
        ]
    
        let commands: VoiceCommandFunctions[] = functions.map((funct: VoiceCommandFunction) => {
            return {
                ...voiceFunctionProvider.provideFunctions(
                    funct, 
                    props, 
                    (command: string) => {
                        setDisplay(command)
                        setTimeout(() => {
                            setDisplay('Listening...');
                        }, 2000);
                    }
                ) as VoiceCommandFunctions,
            };
        });
        return commands
    }

    const listenHandle = () => {
        if (!isListening) {
            setIsListening(true);
            setDisplay("Listening...")
            microphoneRef.current?.classList.add("listening");
            SpeechRecognition.startListening({
                continuous: true,
            });
        } else {
            setIsListening(false);
            setDisplay("Microphone off");
            microphoneRef.current?.classList.remove("listening");
            SpeechRecognition.stopListening();
            resetTranscript();
        }
    };

    const commandsModalHandle = () => {
        if (showModal) {
            setShowModal(false);
        } else {
            setShowModal(true);
        }
    }

    const commandsModal = () => {
        return (
            <React.Fragment>
                <div className="voice-commands-popup-modal">
                    <div id="close-modal">
                        <button onClick={() => setShowModal(false)}>
                            <span className="material-icons">cancel</span>
                        </button>
                    </div>
                <div id="commands">
                    <p>Commands: </p>
                    <ul>
                        <li>Drive Forward</li> 
                        <li>Drive Backward</li> 
                        <li>Rotate Robot Left</li> 
                        <li>Rotate Robot Right</li> 
                        <li>Lower Arm</li>
                        <li>Raise Arm</li>
                        <li>Extend Arm</li>
                        <li>Retract Arm</li>
                        <li>Rotate Wrist Counterclockwise</li>
                        <li>Rotate Wrist Clockwise</li>
                        <li>Open Gripper</li>
                        <li>Close Gripper</li>
                        <li>Stop</li>
                        <li>Set Speed To *</li>
                    </ul>
                </div>
                </div>
                <div onClick={() => setShowModal(false)} id="popup-background"></div>
            </React.Fragment>
        )
    }

    if (!SpeechRecognition.browserSupportsSpeechRecognition()) {
        return (
            <div className="mircophone-container">
                Browser is not Support Speech Recognition.
            </div>
        );
    } else {
        return (
            <>
                <div className="voiceControlContainer">
                    <button
                        className="microphoneButton"
                        ref={microphoneRef}
                        onClick={listenHandle}
                    >
                        {isListening
                            ? <Mic />
                            : <MicOff />}
                    </button>
                </div>
                <div className="operator-voice-commands">
                    <div id="voice-command-container">
                        {isListening
                            ? <><span id="record-icon"><RadioButtonChecked /></span><p>{display}</p></>
                            : <></>
                        }
                        {/* <div onClick={commandsModalHandle}>
                            <button>
                                <Info />
                            </button>
                            {showModal
                                ? commandsModal()
                                : null}
                        </div> */}
                    </div>
                </div>
            </>
        );
    }
}