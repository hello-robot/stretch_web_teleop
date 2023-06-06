import { useState, useRef, useEffect } from "react";
import SpeechRecognition, { useSpeechRecognition } from "react-speech-recognition";
import { voiceFunctionProvider } from "operator/tsx/index";
import "operator/css/voicecommands.css"
import "operator/css/basic_components.css"
import React from "react";

/** All the possible button functions */
export enum VoiceCommandFunction {
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
    Stop,
    SetSpeed
}

/** Defining keyword and associated callback. */
export type VoiceCommandFunctions = {
    command: string,
    callback: (speed?: string) => void
}
type VoiceCommandsProps = {
     /**
     * Callback function when a new speed is selected.
     * @param newScale the new selected speed
     */
      onUpdateVelocityScale: (newScale: number) => void;
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
            VoiceCommandFunction.Stop,
            VoiceCommandFunction.SetSpeed
        ]
    
        let commands: VoiceCommandFunctions[] = functions.map((funct: VoiceCommandFunction) => {
            return {
                ...voiceFunctionProvider.provideFunctions(
                    funct, 
                    props.onUpdateVelocityScale, 
                    (command: string) => setDisplay(command)
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
            <div id="voice-command-container">
                {isListening
                    ? <span className="material-icons" id="record-icon">radio_button_checked</span>
                    : <></>
                }
                <button
                    id="microphone-button"
                    ref={microphoneRef}
                    onClick={listenHandle}
                >
                    {isListening
                        ? <span className="material-icons">mic</span>
                        : <span className="material-icons">mic_off</span>
                    }
                </button>
                <p>{display}</p>
                <div onClick={commandsModalHandle}>
                    <button>
                        <span className="material-icons">info</span>
                    </button>
                    {showModal 
                        ? commandsModal()
                        : null
                    }
                </div>
            </div>
        );
    }
}
