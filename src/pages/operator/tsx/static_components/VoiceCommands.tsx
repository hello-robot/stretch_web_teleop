import React, { useState, useRef, useEffect } from "react";
// import SpeechRecognition, { useSpeechRecognition } from "react-speech-recognition";
import { voiceFunctionProvider } from "operator/tsx/index";
import MicOff from "@mui/icons-material/MicOff";
import Mic from "@mui/icons-material/Mic";
import Info from "@mui/icons-material/Info";
import RadioButtonChecked from "@mui/icons-material/RadioButtonChecked" 
import "operator/css/basic_components.css"
import "operator/css/VoiceCommands.css"
import { ActionMode, LayoutDefinition } from "../utils/component_definitions";
import { UnderVideoButton } from "../function_providers/UnderVideoFunctionProvider";
// import useSpeechToText from 'react-hook-speech-to-text';

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
    // Under video functions
    FollowGripper,
    PredictiveDisplay,
    RealsenseDepthSensing,
    DetectObjects,
    SelectDetectedObject,
    SelectObject,
    HorizontalGrasp,
    VerticalGrasp,
    // Global functions
    Stop,
    SetSpeed,
    SetActionMode
}

export enum Speed {
    Slowest = 0,
    Slow = 1,
    Medium = 2,
    Fast = 3,
    Fastest = 4
}

export enum CheckboxAction {
    Check,
    Uncheck
}

export interface VoiceCommand {
    funct: VoiceCommandFunction;
    arg?: Speed | ActionMode | CheckboxAction;
}

const commandFuncts: Map<string, VoiceCommand> = new Map([
    ['slowest', { funct: VoiceCommandFunction.SetSpeed, arg: Speed.Slowest}],
    ['slow', { funct: VoiceCommandFunction.SetSpeed, arg: Speed.Slow}],
    ['medium', { funct: VoiceCommandFunction.SetSpeed, arg: Speed.Medium}],
    ['fast', { funct: VoiceCommandFunction.SetSpeed, arg: Speed.Fast}],
    ['fastest', { funct: VoiceCommandFunction.SetSpeed, arg: Speed.Fastest}],
    ['step actions', { funct: VoiceCommandFunction.SetActionMode, arg: ActionMode.StepActions}],    
    ['press and hold', { funct: VoiceCommandFunction.SetActionMode, arg: ActionMode.PressAndHold}],    
    ['click click', { funct: VoiceCommandFunction.SetActionMode, arg: ActionMode.ClickClick}],  
    ['forward', { funct: VoiceCommandFunction.BaseForward}],    
    ['backward', { funct: VoiceCommandFunction.BaseReverse}],    
    ['turn left', { funct: VoiceCommandFunction.BaseRotateLeft}],    
    ['turn right', { funct: VoiceCommandFunction.BaseRotateRight}],    
    ['raise arm', { funct: VoiceCommandFunction.ArmLift}],    
    ['lower arm', { funct: VoiceCommandFunction.ArmLower}],    
    ['extend arm', { funct: VoiceCommandFunction.ArmExtend}],    
    ['retract arm', { funct: VoiceCommandFunction.ArmRetract}],    
    ['open gripper', { funct: VoiceCommandFunction.GripperOpen}],    
    ['close gripper', { funct: VoiceCommandFunction.GripperClose}], 
    ['rotate left', { funct: VoiceCommandFunction.BaseRotateLeft}],    
    ['rotate right', { funct: VoiceCommandFunction.BaseRotateRight}],    
    ['pitch up', { funct: VoiceCommandFunction.WristPitchUp}],    
    ['pitch down', { funct: VoiceCommandFunction.WristPitchDown}],    
    ['roll left', { funct: VoiceCommandFunction.WristRollLeft}],    
    ['roll right', { funct: VoiceCommandFunction.WristRollRight}],    
    ['check follow gripper', { funct: VoiceCommandFunction.FollowGripper, arg: CheckboxAction.Check}],    
    ['uncheck follow gripper', { funct: VoiceCommandFunction.FollowGripper, arg: CheckboxAction.Uncheck}], 
    ['check predictive display', { funct: VoiceCommandFunction.PredictiveDisplay, arg: CheckboxAction.Check}],    
    ['uncheck predictive display', { funct: VoiceCommandFunction.PredictiveDisplay, arg: CheckboxAction.Uncheck}], 
    ['check depth sensing', { funct: VoiceCommandFunction.RealsenseDepthSensing, arg: CheckboxAction.Check}],    
    ['uncheck depth sensing', { funct: VoiceCommandFunction.RealsenseDepthSensing, arg: CheckboxAction.Uncheck}],    
])

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
    // const { transcript, resetTranscript } = useSpeechRecognition({ commands: createCommands() });
    const [isListening, setIsListening] = useState(false);
    const [display, setDisplay] = useState("Microphone off");
    const [showModal, setShowModal] = useState(false);
    const [command, setCommand] = useState<string>();
    const microphoneRef = useRef<HTMLButtonElement>(null);

    const AudioContext = window.AudioContext || (window as any).webkitAudioContext;
    const SpeechRecognition =
        window.SpeechRecognition || (window as any).webkitSpeechRecognition;

    const audioContextRef = useRef<AudioContext>();
    let recognition = new SpeechRecognition();
    recognition.continuous = true;
    recognition.interimResults = false;
    recognition.maxAlternatives = 2;

    recognition.onresult = (e) => {
        const results = e.results[e.results.length - 1];
        console.log(results)
        for (let i = 0; i < results.length; i++) {
            let command = results[i].transcript.trimStart().toLowerCase()
            if (commandFuncts.has(command)) {
                setCommand(command)
                let voiceCommand = commandFuncts.get(command)
                voiceFunctionProvider.provideFunctions(voiceCommand.funct, voiceCommand.arg, props)()
            }
        }
    }

    if (!audioContextRef.current) {
        audioContextRef.current = new AudioContext();
    }
    // let {
    //     isRecording,
    //     startSpeechToText,
    //     stopSpeechToText,
    //     results,
    // } = useSpeechToText({
    //     continuous: true,
    //     useLegacyResults: false,
    // })

    // useEffect(() => {
    //     if(results.length > 0) {
    //       const latestResult = results[results.length - 1]
    //       const { transcript } = latestResult
    //     }
    //   }, [results])

    function executeCommand(command: string) {
        if (commandFuncts.has(command)) {
            console.log(command)
            let voiceCommand = commandFuncts.get(command)
            voiceFunctionProvider.provideFunctions(voiceCommand.funct, voiceCommand.arg, props)()
        }
    }

    // function createCommands(): VoiceCommandFunctions[] {
    //     let functions: VoiceCommandFunction[] = [
    //         VoiceCommandFunction.BaseForward,
    //         VoiceCommandFunction.BaseReverse,
    //         VoiceCommandFunction.BaseRotateRight,
    //         VoiceCommandFunction.BaseRotateLeft,
    //         VoiceCommandFunction.ArmLift,
    //         VoiceCommandFunction.ArmLower,
    //         VoiceCommandFunction.ArmExtend,
    //         VoiceCommandFunction.ArmRetract,
    //         VoiceCommandFunction.GripperOpen,
    //         VoiceCommandFunction.GripperClose,
    //         VoiceCommandFunction.WristRotateIn,
    //         VoiceCommandFunction.WristRotateOut,
    //         VoiceCommandFunction.WristPitchUp,
    //         VoiceCommandFunction.WristPitchDown,
    //         VoiceCommandFunction.WristRollLeft,
    //         VoiceCommandFunction.WristRollRight,
    //         VoiceCommandFunction.FollowGripper,
    //         VoiceCommandFunction.PredictiveDisplay,
    //         VoiceCommandFunction.RealsenseDepthSensing,
    //         VoiceCommandFunction.DetectObjects,
    //         VoiceCommandFunction.SelectDetectedObject,
    //         VoiceCommandFunction.SelectObject,
    //         VoiceCommandFunction.HorizontalGrasp,
    //         VoiceCommandFunction.VerticalGrasp,
    //         VoiceCommandFunction.Stop,
    //         VoiceCommandFunction.SetSpeed,
    //         VoiceCommandFunction.SetActionMode
    //     ]
    
    //     let commands: VoiceCommandFunctions[] = functions.map((funct: VoiceCommandFunction) => {
    //         return {
    //             ...voiceFunctionProvider.provideFunctions(
    //                 funct, 
    //                 props, 
    //                 (command: string) => {
    //                     setDisplay(command)
    //                     setTimeout(() => {
    //                         setDisplay('Listening...');
    //                     }, 2000);
    //                 }
    //             ) as VoiceCommandFunctions,
    //         };
    //     });
    //     return commands
    // }

    const listenHandle = () => {
        if (!isListening) {
            setIsListening(true);
            setDisplay("Listening...")
            // startSpeechToText()
            recognition.start();
            microphoneRef.current?.classList.add("listening");
            // SpeechRecognition.startListening({
            //     continuous: true,
            // });
        } else {
            setIsListening(false);
            setDisplay("Microphone off");
            microphoneRef.current?.classList.remove("listening");
            // stopSpeechToText()
            // SpeechRecognition.stopListening();
            // resetTranscript();
        }
    };

    // if (!SpeechRecognition.browserSupportsSpeechRecognition()) {
    //     return (
    //         <div className="mircophone-container">
    //             Browser is not Support Speech Recognition.
    //         </div>
    //     );
    // } else {
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
                            ? <><span id="record-icon"><RadioButtonChecked /></span>
                                <p>{command}</p>
                                {/* {results && results.length > 0 
                                    ? <p>{results.pop().transcript}</p> : ""} */}
                              </>
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
