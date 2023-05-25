import { useState, useRef, useEffect } from "react";
import SpeechRecognition, { useSpeechRecognition } from "react-speech-recognition";
import { voiceFunctionProvider } from "operator/tsx/index";
import "operator/css/voicecommands.css"

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
            </div>
        );
    }
}
