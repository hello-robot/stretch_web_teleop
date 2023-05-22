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
    Stop
}

/** Defining keyword and associated callback. */
export type VoiceCommandFunctions = {
    command: string,
    callback: (speed?: string) => void
}

export const VoiceCommands = () => {
    const { transcript, resetTranscript } = useSpeechRecognition({ commands: createCommands() });
    const [isListening, setIsListening] = useState(false);
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
            VoiceCommandFunction.Stop
        ]
    
        let commands: VoiceCommandFunctions[] = functions.map((funct: VoiceCommandFunction) => {
            return {
                ...voiceFunctionProvider.provideFunctions(funct) as VoiceCommandFunctions,
            };
        });
        return commands
    }

    const listenHandle = () => {
        if (!isListening) {
            setIsListening(true);
            microphoneRef.current?.classList.add("listening");
            SpeechRecognition.startListening({
                continuous: true,
            });
        } else {
            setIsListening(false);
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
        );
    }
}
