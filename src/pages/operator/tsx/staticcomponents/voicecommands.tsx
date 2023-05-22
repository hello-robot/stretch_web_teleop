import { useState, useRef } from "react";
import SpeechRecognition, { useSpeechRecognition } from "react-speech-recognition";
import "operator/css/voicecommands.css"

export const VoiceCommands = () => {
    const commands = [
        {
            command: "set speed *",
            callback: (speed: string) => {
                console.log("setting speed to: ", speed)
            },
        },
        {
            command: "drive forward",
            callback: () => {
                console.log("drive forward")
            },
        },
    ];

    const { transcript, resetTranscript } = useSpeechRecognition({ commands });
    const [isListening, setIsListening] = useState(false);
    const microphoneRef = useRef<HTMLButtonElement>(null);
    if (!SpeechRecognition.browserSupportsSpeechRecognition()) {
        return (
            <div className="mircophone-container">
                Browser is not Support Speech Recognition.
            </div>
        );
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

    return (
        <button
            id="microphone-button"
            ref={microphoneRef}
            onClick={listenHandle}
        >
            {isListening
                ? <span className="material-icons">mic_off</span>
                : <span className="material-icons">mic</span>
            }
        </button>
    );
}
