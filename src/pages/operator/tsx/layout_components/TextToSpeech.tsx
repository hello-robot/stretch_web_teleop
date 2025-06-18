import React, { useEffect, useState } from "react";
import { textToSpeechFunctionProvider } from "operator/tsx/index";
import { DropdownInput } from "../basic_components/DropdownInput";
import { Tooltip } from "../static_components/Tooltip";
import "operator/css/TextToSpeech.css";
import "operator/css/basic_components.css";
import { isMobile } from "react-device-detect";
import PlayCircleIcon from "@mui/icons-material/PlayCircle";
import StopCircleIcon from "@mui/icons-material/StopCircle";
import SaveIcon from "@mui/icons-material/Save";
import DeleteForeverIcon from "@mui/icons-material/DeleteForever";

/** All the possible button functions */
export enum TextToSpeechFunction {
    Play,
    Stop,
    SaveText,
    DeleteText,
    SavedTexts,
}

export interface TextToSpeechFunctions {
    Play: (text: string) => void;
    Stop: () => void;
    SaveText: (text: string) => void;
    DeleteText: (text: string) => void;
    SavedTexts: () => string[];
}

export const TextToSpeech = (props: { hideLabels: boolean }) => {
    let functions: TextToSpeechFunctions = {
        SaveText: textToSpeechFunctionProvider.provideFunctions(
            TextToSpeechFunction.SaveText,
        ) as (name: string) => void,
        DeleteText: textToSpeechFunctionProvider.provideFunctions(
            TextToSpeechFunction.DeleteText,
        ) as (text: string) => void,
        SavedTexts: textToSpeechFunctionProvider.provideFunctions(
            TextToSpeechFunction.SavedTexts,
        ) as () => string[],
        Play: textToSpeechFunctionProvider.provideFunctions(
            TextToSpeechFunction.Play,
        ) as (text: string) => void,
        Stop: textToSpeechFunctionProvider.provideFunctions(
            TextToSpeechFunction.Stop,
        ) as () => void,
    };

    const [savedTexts, setSavedTexts] = useState<string[]>(
        functions.SavedTexts(),
    );
    const [selectedIdx, setSelectedIdx] = React.useState<number | undefined>(
        undefined,
    );
    const [text, setText] = React.useState<string>("");

    return !isMobile ? (
        <React.Fragment>
            <div id="text-to-speech-container">Text-to-Speech</div>
            <div id="text-to-speech-container">
                <DropdownInput
                    text={text}
                    setText={setText}
                    selectedIndex={selectedIdx}
                    setSelectedIndex={setSelectedIdx}
                    possibleOptions={savedTexts}
                    placeholderText="Enter text..."
                    placement="bottom"
                    rows={2}
                />
                {/* Play the text */}
                <Tooltip text="Play text" position="top">
                    <button
                        className="play-btn btn-label"
                        onPointerDown={() => {
                            functions.Play(text);
                        }}
                    >
                        <span hidden={props.hideLabels}>Play</span>
                        <PlayCircleIcon />
                    </button>
                </Tooltip>
                {/* Stop the playing text */}
                <Tooltip text="Stop text" position="top">
                    <button
                        className="stop-btn btn-label"
                        onPointerDown={() => {
                            functions.Stop();
                        }}
                    >
                        <span hidden={props.hideLabels}>Stop</span>
                        <StopCircleIcon />
                    </button>
                </Tooltip>
                {/* If we are on saved text, then we show a delete button, else a save button. */}
                {selectedIdx != undefined ? (
                    <Tooltip text="Delete text" position="top">
                        <button
                            className="delete-btn btn-label"
                            onPointerDown={() => {
                                if (selectedIdx != undefined) {
                                    functions.DeleteText(text.trim());
                                }
                                setSavedTexts(functions.SavedTexts());
                                setSelectedIdx(undefined);
                            }}
                        >
                            <span hidden={props.hideLabels}>Delete</span>
                            <DeleteForeverIcon />
                        </button>
                    </Tooltip>
                ) : (
                    <Tooltip text={"Save text"} position="top">
                        <button
                            className="save-btn btn-label"
                            onPointerDown={() => {
                                functions.SaveText(text.trim());
                                setSavedTexts(functions.SavedTexts());
                            }}
                        >
                            <i hidden={props.hideLabels}>Save</i>
                            <SaveIcon />
                        </button>
                    </Tooltip>
                )}
            </div>
        </React.Fragment>
    ) : (
        <React.Fragment>
            <div id="text-to-speech-container">
                Text-to-speech not yet implemented for mobile
            </div>
        </React.Fragment>
    );
};
