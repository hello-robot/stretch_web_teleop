import React from "react";
import "operator/css/actionmodebutton.css"
import { className } from "shared/util";

/** Enumerator for the possible action modes */
export enum ActionMode {
    StepActions = 'Step-Actions',
    PressRelease = 'Press-Release',
    ClickClick = 'Click-Click'
}

/** Turns {@link ActionMode} enum into an array */
const ActionModes = Object.values(ActionMode) as ActionMode[];

/** Props for {@link ActionModeButton} */
type ActionModeButtonProps = {
    actionMode: ActionMode,
    onChange: (am: ActionMode) => void
}

/**
 * Button and dropdown for changing the action mode.
 */
export const ActionModeButton = (props: ActionModeButtonProps) => {
    const [showModes, setShowModes] = React.useState(false);
    const inputRef = React.useRef<HTMLDivElement>(null);
    const actionMode = props.actionMode;

    // Handler to close dropdown when click outside
    React.useEffect(() => {
        const handler = (e: any) => {
            if (inputRef.current && !inputRef.current.contains(e.target)) {
                setShowModes(false);
                console.log('clicked')
            }
        };
        if (showModes) {
            window.addEventListener("click", handler);
            return () => {
                window.removeEventListener("click", handler);
            };
        }
    });

    /** Maps action modes into selections the user can click on. */
    const mapFunc = (am: any) => {
        const active = actionMode === am;
        return (
            <div
                key={`action-mode-option-${am}`}
                onClick={() => {
                    setShowModes(false);
                    props.onChange(am);
                }}
                className={className("action-mode-option", { active })}
            >
                {am}
            </div>
        )
    }

    return (
        <div ref={inputRef}>
            <button id="action-mode-button" onClick={() => setShowModes(!showModes)}>
                {actionMode}
                <span className="material-icons">expand_more</span>
            </button>
            <div hidden={!showModes} id="action-mode-popup">
                {ActionModes.map(mapFunc)}
            </div>

        </div>
    )
}