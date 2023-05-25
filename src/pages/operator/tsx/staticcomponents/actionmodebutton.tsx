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
    /** The current action mode */
    actionMode: ActionMode,
    /** Callback function when a new action mode is selected */
    onChange: (am: ActionMode) => void
}

/**
 * Button and dropdown for changing the action mode.
 */
export const ActionModeButton = (props: ActionModeButtonProps) => {
    const [showModes, setShowModes] = React.useState(false);
    const inputRef = React.useRef<HTMLDivElement>(null);

    // Handler to close dropdown when click outside
    React.useEffect(() => {
        const handler = (e: any) => {
            if (inputRef.current && !inputRef.current.contains(e.target)) {
                setShowModes(false);
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
    const mapFunc = (thisActionMode: ActionMode) => {
        const active = props.actionMode === thisActionMode;
        return (
            <button
                key={`action-mode-option-${thisActionMode}`}
                onClick={() => {
                    setShowModes(false);
                    // Only record change if selected is different from already active
                    if (!active) props.onChange(thisActionMode);
                }}
                className={className("action-mode-option", { active })}
            >
                {thisActionMode}
            </button>
        )
    }

    return (
        <div ref={inputRef}>
            <button
                id="action-mode-button"
                className={showModes ? "expanded" : ""}
                onClick={() => setShowModes(!showModes)}
            >
                {props.actionMode}
                <span className="material-icons">expand_more</span>
            </button>
            <div hidden={!showModes} id="action-mode-popup">
                {ActionModes.map(mapFunc)}
            </div>

        </div>
    )
}