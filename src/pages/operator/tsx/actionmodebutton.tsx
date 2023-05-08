import React from "react";
import "operator/css/actionmodebutton.css"

/** Creates down arrow icon for dropdowns. */
// const DropdownIcon = () => {
//     return (
//         <svg height="20" width="20" viewBox="0 0 20 20">
//             <path d="M4.516 7.548c0.436-0.446 1.043-0.481 1.576 0l3.908 3.747 3.908-3.747c0.533-0.481 1.141-0.446 1.574 0 0.436 0.445 0.408 1.197 0 1.615-0.406 0.418-4.695 4.502-4.695 4.502-0.217 0.223-0.502 0.335-0.787 0.335s-0.57-0.112-0.789-0.335c0 0-4.287-4.084-4.695-4.502s-0.436-1.17 0-1.615z"></path>
//         </svg>
//     );
// };

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
    default: ActionMode,
    onChange: (am: ActionMode) => void
}

/**
 * Button and dropdown for changing the action mode
 */
export const ActionModeButton = (props: ActionModeButtonProps) => {
    const [actionMode, setActionMode] = React.useState(props.default);
    const [showModes, setShowModes] = React.useState(false);
    const inputRef = React.useRef<HTMLDivElement>(null);

    // Handler to close dropdown when click outside z
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

    /** Maps action modes to divs to click on. */
    const mapFunc = (am: any) => {
        const isActive = actionMode === am;
        return (
            <div
                onClick={() => { 
                    setActionMode(am); 
                    setShowModes(false);
                    props.onChange(am);
                }}
                className={"action-mode-option" + (isActive ? " active" : "")}
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