import React from "react";
import "operator/css/basiccomponents.css"
import { className } from "shared/util";
import { relative } from "path";

export const Dropdown = <T extends string>(props: {
    onChange: (option: T) => void,
    selectedOption?: T,
    possibleOptions: T[]
    placeholderText?: string,
    showActive?: boolean
}) => {
    const [showModes, setShowModes] = React.useState(false);
    const inputRef = React.useRef<HTMLDivElement>(null);
    if (!props.selectedOption && !props.placeholderText)
        throw Error("both selectedOption and placeholderText undefined");
    function mapFunc(option: T) {
        const active = option === props.selectedOption;
        if (active && !props.showActive) return null;
        return (
            <button
                key={option}
                onClick={() => { setShowModes(false); if (!active) props.onChange(option); }}
                className={className("dropdown-option", { active })}
            >
                {option}
            </button>
        )
    }

    return (
        <div ref={inputRef} className="dropdown">
            <button
                className={className("dropdown-button", { "expanded": showModes })}
                onClick={() => setShowModes(!showModes)}
            >
                {props.selectedOption || props.placeholderText}
                <span className="material-icons">expand_more</span>
            </button>
            <div hidden={!showModes} className="dropdown-popup">
                {props.possibleOptions.map(mapFunc)}
            </div>

        </div>
    );
}