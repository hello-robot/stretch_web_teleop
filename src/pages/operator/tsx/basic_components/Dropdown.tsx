import React from "react";
import { className } from "shared/util";
import "operator/css/basic_components.css"

export const Dropdown = <T extends string | JSX.Element>(props: {
    onChange: (selectedIndex: number) => void,
    possibleOptions: T[],
    selectedIndex?: number,
    placeholderText?: string,
    showActive?: boolean,
    top?: boolean
}) => {
    const [showDropdown, setShowDropdown] = React.useState(false);
    const inputRef = React.useRef<HTMLDivElement>(null);
    if (props.selectedIndex === undefined && !props.placeholderText)
        throw Error("both selectedOption and placeholderText undefined");


    // Handler to close dropdown when click outside
    React.useEffect(() => {
        const handler = (e: any) => {
            if (inputRef.current && !inputRef.current.contains(e.target)) {
                setShowDropdown(false);
            }
        };
        if (showDropdown) {
            window.addEventListener("click", handler);
            return () => {
                window.removeEventListener("click", handler);
            };
        }
    });

    function mapFunc(option: T, idx: number) {
        const active = idx === props.selectedIndex;
        if (active && !props.showActive) return null;
        return (
            <button
                key={idx}
                onClick={() => { setShowDropdown(false); if (!active) props.onChange(idx); }}
                className={className("dropdown-option", { active })}
            >
                {option}
            </button>
        )
    }

    return (
        <div ref={inputRef} className="dropdown">
            <button
                className={className("dropdown-button", { "expanded": showDropdown })}
                onClick={() => setShowDropdown(!showDropdown)}
            >
                {props.selectedIndex === undefined ? props.placeholderText : props.possibleOptions[props.selectedIndex]}
                <span className="material-icons">expand_more</span>
            </button>
            <div hidden={!showDropdown} className={className("dropdown-popup", { "top": props.top == undefined ? false : props.top })}>
                {props.possibleOptions.map(mapFunc)}
            </div>

        </div>
    );
}