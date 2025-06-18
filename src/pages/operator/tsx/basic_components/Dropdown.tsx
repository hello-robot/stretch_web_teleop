import React from "react";
import { className } from "shared/util";
import "operator/css/basic_components.css";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";

export const Dropdown = <T extends string | JSX.Element>(props: {
    onChange: (selectedIndex: number) => void;
    possibleOptions: T[];
    selectedIndex?: number;
    placeholderText?: string;
    showActive?: boolean;
    placement: string;
}) => {
    const [showDropdown, setShowDropdown] = React.useState(false);
    const [placement, setPlacement] = React.useState(props.placement);
    const inputRef = React.useRef<HTMLDivElement>(null);
    const dropdownPopupRef = React.useRef<HTMLDivElement>(null);
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

    // Function to convert each possible option into a button
    function mapFunc(option: T, idx: number) {
        const active = idx === props.selectedIndex;
        if (active && !props.showActive) return null;
        return (
            <button
                key={idx}
                onPointerDown={() => {
                    setShowDropdown(false);
                    if (!active) props.onChange(idx);
                }}
                className={className("dropdown-option", { active })}
            >
                {option}
            </button>
        );
    }

    // Set the max-height of the popup to the screen height minus the top of the popup
    function resizeDropdownPopup() {
        if (dropdownPopupRef.current) {
            const top = dropdownPopupRef.current.getBoundingClientRect().top;
            dropdownPopupRef.current.style.maxHeight = `calc(100vh - ${top}px)`;
        }
    }
    React.useEffect(resizeDropdownPopup, [showDropdown]);
    React.useEffect(() => {
        window.addEventListener("resize", resizeDropdownPopup);
        return () => {
            window.removeEventListener("resize", resizeDropdownPopup);
        };
    });

    return (
        <div ref={inputRef} className="dropdown">
            <button
                className={className("dropdown-button", {
                    expanded: showDropdown,
                    top: props.placement == "top",
                    bottom: props.placement == "bottom",
                })}
                onPointerDown={() => setShowDropdown(!showDropdown)}
            >
                {props.selectedIndex === undefined
                    ? props.placeholderText
                    : props.possibleOptions[props.selectedIndex]}
                <ExpandMoreIcon />
            </button>
            <div
                hidden={!showDropdown}
                className={className("dropdown-popup", {
                    top: props.placement == "top",
                    bottom: props.placement == "bottom",
                })}
                ref={dropdownPopupRef}
            >
                {props.possibleOptions.map(mapFunc)}
            </div>
        </div>
    );
};
