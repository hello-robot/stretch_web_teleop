import React from "react";
import { className } from "shared/util";
import "operator/css/RadioGroup.css";
import Delete from "@mui/icons-material/DeleteOutline";
import ModeEdit from "@mui/icons-material/ModeEditOutline";
import { isBrowser, isTablet } from "react-device-detect";

export const RadioButton = (props: {
    label: string;
    selected: boolean;
    onClick: () => void;
    functs: RadioFunctions;
}) => {
    return (
        <div
            className={!isBrowser && !isTablet  ? "radio-btn-mobile" : "radio-btn"}
            onPointerDown={props.onClick}
        >
            <label key={props.label}>
                <input
                    type="radio"
                    className={!isBrowser && !isTablet  ? "radio-mobile" : "radio"}
                    value={props.label}
                    key={props.label}
                    checked={props.selected}
                    onChange={props.onClick}
                />
                {props.label}
            </label>
            <div className="modify">
                {props.functs.Edit && <ModeEdit />}
                {props.functs.Delete && (
                    <span
                        onPointerDown={() => props.functs.Delete!(props.label)}
                    ><Delete /></span>
                )}
            </div>
        </div>
    );
};

export interface RadioFunctions {
    GetLabels: () => string[];
    SelectedLabel: (label: string) => void;
    // Add?: () => void
    Edit?: (label: string) => void;
    Delete?: (label: string) => void;
    // Start?: (label: string) => void,
    // Cancel?: () => void
}

export const RadioGroup = (props: { 
    functs: RadioFunctions,
    onChange? (label: string): void}
) => {
    const [selected, setSelected] = React.useState<string>();

    return (
        <div
            className={!isBrowser && !isTablet  ? "radio-group-mobile" : "radio-group"}
            onContextMenu={(e) => e.preventDefault()}
        >
            {props.functs.GetLabels().map((label, index) => (
                <RadioButton
                    key={label}
                    label={label}
                    selected={selected === label}
                    onClick={() => {
                        if (selected === label) {
                            setSelected("");
                            props.functs.SelectedLabel("");
                        } else {
                            setSelected(label);
                            props.functs.SelectedLabel(label);
                        }
                        
                        if (props.onChange) props.onChange(label)
                    }}
                    functs={props.functs}
                />
            ))}
            {/* {props.functs.Add &&
                <span className="material-icons add-btn">
                    add
                </span>
            }
            {props.functs.Start &&
                <span className="material-icons start-btn">
                    play_arrow
                </span>
            } */}
        </div>
    );
};
