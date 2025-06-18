import { className } from "shared/util";
import "operator/css/basic_components.css";
import { isBrowser, isTablet } from "react-device-detect";
import React from "react";
import CheckBoxOutlineBlankIcon from "@mui/icons-material/CheckBoxOutlineBlank";
import CheckBoxIcon from "@mui/icons-material/CheckBox";

/**
 * Properties for {@link CheckToggleButton}
 */
type CheckToggleButtonProps = {
    /** Toggled on if true, toggled off if false. */
    checked: boolean;
    /**
     * Function when button is clicked, this should probably toggle the state
     * of `checked`
     */
    onPointerDown: () => void;
    /**
     * Text to display on the button to the right of the checkbox.
     */
    label: string;
};

/**
 * A button with a check box on the left side to indicate if the button is
 * toggled on or off.
 *
 * @param props {@link CheckToggleButtonProps}
 */
export const CheckToggleButton = (props: CheckToggleButtonProps) => {
    const { checked } = props;
    const icon = checked ? <CheckBoxIcon /> : <CheckBoxOutlineBlankIcon />;
    return (
        <button
            aria-label={props.label}
            aria-required="true"
            className={className(
                !isBrowser && !isTablet  ? "check-toggle-button-mobile" : "check-toggle-button",
                { checked },
            )}
            onPointerDown={props.onPointerDown}
        >
            <span className={"material-icons"}>{icon}</span>
            {props.label}
        </button>
    );
};
