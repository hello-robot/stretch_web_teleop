import React from "react";
import PlayCircleFilledIcon from "@mui/icons-material/PlayCircleFilled";

import {
    buttonFunctionProvider,
} from "..";
import {
    ButtonPadButton,
} from "../function_providers/ButtonFunctionProvider";

/**
 * Creates a single button for controlling the pan or tilt of the realsense camera
 *
 * @param props the direction of the button {@link PanTiltButton}
 */
export const PanTiltButton = (props: { direction: ButtonPadButton }) => {
    let gridPosition: { gridRow: number; gridColumn: number }; // the position in the 3x3 grid around the video element
    let rotation: string; // how to rotate the arrow icon to point in the correct direction
    const functs = buttonFunctionProvider.provideFunctions(props.direction);

    // Specify button details based on the direction
    switch (props.direction) {
        case ButtonPadButton.CameraTiltUp:
            gridPosition = { gridRow: 1, gridColumn: 2 };
            rotation = "-90";
            break;
        case ButtonPadButton.CameraTiltDown:
            gridPosition = { gridRow: 3, gridColumn: 2 };
            rotation = "90";
            break;
        case ButtonPadButton.CameraPanLeft:
            gridPosition = { gridRow: 2, gridColumn: 1 };
            rotation = "180";
            break;
        case ButtonPadButton.CameraPanRight:
            gridPosition = { gridRow: 2, gridColumn: 3 };
            rotation = "0"; // by default the arrow icon points right
            break;
        default:
            throw Error(`unknown pan tilt button direction ${props.direction}`);
    }

    return (
        <button
            style={gridPosition}
            className={props.direction}
            onMouseDown={functs.onClick}
            onMouseUp={functs.onRelease}
            onMouseLeave={functs.onLeave}
        >
            <PlayCircleFilledIcon
                className="panTiltIcon"
                style={{ transform: `rotate(${rotation}deg)` }}
            />
        </button>
    );
};