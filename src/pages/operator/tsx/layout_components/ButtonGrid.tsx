import React from "react";
import { buttonFunctionProvider } from "operator/tsx/index";
import { ButtonFunctions, ButtonPadButton, ButtonState } from "../function_providers/ButtonFunctionProvider";
import { CustomizableComponentProps, isSelected } from "./CustomizableComponent";
import { className } from "shared/util";
import "operator/css/ButtonGrid.css"
import { getIcon } from "../utils/svg";
import { ButtonGridDefinition } from "../utils/component_definitions";

const BUTTON_NAMES = [
    "Left",
    "Right",

    "Lift Up",
    "Lift Down",
    "Retract Arm",
    "Extend Arm",

    "Rotate Left",
    "Rotate Right",

    "Open Gripper",
    "Close Gripper"
]

const BUTTON_FUNCTIONS = [
    ButtonPadButton.BaseForward,
    ButtonPadButton.BaseReverse,
    ButtonPadButton.ArmLift,
    ButtonPadButton.ArmLower,
    ButtonPadButton.ArmRetract,
    ButtonPadButton.ArmExtend,
    ButtonPadButton.WristRotateIn,
    ButtonPadButton.WristRotateOut,
    ButtonPadButton.GripperOpen,
    ButtonPadButton.GripperClose
]

const HEADER_NAMES = [
    "Driving Controls",
    "Arm Controls",
    "Wrist Controls",
    "Gripper Controls"
]

const BACKGROUND_COLORS: JSX.Element[] = [];
for (let i = 0; i < 4; i++) {
    BACKGROUND_COLORS.push(
        <span
            key={i}
            style={{
                gridRow: (i + 1) * 2,
                // backgroundColor: `hsl(210, 100%, ${10 + (i * 6)}%)`
            }}
            className="button-grid-bkg-color"
        />
    )
}

/** Properties for {@link ButtonGrid} */
type ButtonGridProps = CustomizableComponentProps & {
    /* Whether to display icons or text labels. */
    displayIcons?: boolean;
}

export const ButtonGrid = (props: ButtonGridProps) => {
    const { customizing } = props.sharedState;
    const selected = isSelected(props);
    const definition = props.definition as ButtonGridDefinition;
    function handleSelect(event: React.MouseEvent<HTMLDivElement>) {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    }
    return (
        <div className={className("button-grid", { selected, customizing })} onClick={handleSelect}>
            {BACKGROUND_COLORS}
            {HEADER_NAMES.map((headerName, idx) => (
                <p key={idx} style={{ gridArea: `header${idx}` }}>{headerName}</p>
            ))}
            {BUTTON_NAMES.map((buttonName, idx) => {
                const buttonFunction = BUTTON_FUNCTIONS[idx];
                const buttonState: ButtonState = props.sharedState.buttonStateMap?.get(buttonFunction) || ButtonState.Inactive;
                const functs: ButtonFunctions = buttonFunctionProvider.provideFunctions(buttonFunction);
                const clickProps = props.sharedState.customizing ? {} : {
                    onMouseDown: functs.onClick,
                    onMouseUp: functs.onRelease,
                    onMouseLeave: functs.onLeave
                }
                return (
                    <button
                        key={idx}
                        style={{ gridArea: `b${idx}` }}
                        {...clickProps}
                        className={buttonState}
                    >
                        {definition.displayIcons == undefined || definition.displayIcons ? 
                            <img height={50} width={50} src={getIcon(buttonFunction)} className={buttonState} />
                        :
                            buttonName
                        }
                    </button>
                );
            })}
        </div>
    )
}