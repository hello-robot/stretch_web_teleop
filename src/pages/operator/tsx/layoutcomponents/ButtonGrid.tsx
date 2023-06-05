import { buttonFunctionProvider } from "operator/tsx/index";
import { ButtonFunctions, ButtonPadButton, ButtonState } from "../functionprovider/buttonpads";
import { CustomizableComponentProps, isSelected } from "./customizablecomponent";
import "operator/css/ButtonGrid.css"
import { className } from "shared/util";

const BUTTON_NAMES = [
    "Forward",
    "Backwards",
    "Turn Left",
    "Turn Right",

    "Move Lift Up",
    "Move Lift Down",
    "Extend Arm",
    "Collapse Arm",

    "Rotate Left",
    "Rotate Right",

    "Open Gripper",
    "Close Gripper"
]

const BUTTON_FUNCTIONS = [
    ButtonPadButton.BaseForward,
    ButtonPadButton.BaseReverse,
    ButtonPadButton.BaseRotateLeft,
    ButtonPadButton.BaseRotateRight,
    ButtonPadButton.ArmLift,
    ButtonPadButton.ArmLower,
    ButtonPadButton.ArmExtend,
    ButtonPadButton.ArmRetract,
    ButtonPadButton.WristRotateIn,
    ButtonPadButton.WristRotateOut,
    ButtonPadButton.GripperOpen,
    ButtonPadButton.GripperClose
]

const HEADER_NAMES = [
    "Basic Driving Controls",
    "Basic Arm Controls",
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
                backgroundColor: `hsl(317, 79%, ${35 - (6 * i)}%)`
            }}
            className="button-grid-bkg-color"
        />
    )
}

export const ButtonGrid = (props: CustomizableComponentProps) => {
    const { customizing } = props.sharedState;
    const selected = isSelected(props);
    console.log('selected', selected)
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
                        {buttonName}
                    </button>
                );
            })}
        </div>
    )
}