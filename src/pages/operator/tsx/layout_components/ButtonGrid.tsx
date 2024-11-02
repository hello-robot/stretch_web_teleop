import { buttonFunctionProvider } from "operator/tsx/index";
import {
    ButtonFunctions,
    ButtonPadButton,
    ButtonState,
} from "../function_providers/ButtonFunctionProvider";
import {
    CustomizableComponentProps,
    isSelected,
} from "./CustomizableComponent";
import { className } from "shared/util";
import "operator/css/ButtonGrid.css";

const BUTTON_NAMES = [
    "Forward",
    "Backwards",
    "Turn Left",
    "Turn Right",

    "Lift Up",
    "Lift Down",
    "Extend Arm",
    "Retract Arm",

    "Roll Left",
    "Roll Right",
    "Pitch Up",
    "Pitch Down",
    "Rotate Left",
    "Rotate Right",

    "Open Gripper",
    "Close Gripper",
];

const BUTTON_FUNCTIONS = [
    ButtonPadButton.BaseForward,
    ButtonPadButton.BaseReverse,
    ButtonPadButton.BaseRotateLeft,
    ButtonPadButton.BaseRotateRight,
    ButtonPadButton.ArmLift,
    ButtonPadButton.ArmLower,
    ButtonPadButton.ArmExtend,
    ButtonPadButton.ArmRetract,
    ButtonPadButton.WristRollLeft,
    ButtonPadButton.WristRollRight,
    ButtonPadButton.WristPitchUp,
    ButtonPadButton.WristPitchDown,
    ButtonPadButton.WristRotateIn,
    ButtonPadButton.WristRotateOut,
    ButtonPadButton.GripperOpen,
    ButtonPadButton.GripperClose,
];

const HEADER_NAMES = [
    "Base",
    "Arm/Lift",
    "Wrist/Gripper",
    // "Gripper Controls"
];

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
        />,
    );
}

export const ButtonGrid = (props: CustomizableComponentProps) => {
    const { customizing } = props.sharedState;
    const selected = isSelected(props);
    function handleSelect(event: React.MouseEvent<HTMLDivElement>) {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    }
    return (
        <div
            className={className("button-grid", { selected, customizing })}
            onClick={handleSelect}
        >
            {/* {BACKGROUND_COLORS} */}
            {HEADER_NAMES.map((headerName, idx) => (
                <p key={idx} style={{ gridArea: `header${idx}` }}>
                    {headerName}
                </p>
            ))}
            {BUTTON_NAMES.map((buttonName, idx) => {
                const buttonFunction = BUTTON_FUNCTIONS[idx];
                const buttonState: ButtonState =
                    props.sharedState.buttonStateMap?.get(buttonFunction) ||
                    ButtonState.Inactive;
                const functs: ButtonFunctions =
                    buttonFunctionProvider.provideFunctions(buttonFunction);
                const clickProps = props.sharedState.customizing
                    ? {}
                    : {
                          onPointerDown: functs.onClick,
                          onPointerUp: functs.onRelease,
                          onPointerLeave: functs.onLeave,
                      };
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
    );
};
