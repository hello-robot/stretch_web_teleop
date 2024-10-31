import React from "react";
import {
    CustomizableComponentProps,
    SharedState,
    isSelected,
} from "./CustomizableComponent";
import {
    ButtonPadDefinition,
    ButtonPadId,
    ButtonPadIdMobile,
} from "../utils/component_definitions";
import { className } from "shared/util";
import { buttonFunctionProvider } from "operator/tsx/index";
import {
    ButtonPadShape,
    getIcon,
    getPathsFromShape,
    SVG_RESOLUTION,
} from "../utils/svg";
import {
    ButtonFunctions,
    ButtonPadButton,
    ButtonState,
} from "../function_providers/ButtonFunctionProvider";
import { isBrowser, isTablet } from "react-device-detect";
import "operator/css/ButtonPad.css";

/** Properties for {@link ButtonPad} */
type ButtonPadProps = CustomizableComponentProps & {
    /* If the button pad is overlaid on a camera view. */
    overlay?: boolean;
    /* Aspect ratio of the button pad */
    aspectRatio?: number;
};

/**
 * A set of buttons which can be overlaid as a child of a camera view or
 * standalone.
 *
 * @param props {@link ButtonPadProps}
 */
export const ButtonPad = (props: ButtonPadProps) => {
    /** Reference to the SVG which makes up the button pad */
    const svgRef = React.useRef<SVGSVGElement>(null);
    /** List of path shapes for each button on the button pad */
    const definition = props.definition as ButtonPadDefinition;
    const id: ButtonPadId = definition.id;
    if (!id) throw Error("Undefined button pad ID at path " + props.path);
    const [shape, functions] = getShapeAndFunctionsFromId(definition.id);
    const [paths, iconPositions] = getPathsFromShape(shape, props.aspectRatio);

    // Paths and functions should be the same length
    if (paths.length !== functions.length) {
        throw Error(
            `paths length: ${paths.length}, functions length: ${functions.length}`,
        );
    }

    const { customizing } = props.sharedState;
    const { overlay } = props;
    const selected = isSelected(props);

    /** Uses the paths and buttonsProps to create the buttons */
    function mapPaths(svgPath: string, i: number) {
        const buttonProps = {
            iconPosition: iconPositions[i],
            svgPath,
            funct: functions[i],
            sharedState: props.sharedState,
        };
        // Buttons will not function during customization mode
        return <SingleButton {...buttonProps} key={i} />;
    }

    /** Callback when SVG is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<SVGSVGElement>) => {
        // Make sure the container of the button pad doesn't get selected
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    };

    // In customizing state add onClick callback to button pad SVG element
    // note: if overlaid on a video stream, let the parent video stream handle the click
    const selectProp =
        customizing && !overlay
            ? {
                  onClick: onSelect,
              }
            : {};

    return (
        <div className="button-pad">
            {/* {!overlay && isBrowser || isTablet? <h4 className="title">{id}</h4> : <></>} */}
            <svg
                ref={svgRef}
                viewBox={`0 0 ${SVG_RESOLUTION} ${props.aspectRatio ? SVG_RESOLUTION / props.aspectRatio : SVG_RESOLUTION}`}
                preserveAspectRatio="none"
                className={className("button-pads", {
                    customizing,
                    selected,
                    overlay,
                })}
                {...selectProp}
            >
                {paths.map(mapPaths)}
            </svg>
        </div>
    );
};

/** Properties for a single button on a button pad */
export type SingleButtonProps = {
    svgPath: string;
    funct: ButtonPadButton;
    sharedState: SharedState;
    iconPosition: { x: number; y: number };
};

const SingleButton = (props: SingleButtonProps) => {
    const functs: ButtonFunctions = buttonFunctionProvider.provideFunctions(
        props.funct,
    );
    const clickProps = props.sharedState.customizing
        ? {}
        : {
              onPointerDown: functs.onClick,
              onPointerUp: functs.onRelease,
              onPointerLeave: functs.onLeave,
          };
    const buttonState: ButtonState =
        props.sharedState.buttonStateMap?.get(props.funct) ||
        ButtonState.Inactive;
    const icon = getIcon(props.funct);
    const title = props.funct;
    const height = !isBrowser && !isTablet  ? 75 : 85;
    const width = !isBrowser && !isTablet  ? 75 : 85;
    const x = props.iconPosition.x - width / 2;
    const y = props.iconPosition.y - height / 2;
    return (
        <React.Fragment>
            <path d={props.svgPath} {...clickProps} className={buttonState}>
                <title>{title}</title>
            </path>
            <image
                x={x}
                y={y}
                height={height}
                width={width}
                href={icon}
                className={buttonState}
            />
            <p>{title}</p>
        </React.Fragment>
    );
};

/**
 * Provides the shape and functions for a button pad based on the identifier
 *
 * @param id the identifier of the button pad
 * @returns the shape of the button pad {@link ButtonPadShape} and a list of
 * {@link ButtonPadButton} where each element informs the function of
 * the corresponding button on the button pad
 */
function getShapeAndFunctionsFromId(
    id: ButtonPadId | ButtonPadIdMobile,
): [ButtonPadShape, ButtonPadButton[]] {
    let shape: ButtonPadShape;
    let functions: ButtonPadButton[];
    const B = ButtonPadButton;
    switch (id) {
        // case ButtonPadId.Drive:
        //     functions = [
        //         B.BaseForward,
        //         B.BaseRotateRight,
        //         B.BaseReverse,
        //         B.BaseRotateLeft
        //     ];
        //     shape = ButtonPadShape.Directional;
        //     break;
        case ButtonPadId.ManipRealsense:
            functions = [
                B.WristRotateIn,
                B.WristRotateOut,
                B.ArmExtend,
                B.ArmRetract,
                B.BaseForward,
                B.BaseReverse,
                B.ArmLift,
                B.ArmLower,
                B.GripperClose,
                B.GripperOpen,
            ];
            shape = ButtonPadShape.ManipRealsense;
            break;
        case ButtonPadId.GripperLift:
            functions = [
                B.ArmLift,
                B.ArmLower,
                B.WristRotateIn,
                B.WristRotateOut,
                B.GripperOpen,
                B.GripperClose,
            ];
            shape = ButtonPadShape.GripperLift;
            break;
        case ButtonPadId.DexWrist:
            functions = [
                B.WristPitchUp,
                B.WristPitchDown,
                B.WristRotateIn,
                B.WristRotateOut,
                B.WristRollLeft,
                B.WristRollRight,
                B.GripperOpen,
                B.GripperClose,
            ];
            shape = ButtonPadShape.DexWrist;
            break;
        case ButtonPadId.Base:
            functions = [
                B.BaseForward,
                B.BaseReverse,
                B.BaseRotateLeft,
                B.BaseRotateRight,
            ];
            shape = ButtonPadShape.SimpleButtonPad;
            break;
        case ButtonPadId.Camera:
            functions = [
                B.CameraTiltUp,
                B.CameraTiltDown,
                B.CameraPanLeft,
                B.CameraPanRight,
            ];
            shape = ButtonPadShape.SimpleButtonPad;
            break;
        // case ButtonPadId.Wrist:
        //     functions = [
        //         B.WristRollLeft,
        //         B.WristRollRight,
        //         B.WristPitchUp,
        //         B.WristPitchDown,
        //         B.WristRotateIn,
        //         B.WristRotateOut,
        //         B.GripperOpen,
        //         B.GripperClose
        //     ];
        //     shape = ButtonPadShape.StackedButtonPad;
        //     break;
        case ButtonPadId.Arm:
            functions = [B.ArmLift, B.ArmLower, B.ArmRetract, B.ArmExtend];
            shape = ButtonPadShape.SimpleButtonPad;
            break;
        case ButtonPadIdMobile.Arm:
            functions = [B.ArmLift, B.ArmLower, B.ArmRetract, B.ArmExtend];
            shape = ButtonPadShape.RowButtonPad;
            break;
        case ButtonPadIdMobile.Gripper:
            functions = [
                B.WristRotateIn,
                B.WristRotateOut,
                B.GripperOpen,
                B.GripperClose,
            ];
            shape = ButtonPadShape.RowButtonPad;
            break;
        case ButtonPadIdMobile.Drive:
            functions = [
                B.BaseForward,
                B.BaseReverse,
                B.BaseRotateLeft,
                B.BaseRotateRight,
            ];
            shape = ButtonPadShape.RowButtonPad;
            break;
        default:
            throw new Error(`unknow button pad id: ${id}`);
    }

    return [shape, functions];
}
