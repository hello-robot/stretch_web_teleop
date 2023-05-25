import React from "react";
import "operator/css/buttonpads.css"
import { CustomizableComponentProps, SharedState } from "./customizablecomponent";
import { ButtonPadDef, ButtonPadId } from "../utils/componentdefinitions";
import { className } from "shared/util";
import { buttonFunctionProvider } from "operator/tsx/index";
import { getIcon, getDirectionalPaths, getRealsenseManipPaths, getGripperPaths, SVG_RESOLUTION } from "../utils/svg";
import { ButtonFunctions, ButtonPadButton, ButtonState } from "../functionprovider/buttonpads";

/** Possible layouts for the button pad (i.e. the shape and arrangement of the 
 * buttons)
 */
export enum ButtonPadShape {
    Directional,
    Realsense,
    Gripper
}

/** Properties for {@link ButtonPad} */
type ButtonPadProps = CustomizableComponentProps & {
    /* If the button pad is overlaid on a video stream */
    overlay?: boolean;
}

/**
 * A set of buttons which can be overlaid as a child of a video stream or 
 * lonestanding.
 * 
 * @param props {@link ButtonPadProps}
 */
export const ButtonPad = (props: ButtonPadProps) => {
    /** Reference to the SVG which makes up the button pad */
    const svgRef = React.useRef<SVGSVGElement>(null);
    /** List of path shapes for each button on the button pad */
    const definition = props.definition as ButtonPadDef;
    const id: ButtonPadId = definition.id;
    if (!id) throw Error("Undefined button pad ID at path " + props.path);
    const [shape, functions] = getShapeAndFunctionsFromId(definition.id);
    const [paths, iconPositions] = getPathsFromShape(shape);

    // Paths and functions should be the same length
    if (paths.length !== functions.length) {
        throw Error(`paths length: ${paths.length}, functions length: ${functions.length}`);
    }

    const { customizing } = props.sharedState;
    const { overlay } = props;
    const selected = props.path === props.sharedState.activePath;

    /** Uses the paths and buttonsProps to create the buttons */
    function mapPaths(svgPath: string, i: number) {
        const buttonProps = {
            iconPosition: iconPositions[i],
            svgPath,
            funct: functions[i],
            sharedState: props.sharedState
        }
        // Buttons will not function during customization mode
        return <SingleButton {...buttonProps} key={i} />;
    }

    /** Callback when SVG is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<SVGSVGElement>) => {
        // Make sure the container of the button pad doesn't get selected
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);;
    }

    // In customizing state add onClick callback to button pad SVG element
    // note: if overlaid on a video stream, let the parent video stream handle the click
    const selectProp = customizing && !overlay ? {
        "onClick": onSelect
    } : {};

    return (
        <>
            <svg
                ref={svgRef}
                viewBox={`0 0 ${SVG_RESOLUTION} ${SVG_RESOLUTION}`}
                preserveAspectRatio="none"
                className={className("button-pads", { customizing, selected, overlay })}
                {...selectProp}
            >
                {paths.map(mapPaths)}
            </svg>
        </>
    );
}


/** Properties for a single button on a button pad */
export type SingleButtonProps = {
    svgPath: string,
    funct: ButtonPadButton,
    sharedState: SharedState,
    iconPosition: { x: number, y: number }
}

const SingleButton = (props: SingleButtonProps) => {
    const functs: ButtonFunctions = buttonFunctionProvider.provideFunctions(props.funct);
    const clickProps = props.sharedState.customizing ? {} : {
        onMouseDown: functs.onClick,
        onMouseUp: functs.onRelease,
        onMouseLeave: functs.onLeave
    }
    const buttonState: ButtonState = props.sharedState.buttonStateMap?.get(props.funct) || ButtonState.Inactive;
    const icon = getIcon(props.funct);
    const title = props.funct;
    const height = 50;
    const width = 50;
    const x = props.iconPosition.x - width / 2;
    const y = props.iconPosition.y - height / 2;
    return (
        <React.Fragment >
            <path d={props.svgPath} {...clickProps} className={buttonState}>
                <title>{title}</title>
            </path>
            <image x={x} y={y} height={height} width={width} href={icon} className={buttonState} />
        </React.Fragment>
    )
}

/**
 * Provides the shape and fuctions for a button pad based on the identifier
 * 
 * @param id the identifier of the button pad
 * @returns the shape of the button pad {@link ButtonPadShape} and a list of 
 * {@link ButtonPadButton} where each element informs the function of
 * the corresponding button on the button pad
 */
function getShapeAndFunctionsFromId(id: ButtonPadId): [ButtonPadShape, ButtonPadButton[]] {
    let shape: ButtonPadShape;
    let functions: ButtonPadButton[];
    const BF = ButtonPadButton;
    switch (id) {
        case ButtonPadId.overhead:
            functions = [
                BF.BaseForward,
                BF.BaseRotateRight,
                BF.BaseReverse,
                BF.BaseRotateLeft
            ];
            shape = ButtonPadShape.Directional;
            break;
        case ButtonPadId.realsense:
            functions = [
                BF.WristRotateIn,
                BF.WristRotateOut,
                BF.ArmExtend,
                BF.ArmRetract,
                BF.BaseForward,
                BF.BaseReverse,
                BF.ArmLift,
                BF.ArmLower,
                BF.GripperClose,
                BF.GripperOpen
            ]
            shape = ButtonPadShape.Realsense;
            break;
        case ButtonPadId.gripper:
            functions = [
                BF.ArmLift,
                BF.ArmLower,
                BF.WristRotateIn,
                BF.WristRotateOut,
                BF.GripperOpen,
                BF.GripperClose,
            ]
            shape = ButtonPadShape.Gripper;
            break;
        default:
            throw new Error(`unknow button pad id: ${id}`);
    }

    return [shape, functions];
}

/******************************************************************************/
/* Logic to draw the paths for the buttons on each button pad                 */
/******************************************************************************/

/**
 * Gets a list of path string descriptions for each button based on the {@link ButtonPadShape}
 * 
 * @param shape {@link ButtonPadShape} enum representing the shape of the button pad
 * @returns a list of strings where each string is a path description for the shape of a single button
 */
function getPathsFromShape(shape: ButtonPadShape): [string[], { x: number, y: number }[]] {
    switch (shape) {
        case (ButtonPadShape.Directional):
            return getDirectionalPaths();
        case (ButtonPadShape.Realsense):
            return getRealsenseManipPaths();
        case (ButtonPadShape.Gripper):
            return getGripperPaths();
    }
}