import React from "react";
import "operator/css/buttonpads.css"
import { CustomizableComponentProps } from "./customizablecomponent";
import { ButtonPadDef, ButtonPadId, VideoStreamDef, VideoStreamId } from "../utils/componentdefinitions";
import { className } from "shared/util";
import { SVG_RESOLUTION, percent2Pixel, OVERHEAD_ROBOT_BASE, rect } from "shared/svg";
import { buttonFunctionProvider } from "operator/tsx/index";

/** All the possible button functions */
export enum ButtonPadFunction {
    BaseForward,
    BaseReverse,
    BaseRotateRight,
    BaseRotateLeft,
    ArmLift,
    ArmLower,
    ArmExtend,
    ArmRetract,
    GripperOpen,
    GripperClose,
    WristRotateIn,
    WristRotateOut,
    PredictiveDisplay
}

/** Possible layouts for the button pad (i.e. the shape and arrangement of the 
 * buttons)
 */
export enum ButtonPadShape {
    Directional,
    Realsense,
    Gripper
}

/** Which functions called when the user interacts with buttons. */
export type ButtonFunctions = {
    onClick: () => void,
    onRelease?: () => void,
    onLeave?: () => void
}

/** Properties for a single button on a button pad */
export type ButtonProps = ButtonFunctions & {
    /** The name of the button (acts as a tooltip) */
    label: string,
    icon?: object  // TODO: figure out how to put an icon on a button
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
    const buttonsProps = functions.map((funct: ButtonPadFunction) => {
        return {
            ...buttonFunctionProvider.provideFunctions(funct),
            label: "" + funct
        } as ButtonProps;
    });
    const paths: string[] = getPathsFromShape(shape);

    // Paths and functions should be the same length
    if (paths.length !== functions.length) {
        throw Error(`paths length: ${paths.length}, functions length: ${functions.length}`);
    }

    const { customizing } = props.sharedState;
    const { overlay } = props;
    const selected = props.path === props.sharedState.activePath;

    /** Uses the paths and buttonsProps to create the buttons */
    const mapPaths = (path: string, i: number) => {
        const buttonProps = buttonsProps[i]
        // Buttons will not function during customization mode
        const clickProps = customizing ? {} : {
            onMouseDown: buttonProps.onClick,
            onMouseUp: buttonProps.onRelease,
            onMouseLeave: buttonProps.onLeave
        }
        const title = buttonProps.label;
        return (
            <path key={i} d={path} {...clickProps}>
                <title>{title}</title>
            </path>
        )
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

/**
 * Provides the shape and fuctions for a button pad based on the identifier
 * 
 * @param id the identifier of the button pad
 * @returns the shape of the button pad {@link ButtonPadShape} and a list of 
 * {@link ButtonPadFunction} where each element informs the function of
 * the corresponding button on the button pad
 */
function getShapeAndFunctionsFromId(id: ButtonPadId): [ButtonPadShape, ButtonPadFunction[]] {
    let shape: ButtonPadShape;
    let functions: ButtonPadFunction[];
    const BF = ButtonPadFunction;
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
function getPathsFromShape(shape: ButtonPadShape): string[] {
    switch (shape) {
        case (ButtonPadShape.Directional):
            return getDirectionalPaths();
        case (ButtonPadShape.Realsense):
            return getRealsenseManipPaths();
        case (ButtonPadShape.Gripper):
            return getGripperPaths();
    }
}

/** Represents the position and size of a box */
type BoxPosition = {
    centerX: number,
    centerY: number,
    height: number,
    width: number
}

/** Default box position with the box centered and a height and width 
 * of 10%
 */
const DEFAULT_POSITION = {
    centerX: percent2Pixel(50),
    centerY: percent2Pixel(50),
    height: percent2Pixel(10),
    width: percent2Pixel(10)
}

/**
 * Directional button pad made up of four trapazoids around a box in the 
 * center of the button pad.
 * 
 * @param onRobot if the square should be around the robot, centered if false
 */
function getDirectionalPaths(onRobot: boolean = true) {
    const boxPosition: BoxPosition = onRobot ? OVERHEAD_ROBOT_BASE : DEFAULT_POSITION;
    const { centerX, centerY, height, width } = boxPosition;
    const top = centerY - height / 2
    const bot = centerY + height / 2
    const lft = centerX - width / 2
    const rgt = centerX + width / 2

    const pathTop = `M 0 0 ${SVG_RESOLUTION} 0 ${rgt} ${top} ${lft} ${top} Z`
    const pathRgt = `M ${SVG_RESOLUTION} 0 ${SVG_RESOLUTION} ${SVG_RESOLUTION} 
                        ${rgt} ${bot} ${rgt} ${top} Z`
    const pathBot = `M 0 ${SVG_RESOLUTION} ${SVG_RESOLUTION} ${SVG_RESOLUTION} 
                        ${rgt} ${bot} ${lft} ${bot} Z`
    const pathLft = `M 0 0 0 ${SVG_RESOLUTION} ${lft} ${bot} ${lft} ${top} Z`

    const paths = [pathTop, pathRgt, pathBot, pathLft]
    return paths;
}

/**
 * Ordered: top left, top right, then top right, bottom, left trapezoids, then 
 * top and bottom center buttons, and finally bottom left and bottom right.
 */
function getRealsenseManipPaths() {
    /**Number of button layers from top to bottom in the display*/
    const numVerticalLayers = 6;
    /**How tall each layer of buttons should be.*/
    const height = SVG_RESOLUTION / numVerticalLayers;
    const centerWidth = percent2Pixel(30);
    const centerLeft = (SVG_RESOLUTION - centerWidth) / 2;
    const centerRight = centerLeft + centerWidth;
    const center = percent2Pixel(50);
    const paths = [
        // Top two buttons
        rect(0, 0, center, height),
        rect(center, 0, center, height),
        // Center directional trapezoid buttons
        `M 0 ${height} ${SVG_RESOLUTION} ${height} ${centerRight} ${height * 2} 
            ${centerLeft} ${height * 2} Z`,
        `M 0 ${height * 5} ${SVG_RESOLUTION} ${height * 5} 
            ${centerRight},${height * 4} ${centerLeft},${height * 4} Z`,
        `M 0 ${height} 0 ${height * 5} ${centerLeft},${height * 4} 
            ${centerLeft},${height * 2} Z`,
        `M ${SVG_RESOLUTION} ${height} ${SVG_RESOLUTION} ${height * 5} 
            ${centerRight},${height * 4} ${centerRight},${height * 2} Z`,
        // // Center two rectangle buttons
        rect(centerLeft, height * 2, centerWidth, height),
        rect(centerLeft, height * 3, centerWidth, height),
        // // Bottom two buttons
        rect(0, height * 5, center, height),
        rect(center, height * 5, center, height)
    ]
    return paths;
}

/**
 * Ordered top, botton, left, right, larger center, smaller center
 */
function getGripperPaths() {
    /**Number of button layers from top to bottom in the display*/
    const numLayers = 5;
    /**How tall each layer of buttons should be.*/
    const margin = SVG_RESOLUTION / numLayers;
    const paths = [
        rect(0, 0, SVG_RESOLUTION, margin),  // top
        rect(0, SVG_RESOLUTION - margin, SVG_RESOLUTION, margin),  // bottom
        rect(0, margin, margin, margin * 3),  // left
        rect(SVG_RESOLUTION - margin, margin, margin, margin * 3), // right
        rect(margin, margin, margin * 3, margin * 3),  // gripper open
        rect(margin * 2, margin * 2, margin, margin)  // gripper close
    ]
    return paths;
}