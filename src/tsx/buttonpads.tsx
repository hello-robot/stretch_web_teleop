import React from "react";
import { SVG_RESOLUTION, percent2Pixel, OVERHEAD_ROBOT_BASE } from "../util/svg";
import "../css/buttonpads.css"

/** Properties for a single button in a button pad */
export type ButtonProps = {
    /** The name of the button (acts as a tooltip) */
    label: string,
    onClick: () => void;
    onRelease?: () => void;
    icon?: object  // TODO: figure out how to put an icon on a button
}

/** Properties shared across all button pad const
 * ents */
interface ButtonPadProps {
    /** List of properties for the buttons in this button pad
     * @length should be equal to number of buttons on the pad
     */
    buttonsProps: ButtonProps[]
}

/** Type to represent the position and size of a box */
type BoxPosition = {
    centerX: number,
    centerY: number,
    height: number,
    width: number
}

/** Properties for the directional button pad */
interface DirectionalButtonPadProps extends ButtonPadProps {
    /** Position and size of the box in the middle of the pad */
    boxPosition?: BoxPosition;
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

/**Creates the SVG path for a rectangle
 * @param x left edge location
 * @param y top edge location
 * @param width the width
 * @param height the height
*/
function rect(x: number, y: number, width: number, height: number) {
    return `M ${x} ${y} ${x+width} ${y} ${x+width} ${y+height} ${x} ${y+height} Z`;
}

/**Creates an SVG containing all the paths with the corresponding button properties.
 * @param paths the string description of the path
 * @param buttonsProps the properties of each button
 * @note paths and buttonsProps must be the same length, path object corresponds 
 * to ButtonProps object at same index in array
 */
function createButtonPad(paths: string[], buttonsProps: ButtonProps[]) {
    if (paths.length > buttonsProps.length) {
        console.error(`paths length: ${paths.length}, buttonsProps length: ${buttonsProps.length}`);
        return <b style={{color: "red"}}>ERROR</b>;
    }
    return (
        <svg 
            viewBox={`0 0 ${SVG_RESOLUTION} ${SVG_RESOLUTION}`} 
            preserveAspectRatio="none"
            className="button-pads"
        >
            {paths.map((path, i) => 
                <path d={path} onClick={buttonsProps[i].onClick}>
                    <title>{buttonsProps[i].label}</title>
                </path>
            )}
        </svg>
    );
}

/** Directional button pad made up of four trapazoids around a box in the 
 * center of the button pad.
 * @note buttonProps are interpreted in this order: top, right, bottom, left
 */
export const DirectionalButtonPad = (props: DirectionalButtonPadProps) => {
    const boxPosition: BoxPosition = props.boxPosition || DEFAULT_POSITION;
    const {centerX, centerY, height, width} = boxPosition;
    const top = centerY - height / 2
    const bot = centerY + height / 2
    const lft = centerX - width / 2
    const rgt = centerX + width / 2

    const pathTop = `M 0 0 ${SVG_RESOLUTION} 0 ${rgt} ${top} ${lft} ${top} Z`
    const pathRgt = `M ${SVG_RESOLUTION} 0 ${SVG_RESOLUTION} ${SVG_RESOLUTION} ${rgt} ${bot} ${rgt} ${top} Z`
    const pathBot = `M 0 ${SVG_RESOLUTION} ${SVG_RESOLUTION} ${SVG_RESOLUTION} ${rgt} ${bot} ${lft} ${bot} Z`
    const pathLft = `M 0 0 0 ${SVG_RESOLUTION} ${lft} ${bot} ${lft} ${top} Z`

    const paths = [pathTop, pathRgt, pathBot, pathLft]
    return createButtonPad(paths, props.buttonsProps);
}

/**Button pad for the realsense camera in manipulation mode from the 
 * default study interface. 
 * @todo rename this component to be more universal and convey its purpose,
 *       maybe "10 button interface"
 */
export const RealsenseMaipButtonPad = (props: ButtonPadProps) => {
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
        `M 0 ${height} ${SVG_RESOLUTION} ${height} ${centerRight} ${height*2} ${centerLeft} ${height*2} Z`,
        `M 0 ${height*5} ${SVG_RESOLUTION} ${height*5} ${centerRight},${height*4} ${centerLeft},${height*4} Z`,
        `M 0 ${height} 0 ${height*5} ${centerLeft},${height*4} ${centerLeft},${height*2} Z`,
        `M ${SVG_RESOLUTION} ${height} ${SVG_RESOLUTION} ${height*5} ${centerRight},${height*4} ${centerRight},${height*2} Z`,
        // // Center two rectangle buttons
        rect(centerLeft, height*2, centerWidth, height),
        rect(centerLeft, height*3, centerWidth, height),
        // // Bottom two buttons
        rect(0, height*5, center, height),
        rect(center, height*5, center, height)
    ]
    return createButtonPad(paths, props.buttonsProps);
}

/**Button pad for gripper camera in manipulation mode for the default study
 * interface.
 */
export const GripperButtonPad = (props: ButtonPadProps) => {
    /**Number of button layers from top to bottom in the display*/
    const numLayers = 5;
    /**How tall each layer of buttons should be.*/
    const margin = SVG_RESOLUTION / numLayers;
    const paths = [
        rect(0, 0, SVG_RESOLUTION, margin),  // top
        rect(0, SVG_RESOLUTION-margin, SVG_RESOLUTION, margin),  // bottom
        rect(0, margin, margin, margin*3),  // left
        rect(SVG_RESOLUTION-margin, margin, margin, margin*3), // right
        rect(margin, margin, margin*3, margin*3),  // gripper open
        rect(margin*2, margin*2, margin, margin)  // gripper close
    ]
    return createButtonPad(paths, props.buttonsProps);
}

/*************************************************************************
 * ***********************************************************************
 * This is example code about how to create a button pad. this logic
 * should be moved somewhere central to make it easier to dynamically 
 * create button pads.
 */

const EXAMPLE_BASE_TRANSLATE_FORWARD: ButtonProps = {
    label: "Forward",
    onClick: () => console.log("move forward")
}

const EXAMPLE_BASE_TRANSLATE_BACKWARD: ButtonProps = {
    label: "Backward",
    onClick: () => console.log("move backward")
}

const EXAMPLE_BASE_ROTATE_RIGHT: ButtonProps = {
    label: "Rotate right",
    onClick: () => console.log("rotate right")
}

const EXAMPLE_BASE_ROTATE_LEFT: ButtonProps = {
    label: "Rotate left",
    onClick: () => console.log("rotate left")
}

/** Example of defining the buttons in a button pad */
const EXAMPLE_OVERHEAD_BUTTONS: ButtonProps[] = [
    EXAMPLE_BASE_TRANSLATE_FORWARD,
    EXAMPLE_BASE_ROTATE_RIGHT,
    EXAMPLE_BASE_TRANSLATE_BACKWARD,
    EXAMPLE_BASE_ROTATE_LEFT
]

/** Example of creating a button pad */
const ExampleOverheadButtonPad = () => (
    <DirectionalButtonPad 
        boxPosition={OVERHEAD_ROBOT_BASE} 
        buttonsProps={EXAMPLE_OVERHEAD_BUTTONS}
    />
);

/** Example of creating a button pad */
const ExampleRealsenseMaipButtonPad = () => {
    const buttons: string[] = [
        "gripper in",
        "gripper out",
        "extend arm",
        "retract arm",
        "base forward",
        "base backward",
        "lift arm",
        "lower arm",
        "close gripper",
        "open gripper" 
    ]
    const mapFunc = (text: string) => (
        {label: text, onClick: (() => (console.log(text)))}
    );
    return (
        <RealsenseMaipButtonPad 
            buttonsProps={buttons.map(mapFunc)}
        />
    );
};

/** Example of creating a button pad */
const ExampleGripperButtonPad = () => {
    const buttons: string[] = [
        "lift arm",
        "lower arm",
        "gripper in",
        "gripper out",
        "open gripper",
        "close gripper"
    ]
    const mapFunc = (text: string) => (
        {label: text, onClick: (() => (console.log(text)))}
    );
    return (
        <GripperButtonPad 
            buttonsProps={buttons.map(mapFunc)}
        />
    );
};

export const ExampleButtonPads = [
    <ExampleOverheadButtonPad />,
    <ExampleRealsenseMaipButtonPad />,
    <ExampleGripperButtonPad />
]