import React from "react";
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
    centerX: 50,
    centerY: 50,
    height: 10,
    width: 10
}

/** Default position for the box in the directional button pad for the
 * overhead display, such that the box is around the base of the robot
 */
export const OVERHEAD_DEFAULT_BOX_POSITION = {
    centerX: 55,
    centerY: 55,
    height: 12,
    width: 15
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

    const pathTop = `M 0 0 100 0 ${rgt} ${top} ${lft} ${top} Z`
    const pathRgt = `M 100 0 100 100 ${rgt} ${bot} ${rgt} ${top} Z`
    const pathBot = `M 0 100 100 100 ${rgt} ${bot} ${lft} ${bot} Z`
    const pathLft = `M 0 0 0 100 ${lft} ${bot} ${lft} ${top} Z`

    const paths = [pathTop, pathRgt, pathBot, pathLft]
    return (
        <svg width="100%" height="100%" viewBox="0 0 100 100" preserveAspectRatio="none">
            {paths.map((path, i) => 
                <path d={path} onClick={props.buttonsProps[i].onClick}/>
            )}
        </svg>
    );
}

/*************************************************************************
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
export const ExampleOverheadButtonPad = () => (
    <DirectionalButtonPad 
        boxPosition={OVERHEAD_DEFAULT_BOX_POSITION} 
        buttonsProps={EXAMPLE_OVERHEAD_BUTTONS}
    />
);