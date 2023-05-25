
import armDown from "operator/icons/Arm_Down.svg"
import armExtend from "operator/icons/Arm_Extend.svg"
import armRetract from "operator/icons/Arm_Retract.svg"
import armUp from "operator/icons/Arm_Up.svg"
import driveLeft from "operator/icons/Drive_Left.svg"
import driveRight from "operator/icons/Drive_Right.svg"
import gripClose from "operator/icons/Grip_Close.svg"
import gripLeft from "operator/icons/Grip_Left.svg"
import gripOpen from "operator/icons/Grip_Open.svg"
import gripRight from "operator/icons/Grip_Right.svg"
import driveForward from "operator/icons/Drive_FWD.svg"
import driveReverse from "operator/icons/Drive_RVS.svg"
import { ButtonPadButton } from "../functionprovider/buttonpads"


/**Resolution of SVG components */
export const SVG_RESOLUTION = 500;

/**
 * Takes a percentage value and returns the real pixel location on 
 * the SVG
 * @param percentage value between 0 and 100
 * @returns the pixel location
 * @example 0 -> 0, 50 -> resolution/2, 100 -> resolution
 */
export function percent2Pixel(percentage: number) {
    return SVG_RESOLUTION / 100 * percentage;
}

/**
 * Position and dimensions of the robot base from the overhead camera view
 */
export const OVERHEAD_ROBOT_BASE = {
    centerX: percent2Pixel(50),
    centerY: percent2Pixel(70),
    height: percent2Pixel(10),
    width: percent2Pixel(10)
}

/**Creates the SVG path for a rectangle
 * @param x left edge location
 * @param y top edge location
 * @param width the width
 * @param height the height
*/
export function rect(x: number, y: number, width: number, height: number) {
    return `M ${x} ${y} ${x + width} ${y} ${x + width} ${y + height} 
                ${x} ${y + height} Z`;
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
export function getDirectionalPaths(onRobot: boolean = true): [string[], { x: number, y: number }[]] {
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
    const iconPositions = [
        { x: centerX, y: top / 2 },
        { x: (SVG_RESOLUTION + rgt) / 2, y: centerY },
        { x: centerX, y: (SVG_RESOLUTION + bot) / 2 },
        { x: (lft) / 2, y: centerY}
    ]
    return [paths, iconPositions];
}

/**
 * Ordered: top left, top right, then top, bottom, left, right trapezoids, then 
 * top and bottom center buttons, and finally bottom left and bottom right.
 */
export function getRealsenseManipPaths(): [string[], { x: number, y: number }[]] {
    /**Number of button layers from top to bottom in the display*/
    const numVerticalLayers = 6;
    /**How tall each layer of buttons should be.*/
    const height = SVG_RESOLUTION / numVerticalLayers;
    const centerWidth = percent2Pixel(30);
    const centerLeft = (SVG_RESOLUTION - centerWidth) / 2;
    const centerRight = centerLeft + centerWidth;
    const center = percent2Pixel(50);
    const paths = [
        // Top two buttons: left, right
        rect(0, 0, center, height),
        rect(center, 0, center, height),
        // Center directional trapezoid buttons: top, bottom, left, right
        `M 0 ${height} ${SVG_RESOLUTION} ${height} ${centerRight} ${height * 2} 
            ${centerLeft} ${height * 2} Z`,
        `M 0 ${height * 5} ${SVG_RESOLUTION} ${height * 5} 
            ${centerRight},${height * 4} ${centerLeft},${height * 4} Z`,
        `M 0 ${height} 0 ${height * 5} ${centerLeft},${height * 4} 
            ${centerLeft},${height * 2} Z`,
        `M ${SVG_RESOLUTION} ${height} ${SVG_RESOLUTION} ${height * 5} 
            ${centerRight},${height * 4} ${centerRight},${height * 2} Z`,
        // // Center two rectangle buttons: top, bottom
        rect(centerLeft, height * 2, centerWidth, height),
        rect(centerLeft, height * 3, centerWidth, height),
        // // Bottom two buttons: left, right
        rect(0, height * 5, center, height),
        rect(center, height * 5, center, height)
    ]
    const iconPositions = [
        // Top two
        { x: center / 2, y: height / 2 },
        { x: (SVG_RESOLUTION + center) / 2, y: height / 2 },
        // Center directional trapezoid buttons
        { x: SVG_RESOLUTION / 2, y: height * 3 / 2 },
        { x: SVG_RESOLUTION / 2, y: height * 9 / 2 },
        { x: centerLeft / 2, y: height * 6 / 2 },
        { x: (SVG_RESOLUTION + centerRight) / 2, y: height * 6 / 2 },
        // Center two rectangle buttons
        { x: SVG_RESOLUTION / 2, y: height * 5 / 2 },
        { x: SVG_RESOLUTION / 2, y: height * 7 / 2 },
        // Bottom two buttons
        { x: center / 2, y: height * 11 / 2 },
        { x: (SVG_RESOLUTION + center) / 2, y: height * 11 / 2 }
    ]
    return [paths, iconPositions];
}

/**
 * Ordered top, botton, left, right, larger center, smaller center
 */
export function getGripperPaths(): [string[], { x: number, y: number }[]] {
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
    const iconPositions = [
        { x: SVG_RESOLUTION / 2, y: margin / 2 },  // top
        { x: SVG_RESOLUTION / 2, y: (2 * SVG_RESOLUTION - margin) / 2 },  // bottom
        { x: margin / 2, y: SVG_RESOLUTION / 2 },  // left
        { x: (2 * SVG_RESOLUTION - margin) / 2, y: SVG_RESOLUTION / 2 },  // right
        { x: margin * 7 / 2, y: SVG_RESOLUTION / 2 },  // gripper open
        { x: SVG_RESOLUTION / 2, y: SVG_RESOLUTION / 2 },  // gripper close
    ]
    return [paths, iconPositions];
}

/**
 * Gets the icon corresponding to a button in a button pad.
 * 
 * @param buttonPadButton 
 * @returns icon source
 */
export function getIcon(buttonPadButton: ButtonPadButton) {
    switch (buttonPadButton) {
        case (ButtonPadButton.BaseForward):
            return driveForward;
        case (ButtonPadButton.BaseReverse):
            return driveReverse;
        case (ButtonPadButton.BaseRotateRight):
            return driveRight;
        case (ButtonPadButton.BaseRotateLeft):
            return driveLeft;
        case (ButtonPadButton.ArmLift):
            return armUp;
        case (ButtonPadButton.ArmLower):
            return armDown;
        case (ButtonPadButton.ArmExtend):
            return armExtend;
        case (ButtonPadButton.ArmRetract):
            return armRetract;
        case (ButtonPadButton.GripperOpen):
            return gripOpen;
        case (ButtonPadButton.GripperClose):
            return gripClose;
        case (ButtonPadButton.WristRotateIn):
            return gripLeft;
        case (ButtonPadButton.WristRotateOut):
            return gripRight
        default:
            throw Error(`unknown button pad button\t${buttonPadButton}`);
    }
}