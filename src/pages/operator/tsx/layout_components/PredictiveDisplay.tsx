import React from "react";
import { className, navigationProps } from "shared/util";
import { CustomizableComponentProps } from "./CustomizableComponent";
import { predicitiveDisplayFunctionProvider } from "operator/tsx/index";
import {
    SVG_RESOLUTION,
    percent2Pixel,
    OVERHEAD_ROBOT_BASE as BASE,
} from "../utils/svg";
import "operator/css/PredictiveDisplay.css";

/**
 * Scales height values to fit in the navigation camera
 * @param y the number to scale
 * @returns scaled number
 */
function scaleToNavAspectRatio(y: number) {
    return (y / navigationProps.width) * navigationProps.height;
}

/**Arguments for drawing the dashed line in the center of the path */
const strokeDasharray = "4 10";
/**Height of the predictive display SVG */
const resolution_height = scaleToNavAspectRatio(SVG_RESOLUTION);
/**Pixel location of the front of the robot */
const baseFront = scaleToNavAspectRatio(BASE.centerY - BASE.height / 2);
/**Pixel location of the back of the robot */
const baseBack = scaleToNavAspectRatio(BASE.centerY + BASE.height / 2);
/**Y pixel position of the center of the base */
const baseCenterY = scaleToNavAspectRatio(BASE.centerY);
/**Left side of the robot */
const baseLeft = BASE.centerX - BASE.width / 2;
/**Right side of the robot */
const baseRight = BASE.centerX + BASE.width / 2;
/**Radius around the base of the rotation arrows */
const rotateArcRadius = percent2Pixel(10);

/**If the cursor is at this fraction of maxDistance, set the speed to max. */
const distanceRatioForMaxSpeed = 0.9;

/** Functions required for predictive display */
export type PredictiveDisplayFunctions = {
    /** Callback function when mouse is clicked in predicitive display area */
    onPointerDown: (length: number, angle: number) => void;
    /** Callback function when cursor is moved in predictive display area */
    onMove?: (length: number, angle: number) => void;
    /** Callback function for release */
    onRelease?: () => void;
    /** Callback function for leaving predictive display area */
    onLeave?: () => void;
};

/**
 * Example trajectory to display while in customizing mode so the user can see
 * the predictive display overlay on the overhead camera.
 */
const customizingTrajectory = drawForwardTraj(106, 161)[2];

/**
 * Overlay for overhead video stream where a curved path follows the cursor,
 * and clicking translates and/or rotates the robot base.
 *
 * @param props {@link CustomizableComponentProps}
 */
export const PredictiveDisplay = (props: CustomizableComponentProps) => {
    const svgRef = React.useRef<SVGSVGElement>(null);
    const { customizing } = props.sharedState;
    const [trajectory, setTrajectory] = React.useState<JSX.Element | undefined>(
        undefined,
    );
    const [moving, setMoving] = React.useState<boolean>(false);
    const functions =
        predicitiveDisplayFunctionProvider.provideFunctions(setMoving);
    const length = React.useRef<number>(0);
    const angle = React.useRef<number>(0);
    const holding = React.useRef<boolean>(false);

    function handleLeave() {
        setTrajectory(undefined);
        if (functions.onLeave) {
            functions.onLeave();
        }
    }

    function handleClick() {
        holding.current = true;
        if (functions.onPointerDown) {
            functions.onPointerDown(length.current, angle.current);
        }
    }

    function handleRelease() {
        holding.current = false;
        if (functions.onRelease) {
            functions.onRelease();
        }
    }

    /** Rerenders the trajectory based on the cursor location */
    function handleMove(event: React.MouseEvent<SVGSVGElement>) {
        const { clientX, clientY } = event;
        const svg = svgRef.current;
        if (!svg) return;

        // Get x and y in terms of the SVG element
        const rect = svg.getBoundingClientRect();
        const x = ((clientX - rect.left) / rect.width) * SVG_RESOLUTION;
        const pixelY = (clientY - rect.top) / rect.height;
        const y = scaleToNavAspectRatio(pixelY * SVG_RESOLUTION);
        const ret = drawTrajectory(x, y);

        length.current = ret[0];
        angle.current = ret[1];
        setTrajectory(ret[2]);

        if (holding && functions.onMove) {
            functions.onMove(length.current, angle.current);
        }
    }

    // If customizing, disable all user interaction
    const controlProps = customizing
        ? {}
        : {
              onMouseMove: handleMove,
              onMouseLeave: handleLeave,
              onMouseDown: handleClick,
              onMouseUp: handleRelease,
          };

    return (
        <svg
            viewBox={`0 0 ${SVG_RESOLUTION} ${resolution_height}`}
            preserveAspectRatio="none"
            ref={svgRef}
            className={className("predictive-display", { customizing, moving })}
            {...controlProps}
        >
            {customizing ? customizingTrajectory : trajectory}
        </svg>
    );
};

/**
 * Creates a trajectory based on the cursor location
 *
 * @param x horizontal position of the cursor
 * @param y vertical position of the cursor
 * @returns the linear distance, the angle, and the trajectory element
 */
function drawTrajectory(x: number, y: number): [number, number, JSX.Element] {
    let ret: [number, number, JSX.Element];
    if (y < baseFront) {
        ret = drawForwardTraj(x, y);
    } else if (y < baseBack) {
        // Next to base, draw rotate trajectory
        ret = drawRotate(x < BASE.centerX);
    } else {
        // Move backward
        ret = drawBackward(y);
    }
    return ret;
}

/**
 * Draws an arc from the base to the cursor, such that the arc is normal
 * to the base.
 *
 * @param x horizontal position of the cursor
 * @param y vertical position of the cursor
 * @returns the linear distance, the angle, and the trajectory element
 */
function drawForwardTraj(x: number, y: number): [number, number, JSX.Element] {
    const baseX = BASE.centerX;
    const baseY = baseFront;
    const dx = baseX - x;
    const dy = baseY - y;
    const heading = Math.atan2(-dx, dy);
    const sweepFlag = dx < 0;

    const distance = Math.sqrt(dx ** 2.0 + dy ** 2.0); // length from base to cursor
    const radius = distance / (2 * Math.sin(heading)); // radius of the center curve
    const centerPath = makeArc(
        BASE.centerX,
        baseFront,
        radius,
        sweepFlag,
        x,
        y,
    );

    const leftEndX = x - (BASE.width / 2) * Math.cos(2 * heading);
    const leftEndY = y - (BASE.width / 2) * Math.sin(2 * heading);
    const leftRadius = radius + BASE.width / 2;
    const leftPath = makeArc(
        baseLeft,
        baseFront,
        leftRadius,
        sweepFlag,
        leftEndX,
        leftEndY,
    );

    const rightEndX = x + (BASE.width / 2) * Math.cos(2 * heading);
    const rightEndY = y + (BASE.width / 2) * Math.sin(2 * heading);
    const rightRadius = radius - BASE.width / 2;
    const rightPath = makeArc(
        baseRight,
        baseFront,
        rightRadius,
        sweepFlag,
        rightEndX,
        rightEndY,
    );

    const trajectory = (
        <>
            <path d={centerPath} style={{ strokeDasharray: strokeDasharray }} />
            <path d={leftPath} />
            <path d={rightPath} />
        </>
    );

    // Compute the max distance in the direction of the cursor.
    // Note that this is not quite accurate. The most accurate way
    // to do this would be to compute the length along the center arc,
    // and then compute the length along the center arc if we were to
    // extend it all the way to the edge of the image, and then normalize.
    // Instead, we compute the length along the straight-line path from
    // the base to the cursor, and then compute the length along the straight-line
    // path from the base to the cursor if we were to extend it all the way to the edge
    // of the image, and then normalize. However, for our purposes straight-line length
    // can serve as a heuristic for arc length.
    const headingTopLeft = Math.atan2(-baseX, baseY);
    const headingTopRight = Math.atan2(SVG_RESOLUTION - baseX, baseY);
    let maxPointX: number;
    let maxPointY: number;
    if (heading < headingTopLeft) {
        // The max point is where the line from base to cursor intersects the left edge of the image
        maxPointX = 0;
        maxPointY = baseY - (baseX * dy) / dx;
    } else if (heading > headingTopRight) {
        // The max point is where the line from base to cursor intersects the right edge of the image
        maxPointX = SVG_RESOLUTION;
        maxPointY = baseY + ((SVG_RESOLUTION - baseX) * dy) / dx;
    } else {
        // The max point is where the line from base to cursor intersects the top edge of the image
        maxPointY = 0;
        maxPointX = baseX - (baseY * dx) / dy;
    }
    const maxDistance = Math.sqrt(
        (baseX - maxPointX) ** 2.0 + (baseY - maxPointY) ** 2.0,
    );

    // Normalize the distance
    const normalizedDistance = Math.min(
        distance / (maxDistance * distanceRatioForMaxSpeed),
        1.0,
    );
    return [normalizedDistance, -1 * heading, trajectory];
}

/**
 * Creates the SVG path elements for circular arrows around the base.
 *
 * @param rotateLeft if true draws a path counter-clockwise, otherwise clockwise
 * @returns SVG path string description of the arrows
 */
function makeArrowPath(rotateLeft: boolean) {
    const arrowLength = percent2Pixel(2.5);
    const top = baseCenterY - rotateArcRadius;
    const bottom = baseCenterY + rotateArcRadius;
    const left = BASE.centerX - rotateArcRadius;
    const right = BASE.centerX + rotateArcRadius;
    const arrowDx = rotateLeft ? arrowLength : -arrowLength;

    let arrows = makeArc(
        rotateLeft ? right : left,
        baseCenterY,
        rotateArcRadius,
        !rotateLeft,
        BASE.centerX,
        top,
    );
    arrows += `L ${BASE.centerX + arrowDx} ${top - arrowLength}`;

    arrows += makeArc(
        rotateLeft ? left : right,
        baseCenterY,
        rotateArcRadius,
        !rotateLeft,
        BASE.centerX,
        bottom,
    );
    arrows += `L ${BASE.centerX - arrowDx} ${bottom + arrowLength}`;
    return arrows;
}

/** Path to draw for turning left in place */
const leftArrowPath: string = makeArrowPath(true);
/** Path to draw for turning right in place */
const rightArrowPath: string = makeArrowPath(false);

/**
 * Draws circular arrows around the base for rotating in place
 *
 * @param rotateLeft if true draws counterclockwise arrow, if false draws
 * clockwise
 *  @returns the linear distance, the angle, and the trajectory element
 */
function drawRotate(rotateLeft: boolean): [number, number, JSX.Element] {
    const path = rotateLeft ? leftArrowPath : rightArrowPath;
    const trajectory = <path d={path} />;
    return [0, rotateLeft ? 1 : -1, trajectory];
}

/**
 * Draws a straight path backward from the base to the y position of the mouse
 *
 * @param y y position of the mouse on the SVG canvas
 * @returns the linear distance, the angle, and the trajectory element
 */
function drawBackward(y: number): [number, number, JSX.Element] {
    const leftPath = `M ${baseLeft} ${baseBack} ${baseLeft} ${y}`;
    const rightPath = `M ${baseRight} ${baseBack} ${baseRight} ${y}`;
    const centerPath = `M ${BASE.centerX} ${baseBack} ${BASE.centerX} ${y}`;
    const trajectory = (
        <>
            <path d={centerPath} style={{ strokeDasharray: strokeDasharray }} />
            <path d={leftPath} />
            <path d={rightPath} />
        </>
    );

    const distance = baseBack - y;
    const maxDistance = resolution_height - baseBack;
    const normalizedDistance = Math.max(
        distance / (maxDistance * distanceRatioForMaxSpeed),
        -1.0,
    );
    return [normalizedDistance, 0, trajectory];
}

/**Formats the SVG path arc string. */
function makeArc(
    startX: number,
    startY: number,
    radius: number,
    sweepFlag: boolean,
    endX: number,
    endY: number,
) {
    const sweep = sweepFlag ? 1 : 0;
    return `M ${startX},${startY} A ${radius} ${radius} 0 0 ${sweep} ${endX},${endY}`;
}
