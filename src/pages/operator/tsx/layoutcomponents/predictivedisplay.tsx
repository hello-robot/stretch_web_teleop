import React from "react"
import { SVG_RESOLUTION, percent2Pixel, OVERHEAD_ROBOT_BASE as BASE } from "shared/svg";
import { className, navigationProps } from "shared/util";
import "operator/css/predictivedisplay.css"
import { CustomizableComponentProps } from "./customizablecomponent";

/**
 * Scales height values to fit in the navigation camera
 * @param y the number to scale
 * @returns scaled number
 */
const scaleToNavAspectRatio = (y: number) => {
    return y / navigationProps.width * navigationProps.height;
}

/**Arguments for drawing the dashed line in the center of the path */
const strokeDasharray = "4 10"
/**Height of the predictive display SVG */
const resolution_height = scaleToNavAspectRatio(SVG_RESOLUTION)
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

/**Formats the SVG path arc string. */
function makeArc(startX: number, startY: number, radius: number, sweepFlag: boolean, endX: number, endY: number) {
    const sweep = sweepFlag ? 1 : 0;
    return `M ${startX},${startY} A ${radius} ${radius} 0 0 ${sweep} ${endX},${endY}`
}

export type PredictiveDisplayFunctions = {
    /** Callback function when mouse is clicked in predicitive display area */
    onClick: (length: number, angle: number) => void;
    /** Callback function when cursor is moved in predictive display area */
    onMove?: (length: number, angle: number) => void;
    /** Callback function for release */
    onRelease?: () => void;
    /** Callback function for leaving predictive display area */
    onLeave?: () => void;
}

/** Properties for the {@link PredictiveDisplay} component */
type PredictiveDisplayProps = CustomizableComponentProps & {
    functions: PredictiveDisplayFunctions;
}

/** State for the {@link PredicitiveDisplay} component */
interface PredictiveDisplayState {
    /** Components to render to display the trajectory. */
    trajectory: React.ReactNode;
}

/**
 * Creates the SVG path elements for circular arrows around the base.
 * @param rotateLeft if true draws a path counter-clockwise, otherwise clockwise
 * @returns SVG path string description of the arrows
 */
const makeArrowPath = (rotateLeft: boolean) => {
    const arrowLength = percent2Pixel(2.5);
    const top = baseCenterY - rotateArcRadius;
    const bottom = baseCenterY + rotateArcRadius;
    const left = BASE.centerX - rotateArcRadius;
    const right = BASE.centerX + rotateArcRadius;
    const arrowDx = rotateLeft ? arrowLength : -arrowLength;

    let arrows = makeArc(rotateLeft ? right : left, baseCenterY, rotateArcRadius, !rotateLeft, BASE.centerX, top)
    arrows += `L ${BASE.centerX + arrowDx} ${top - arrowLength}`

    arrows += makeArc(rotateLeft ? left : right, baseCenterY, rotateArcRadius, !rotateLeft, BASE.centerX, bottom)
    arrows += `L ${BASE.centerX - arrowDx} ${bottom + arrowLength}`
    return arrows
}

/** Path to draw for turning left in place */
const leftArrowPath: string = makeArrowPath(true);
/** Path to draw for turning right in place */
const rightArrowPath: string = makeArrowPath(false);

export class PredictiveDisplay extends React.Component<PredictiveDisplayProps, PredictiveDisplayState> {
    svgRef: React.RefObject<SVGSVGElement>;
    length: number;
    angle: number;
    holding: boolean

    constructor(props: PredictiveDisplayProps) {
        super(props);
        this.state = {
            trajectory: undefined,
        }

        this.length = 0;
        this.angle = 0;
        this.holding = false;

        this.svgRef = React.createRef();
        this.drawForwardTraj = this.drawForwardTraj.bind(this);
        this.drawRotate = this.drawRotate.bind(this);
        this.drawBackward = this.drawBackward.bind(this);
        this.handleClick = this.handleClick.bind(this);
        this.handleLeave = this.handleLeave.bind(this);
        this.handleMove = this.handleMove.bind(this);
        this.handleRelease = this.handleRelease.bind(this);
    }

    /**
     * Draws an arc from the base to the cursor, such that the arc is normal
     * to the base.
     * @param x horizontal position of the cursor
     * @param y vertical position of the cursor
     */
    drawForwardTraj(x: number, y: number): [number, number] {
        const dx = BASE.centerX - x;
        const dy = baseFront - y;
        const heading = Math.atan2(-dx, dy)
        const sweepFlag = dx < 0;

        const c = Math.sqrt(dx * dx + dy * dy)  // length from base to cursor
        const radius = c / (2 * Math.sin(heading))  // radius of the center curve
        const centerPath = makeArc(BASE.centerX, baseFront, radius, sweepFlag, x, y);

        // Next to base, draw rotate trajectory. 
        // note: this handles the case where the cursor is too close to the base
        // of the robot for the robot to achieve that position with only forward
        // wheel spin
        if (Math.abs(radius) < BASE.width / 2) {
            return this.drawRotate(x < BASE.centerX);
        }

        const leftEndX = x - BASE.width / 2 * Math.cos(2 * heading)
        const leftEndY = y - BASE.width / 2 * Math.sin(2 * heading)
        const leftRadius = radius + BASE.width / 2
        const leftPath = makeArc(baseLeft, baseFront, leftRadius, sweepFlag, leftEndX, leftEndY);

        const rightEndX = x + BASE.width / 2 * Math.cos(2 * heading)
        const rightEndY = y + BASE.width / 2 * Math.sin(2 * heading)
        const rightRadius = radius - BASE.width / 2
        const rightPath = makeArc(baseRight, baseFront, rightRadius, sweepFlag, rightEndX, rightEndY);

        const trajectory = (
            <>
                <path d={centerPath} style={{ strokeDasharray: strokeDasharray }} />
                <path d={leftPath} />
                <path d={rightPath} />
            </>
        );
        this.setState({ trajectory });
        const arcLength = 2 * radius * heading;
        return [arcLength, heading];
    }

    /**
     * Draws circular arrows around the base for rotating in place
     * @param rotateLeft if true draws counterclockwise arrow, if false draws 
     * clockwise
     */
    drawRotate(rotateLeft: boolean): [number, number] {
        const path = rotateLeft ? leftArrowPath : rightArrowPath;
        const trajectory = (
            <path d={path} />
        );
        this.setState({ trajectory })
        return [0, -1]  // TODO: map rotate click to angular velocity
    }

    /**
     * Draws a straigh path backward from the base to the y position of the mouse
     * @param y y position of the mouse on the SVG canvas
     * @returns length and angle of the click
     */
    drawBackward(y: number): [number, number] {
        const leftPath = `M ${baseLeft} ${baseBack} ${baseLeft} ${y}`
        const rightPath = `M ${baseRight} ${baseBack} ${baseRight} ${y}`
        const centerPath = `M ${BASE.centerX} ${baseBack} ${BASE.centerX} ${y}`
        const trajectory = (
            <>
                <path d={centerPath} style={{ strokeDasharray: strokeDasharray }} />
                <path d={leftPath} />
                <path d={rightPath} />
            </>
        );
        this.setState({ trajectory })
        return [baseBack - y, 0];
    }

    /** Updates the trajectory display and calls callback.*/
    drawTrajectory(event: React.MouseEvent<SVGSVGElement>) {
        const { clientX, clientY } = event;
        const svg = this.svgRef.current;
        if (!svg) return;
        const rect = svg.getBoundingClientRect();
        // Get x and y in terms of the SVG element
        const x = (clientX - rect.left) / rect.width * SVG_RESOLUTION;
        const pixelY = (clientY - rect.top) / rect.height;
        const y = scaleToNavAspectRatio(pixelY * SVG_RESOLUTION);

        let ret: [number, number];
        if (y < baseFront) {
            ret = this.drawForwardTraj(x, y)
        } else if (y < baseBack) {
            // Next to base, draw rotate trajectory
            ret = this.drawRotate(x < BASE.centerX);
        } else {
            // Move backward
            ret = this.drawBackward(y);
        }

        const [length, angle] = ret;
        this.length = length;
        this.angle = angle;
    }

    handleLeave() {
        this.setState({ trajectory: undefined });
        if (this.props.functions.onLeave) {
            this.props.functions.onLeave()
        }
    }

    handleClick(e) {
        this.holding = true;
        if (this.props.functions.onClick) {
            this.props.functions.onClick(this.length, this.angle);
        }
    }

    handleRelease(e) {
        this.holding = false;
        if (this.props.functions.onRelease) {
            this.props.functions.onRelease()
        }
    }

    handleMove(e) {
        this.drawTrajectory(e);
        if (this.holding && this.props.functions.onMove) {
            this.props.functions.onMove(this.length, this.angle);
        }
    }

    render() {
        const customizing = this.props.sharedState.customizing;
        const controlProps = customizing ? {} : {
            onMouseMove: this.handleMove,
            onMouseLeave: this.handleLeave,
            onMouseDown: this.handleClick,
            onMouseUp: this.handleRelease
        };
        return (
            <svg
                viewBox={`0 0 ${SVG_RESOLUTION} ${resolution_height}`}
                preserveAspectRatio="none"
                ref={this.svgRef}
                className={className("predictive-display", { customizing })}
                {...controlProps}
            >
                {this.state.trajectory}
            </svg>
        )
    }
}