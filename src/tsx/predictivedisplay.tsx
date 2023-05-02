import React from "react"
import { SVG_RESOLUTION, OVERHEAD_ROBOT_BASE as BASE } from "../util/svg";
import { navigationProps } from "../util/util";
import "../css/predictivedisplay.css"

/**
 * Scales height values to fit in the navigation camera
 * @param y the number to scale
 * @returns scaled number
 */
const scaleToNavAspectRatio = (y: number) => {
    return y / navigationProps.width * navigationProps.height;
}

/**Height of the predictive display SVG */
const resolution_height = scaleToNavAspectRatio(SVG_RESOLUTION)

/**The front of the robot */
const baseFront = scaleToNavAspectRatio(BASE.centerY - BASE.height/2);
/**Left side of the robot */
const baseLeft = BASE.centerX - BASE.width/2;
/**Right side of the robot */
const baseRight = BASE.centerX + BASE.width/2;

/**Formats the SVG path arc string. */
function makeArc(startX: number, startY: number, radius: number, sweepFlag: boolean, endX: number, endY: number) {
    const sweep = sweepFlag ? 1 : 0;
    return `M ${startX},${startY} A ${radius} ${radius} 0 0 ${sweep} ${endX},${endY}`  
}

/** Properties for the PredictiveDisplay component */
interface PredictiveDisplayProps {
    /** Callback function when mouse is clicked in predicitive display area */
    onClick: (length: number, angle: number) => void;
    /** Callback function when cursor is moved in predictive display area */
    onMove?: (length: number, angle: number) => void;
    /** Callback function for release, also called when the user exits the 
     * predictive display area
     */
    onRelease?: () => void;
}

/** State for the PredicitiveDisplay component */
interface PredictiveDisplayState {
    /** Components to render to display the trajectory. */
    trajectory: React.ReactNode;
}

export class PredictiveDisplay extends React.Component<PredictiveDisplayProps, PredictiveDisplayState> {
    svgRef: React.RefObject<SVGSVGElement>;

    constructor(props: PredictiveDisplayProps) {
        super(props);
        this.state = {
            trajectory: undefined
        }
        this.svgRef = React.createRef();
        this.onMouseMove = this.onMouseMove.bind(this);
        this.drawForwardTraj = this.drawForwardTraj.bind(this);

    }   

    /**
     * Draws an arc from the base to the cursor, such that the arc is normal
     * to the base.
     * @param x horizontal position of the cursor
     * @param y vertical position of the cursor
     */
    drawForwardTraj(x: number, y: number) {
        const dx = BASE.centerX - x;
        const dy = baseFront - y;
        const heading = Math.atan2(-dx, dy)
        const sweepFlag = dx < 0;

        const c = Math.sqrt(dx * dx + dy * dy)
        const radius = c / (2 * Math.sin(heading))
        const centerPath = makeArc(BASE.centerX, baseFront, radius, sweepFlag, x, y);

        const leftEndX = x - BASE.width/2 * Math.cos(2 * heading)
        const leftEndY = y - BASE.width/2 * Math.sin(2 * heading)
        const leftRadius = radius+BASE.width/2
        const leftPath = makeArc(baseLeft, baseFront, leftRadius, sweepFlag, leftEndX, leftEndY);

        const rightEndX = x + BASE.width/2 * Math.cos(2 * heading)
        const rightEndY = y + BASE.width/2 * Math.sin(2 * heading)
        const rightRadius = radius-BASE.width/2
        const rightPath = makeArc(baseRight, baseFront, rightRadius, sweepFlag, rightEndX, rightEndY);

        const trajectory = (
            <>
                <path d={centerPath} style={{strokeDasharray: "4 10"}}/>
                <path d={leftPath} />
                <path d={rightPath} />
            </>
        );
        this.setState({trajectory});
        const arcLength = 2 * radius * heading;
        return {length: arcLength, angle: heading}
    }

    /** Updates the trajectory display and calls callback.*/
    onMouseMove(event: React.MouseEvent<SVGSVGElement>, click?: boolean) {
        const {clientX, clientY} = event;
        const svg = this.svgRef.current;
        if (!svg) return;
        const rect = svg.getBoundingClientRect();
        // Get x and y in terms of the SVG element
        const x = (clientX - rect.left) / rect.width * SVG_RESOLUTION;
        const pixelY = (clientY - rect.top) / rect.height;
        const y = scaleToNavAspectRatio(pixelY * SVG_RESOLUTION);
        let length: number;
        let angle: number;
        if (y < baseFront) {
            const ret = this.drawForwardTraj(x, y)
            length = ret.length;
            angle = ret.angle;
        } else {
            this.setState({trajectory: undefined})
            length = 0;
            angle = 0;
        }
        // Call the passed in move callback function
        if (!click && this.props.onMove) {
            this.props.onMove(length, angle);
        // Or call the passed in click callback
        } else if (click && this.props.onClick) {
            this.props.onClick(length, angle);
        }
    }

    onMouseLeave() {
        this.setState({trajectory: undefined});
        if (this.props.onRelease) {
            this.props.onRelease()
        }
    }

    render() {
        return (
            <svg
                viewBox={`0 0 ${SVG_RESOLUTION} ${resolution_height}`}
                preserveAspectRatio="none"
                ref={this.svgRef}
                onMouseMove={this.onMouseMove}
                onMouseLeave={() => {this.setState({trajectory: undefined})}}
                className="predictive-display"
                onClick={(e) => this.onMouseMove(e, true)}
            >
                {this.state.trajectory}
            </svg>
        )
    }
}