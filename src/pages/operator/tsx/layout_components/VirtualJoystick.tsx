import React from "react";
import { CustomizableComponentProps, isSelected } from "./CustomizableComponent";
import { className } from "shared/util";
import { SVG_RESOLUTION } from "../utils/svg";
import { predicitiveDisplayFunctionProvider } from "..";
import "operator/css/VirtualJoystick.css"

const OUTER_RADIUS = SVG_RESOLUTION / 2 * 0.7;
const JOYSTICK_RADIUS = SVG_RESOLUTION / 2 * 0.4;


export const VirtualJoystick = (props: CustomizableComponentProps) => {
    const svgRef = React.useRef<SVGSVGElement>(null);
    const [active, setActive] = React.useState<boolean>(false);
    const { customizing } = props.sharedState;
    const selected = isSelected(props);
    const [joystick, setJoystick] = React.useState<JSX.Element>(drawJoystickCenter());
    const functions = predicitiveDisplayFunctionProvider.provideFunctions(handleSetActive);

    function handleSetActive(active: boolean) {
        if (!active) {
            setJoystick(drawJoystickCenter());
        }
        setActive(active);
    }

    const length = React.useRef<number>(0);
    const angle = React.useRef<number>(0);

    function drawJoystickCenter() {
        return drawJoystick(SVG_RESOLUTION / 2, SVG_RESOLUTION / 2)
    }

    function drawJoystick(x: number, y: number) {
        const newJoystick = (
            <circle className="joystick" cx={x} cy={y} r={JOYSTICK_RADIUS} />
        );
        return newJoystick
    }

    function handleLeave() {
        setJoystick(drawJoystickCenter);
        if (functions.onLeave) {
            functions.onLeave();
        }
    }

    function handleClick(event: React.MouseEvent<SVGSVGElement>) {
        if (!active) {
            setJoystickToMouse(event);
        }
        if (functions.onClick) functions.onClick(length.current, angle.current);
    }

    function handleRelease() {
        if (functions.onRelease) functions.onRelease();
    }

    function setLengthAndWidth(x: number, y: number) {
        const xLocal = x - SVG_RESOLUTION / 2;
        const yLocal = y - SVG_RESOLUTION / 2;

        angle.current = -xLocal / (SVG_RESOLUTION / 2)
        length.current = -yLocal / (SVG_RESOLUTION / 2);
    }

    function setJoystickToMouse(event: React.MouseEvent<SVGSVGElement>): [number, number] {
        const { clientX, clientY } = event;
        const svg = svgRef.current;
        if (!svg) return [0, 0];

        // Get x and y in terms of the SVG element
        const rect = svg.getBoundingClientRect();
        const x = (clientX - rect.left) / rect.width * SVG_RESOLUTION;
        const y = (clientY - rect.top) / rect.height * SVG_RESOLUTION;
        setLengthAndWidth(x, y);
        setJoystick(drawJoystick(x, y));
        return [x, y];

    }

    function handleMove(event: React.MouseEvent<SVGSVGElement>) {
        if (!active) return;
        setJoystickToMouse(event);
        if (functions.onMove) functions.onMove(length.current, angle.current);
    }

    function handleSelect(event: React.MouseEvent<SVGSVGElement>) {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path)
    }

    const controlProps = customizing ? { onClick: handleSelect } : {
        onPointerMove: active ? handleMove : undefined,
        onPointerLeave: handleLeave,
        onPointerDown: handleClick,
        onPointerUp: handleRelease
    };
    return (
        <div className={className("virtual-joystick", { customizing, selected, active })} >
            <svg
                ref={svgRef}
                viewBox={`0 0 ${SVG_RESOLUTION} ${SVG_RESOLUTION}`}
                preserveAspectRatio={'xMidYMid meet'}
                {...controlProps}
            >
                <circle className="outer-circle" cx={SVG_RESOLUTION / 2} cy={SVG_RESOLUTION / 2} r={OUTER_RADIUS} />
                {/* <path d={`M 0 ${SVG_RESOLUTION / 2} H ${SVG_RESOLUTION}`} />
                <path d={`M ${SVG_RESOLUTION / 2} 0 V ${SVG_RESOLUTION}`} /> */}
                {joystick}
            </svg>
        </div>
    );
}
