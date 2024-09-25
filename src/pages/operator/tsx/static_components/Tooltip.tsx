// Code adapted from https://coderpad.io/blog/development/react-tooltip-a-how-to-guide/

import React from "react";
import "operator/css/Tooltip.css";

type TooltipProps = {
    children: any;
    text: string;
    divProps?: { [x: string]: any };
    position: "top" | "bottom" | "left" | "right";
    style?: React.CSSProperties;
};

export const Tooltip = (props: TooltipProps) => {
    return (
        <div className="tooltip-trigger">
            {props.children}
            <div
                className={`tooltip tooltip-${props.position}`}
                style={props.style}
            >
                {props.text}
            </div>
        </div>
    );
};
