import * as React from "react";
import { Polygon } from 'react-svg-path'

interface OverlayProps {
    width: number; // width of videostream
    height: number; // height of videostream
}

const createPolygon = (points: number[][]) => {
    return <svg xmlns="http://www.w3.org/2000/svg" className="action-overlay">
        <Polygon
            points={points}
            stroke="white"
            strokeWidth={3}
            fill="lightgray"
        />
    </svg>
}

// Button for driving forward on the navigation overhead fisheye camera
export const NavDriveForward = (props: OverlayProps) => {
    const points = [
        [0, props.width], 
        [props.height/2.1, props.width/1.5], 
        [props.height/2.1, props.width/2.75], 
        [0, 10] // 10px is padding to keep overlay inline with videostream
    ]
    return createPolygon(points)
};

// Button for driving backward on the navigation overhead fisheye camera
export const NavDriveBackward = (props: OverlayProps) => {
    const points = [
        [props.height, props.width], 
        [props.height/1.5, props.width/1.5], 
        [props.height/1.5, props.width/2.75], 
        [props.height, 10]
    ]
    return createPolygon(points)
};

// Button for rotating left on the navigation overhead fisheye camera
export const NavRotateLeft = (props: OverlayProps) => {
    const points = [
        [0, props.width], 
        [props.height, props.width], 
        [props.height/1.5, props.width/1.5], 
        [props.height/2.1, props.width/1.5]
    ]
    return createPolygon(points)
};

// Button for rotating right on the navigation overhead fisheye camera
export const NavRotateRight = (props: OverlayProps) => {
    const points = [
        [0,10], // 10px is padding to keep overlay inline with image
        [props.height, 10], // 10px is padding to keep overlay inline with image
        [props.height/1.5, props.width/3 + 10], 
        [props.height/2.1, props.width/3 + 10]
    ]
    return createPolygon(points)
};
