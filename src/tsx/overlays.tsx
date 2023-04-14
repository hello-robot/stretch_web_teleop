import * as React from "react";
import { Polygon } from 'react-svg-path'
import { StringLiteralType } from "typescript";
import { ExecuteBaseVelocity } from './robot'
import * as ROSLIB from 'roslib';
import { useRos } from 'rosreact'
import { ROSJointState } from '../util/util'

interface OverlayProps {
    width: number; // width of videostream
    height: number; // height of videostream
}

const createPolygon = (points: number[][], regionName: string) => {
    console.log('polygon')
    return (
        <div onClick={() => {callback(regionName)}}>
            <svg xmlns="http://www.w3.org/2000/svg" className="action-overlay" id={regionName}>
                <Polygon
                    points={points}
                    stroke="white"
                    strokeWidth={3}
                    fill="lightgray"
                />
            </svg>
        </div>
    )
}

const callback = (regionName: string) => {
    if (regionName == 'navDriveForward') {
        ExecuteBaseVelocity({linVel: 0.5, angVel: 0})
    } else if (regionName == 'navDriveBackward') {
        ExecuteBaseVelocity({linVel: -0.1, angVel: 0})
    } else if (regionName == 'navRotateRight') {
        ExecuteBaseVelocity({linVel: 0, angVel: 0.1})
    } else if (regionName == 'navRotateLeft') {
        ExecuteBaseVelocity({linVel: 0, angVel: -0.1})
    }
}

// Button for driving forward on the navigation overhead fisheye camera
const NavDriveForward = (props: OverlayProps) => {
    const points = [
        [0, props.width], 
        [props.height/2.1, props.width/1.5], 
        [props.height/2.1, props.width/2.75], 
        [0, 10] // 10px is padding to keep overlay inline with videostream
    ]
    return createPolygon(points, 'navDriveForward')
};

// Button for driving backward on the navigation overhead fisheye camera
const NavDriveBackward = (props: OverlayProps) => {
    const points = [
        [props.height, props.width], 
        [props.height/1.5, props.width/1.5], 
        [props.height/1.5, props.width/2.75], 
        [props.height, 10]
    ]
    return createPolygon(points, 'navDriveBackward')
};

// Button for rotating left on the navigation overhead fisheye camera
const NavRotateLeft = (props: OverlayProps) => {
    const points = [
        [0, props.width], 
        [props.height, props.width], 
        [props.height/1.5, props.width/1.5], 
        [props.height/2.1, props.width/1.5]
    ]
    return createPolygon(points, 'navRotateLeft')
};

// Button for rotating right on the navigation overhead fisheye camera
const NavRotateRight = (props: OverlayProps) => {
    const points = [
        [0,10], // 10px is padding to keep overlay inline with image
        [props.height, 10], // 10px is padding to keep overlay inline with image
        [props.height/1.5, props.width/3 + 10], 
        [props.height/2.1, props.width/3 + 10]
    ]
    return createPolygon(points, 'navRotateRight')
};

export const OverheadNavActionOverlay = (props: OverlayProps) => {
    return (
        <div>
            <NavDriveForward width={props.width} height={props.height}/>
            <NavDriveBackward width={props.width} height={props.height}/>
            <NavRotateLeft width={props.width} height={props.height}/>
            <NavRotateRight width={props.width} height={props.height}/>
        </div>
    )
}