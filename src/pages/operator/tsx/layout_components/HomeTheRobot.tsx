import React, { useEffect, useState } from "react";
import { homeTheRobotFunctionProvider } from "../index";
import { Tooltip } from "../static_components/Tooltip";
import "operator/css/HomeTheRobot.css";
import { isMobile } from "react-device-detect";
import HomeIcon from "@mui/icons-material/Home";


/** All the possible button functions */
export enum HomeTheRobotFunction {
    Home,
}

export interface HomeTheRobotFunctions {
    Home: () => void;
}

export const HomeTheRobot = (props: { hideLabels: boolean }) => {
    let functions: HomeTheRobotFunctions = {
        Home: homeTheRobotFunctionProvider.provideFunctions(
            HomeTheRobotFunction.Home,
        ) as () => void,
    };

    return !isMobile ? (
        <React.Fragment>
            <div id="home-the-robot-container">
                <Tooltip text="Robots with relative encoders (vs absolute encoders) need a homing procedure when they power on. For Stretch, it is a 45-second sequence of motions to find each joint's zero position." position="top">
                    <p style={{marginLeft: "40px"}}><b style={{fontWeight: "bold"}}>Home the Robot</b>. Un-homed joints will be greyed-out until this procedure occurs. You can use teleop the mobile base and head to find a clear place for the robot to home.</p>
                </Tooltip>
                <div style={{marginRight: "100px"}}>
                    <button
                        className="home-btn btn-label"
                        onClick={() => {
                            functions.Home();
                        }}
                    >
                        <span hidden={props.hideLabels}>Home</span>
                        <HomeIcon />
                    </button>
                </div>
            </div>
        </React.Fragment>
    ) : (
        <React.Fragment>
            <div id="home-the-robot-container">
                Home-The-Robot not yet implemented for mobile
            </div>
        </React.Fragment>
    );
};
