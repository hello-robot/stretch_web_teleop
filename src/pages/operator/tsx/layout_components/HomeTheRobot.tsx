import React, { useEffect, useState } from "react";
import { homeTheRobotFunctionProvider } from "../index";
import { Tooltip } from "../static_components/Tooltip";
import "operator/css/HomeTheRobot.css";
import { isMobile } from "react-device-detect";
import HomeIcon from "@mui/icons-material/Home";
import CircularProgress from '@mui/material/CircularProgress';


/** All the possible button functions */
export enum HomeTheRobotFunction {
    Home,
}

export interface HomeTheRobotFunctions {
    Home: () => void;
}

export const HomeTheRobot = (props: { hideLabels: boolean }) => {
    const [loading, setLoading] = useState<boolean>(false);
    homeTheRobotFunctionProvider.setModeIsHomingCallback(setLoading);

    let functions: HomeTheRobotFunctions = {
        Home: homeTheRobotFunctionProvider.provideFunctions(
            HomeTheRobotFunction.Home,
        ) as () => void,
    };

    return !isMobile ? (
        <React.Fragment>
            <div id="home-the-robot-container">
                <Tooltip text="Robots with relative encoders (vs absolute encoders) need a homing procedure when they power on. For Stretch, it is a 45-second sequence of motions to find each joint's zero position. Un-homed joints will be greyed-out until this procedure occurs." position="top">
                    {loading ? (
                        <p className="home-msg"><b style={{fontWeight: "bold"}}>Robot is homing...</b> Please wait.</p>
                    ) : (
                        <p className="home-msg"><b style={{fontWeight: "bold"}}>Robot is not homed.</b> Please drive the robot to a safe position and press the home button.</p>
                    )}
                </Tooltip>
                <button
                    onClick={() => {
                        functions.Home();
                    }}
                >
                    {loading ? (
                        <div className="home-btn">
                            <span hidden={props.hideLabels}>Loading...</span>
                            <CircularProgress size="1.2rem" color="inherit" />
                        </div>
                    ) : (
                        <div className="home-btn">
                            <span hidden={props.hideLabels}>Home</span>
                            <HomeIcon />
                        </div>
                    )}
                </button>
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
