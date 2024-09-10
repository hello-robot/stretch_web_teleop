import React, { useEffect, useState } from "react";
import { homeTheRobotFunctionProvider } from "../index";
import { Alert } from "../basic_components/Alert";
import { Tooltip } from "../static_components/Tooltip";
import "operator/css/HomeTheRobot.css";
import { isMobile } from "react-device-detect";
import HomeIcon from "@mui/icons-material/Home";
import QuestionMarkIcon from '@mui/icons-material/QuestionMark';
import CircularProgress from "@mui/material/CircularProgress";
import { className } from "../../../../shared/util";

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
            <Alert type="loud-warning" hide_close_button={true} style={loading ? {} : {paddingTop: "0.8em"}}>
                <div>
                <button
                    onClick={() => {
                        functions.Home();
                    }}
                    className={"home-btn-container" + ( loading ? " fix-home-btn-margin" : "" )}
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
                {loading ? (
                    <span>
                        <b style={{ fontWeight: "bold" }}>
                            Robot is homing...
                        </b>{" "}
                        Please wait.
                    </span>
                ) : (
                    <span>
                        <b style={{ fontWeight: "bold" }}>
                            Robot is not homed
                            <Tooltip
                                text="Robots with relative encoders need a homing procedure when they power on. For Stretch, it is a 45-second sequence of motions to find each joint's zero position. Un-homed joints will be greyed-out until this procedure occurs."
                                position="bottom"
                                style={{width: "500px", padding: "10px", zIndex: "10"}}
                            >
                                <sup><QuestionMarkIcon fontSize="inherit" style={{ fontSize: "14px" }}/></sup>
                            </Tooltip>
                        </b>{" "}
                        Please drive the robot to a safe position and press
                        the home button.
                    </span>
                )}
                </div>
            </Alert>
        </React.Fragment>
    ) : (
        <React.Fragment>
            <Alert type="loud-warning" hide_close_button={true} style={loading ? {} : {paddingTop: "0.8em"}}>
                <span>Robot is not homed. Please switch to Desktop.</span>
            </Alert>
        </React.Fragment>
    );
};
