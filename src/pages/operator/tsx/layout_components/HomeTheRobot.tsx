import React, { useEffect, useState } from "react";
import { homeTheRobotFunctionProvider } from "../index";
import { Alert } from "../basic_components/Alert";
import { Tooltip } from "../static_components/Tooltip";
import "operator/css/HomeTheRobot.css";
import { isTablet, isBrowser } from "react-device-detect";
import HomeIcon from "@mui/icons-material/Home";
import QuestionMarkIcon from "@mui/icons-material/Help";
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

    return isTablet || isBrowser ? (
        <React.Fragment>
            <Alert
                type="loud-warning"
                hide_close_button={true}
                style={loading ? {} : { paddingTop: "0.8em" }}
            >
                <div className="home-alert-container">
                    {loading ? (
                        <div>
                            <span>
                                <b style={{ fontWeight: "bold" }}>
                                    Robot is homing.
                                </b>{" "}
                                Please wait...{" "}
                            </span>
                            <CircularProgress size="1.2rem" color="inherit" />
                        </div>
                    ) : (
                        <span className="home-info">
                            <b style={{ fontWeight: "bold" }}>
                                Robot is not homed.
                            </b>{" "}
                            Please drive the robot to a safe position and press
                            the home button.
                            <Tooltip
                                text="Stretch has a homing procedure when powered on. It consists of 45-second sequence of motions to find each joint's zero position. Un-homed joints will be greyed-out until this procedure occurs."
                                position="bottom"
                                style={{
                                    width: "400px",
                                    padding: "10px",
                                    zIndex: "10",
                                }}
                            >
                                <QuestionMarkIcon />
                            </Tooltip>
                        </span>
                    )}
                    {loading ? (
                        <></>
                    ) : (
                        <button
                            onClick={() => {
                                functions.Home();
                            }}
                            className="home-btn-container"
                        >
                            <div className="home-btn">
                                <span hidden={props.hideLabels}>Home</span>
                                <HomeIcon />
                            </div>
                        </button>
                    )}
                </div>
            </Alert>
        </React.Fragment>
    ) : (
        <React.Fragment>
            <Alert
                type="loud-warning"
                hide_close_button={true}
                style={loading ? {} : { paddingTop: "0.8em" }}
            >
                <span>Robot is not homed. Please switch to Desktop.</span>
            </Alert>
        </React.Fragment>
    );
};
