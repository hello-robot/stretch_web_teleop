import React from 'react'
import { cmd, DriveCommand, CameraPerspectiveCommand, IncrementalMove, setRobotModeCommand, VelocityCommand, RobotPoseCommand } from 'shared/commands';
import { ValidJointStateDict, RobotPose, ValidJoints } from 'shared/util';

export type robotMessageChannel = (message: cmd) => void;

export class RemoteRobot extends React.Component {
    robotChannel: robotMessageChannel;
    sensors: RobotSensors

    constructor(props: { robotChannel: robotMessageChannel }) {
        super(props);
        this.robotChannel = props.robotChannel
        this.sensors = new RobotSensors({})
    }

    driveBase(linVel: number, angVel: number): VelocityCommand {
        let cmd: DriveCommand = {
            type: "driveBase",
            modifier: { linVel: linVel, angVel: angVel }
        };
        this.robotChannel(cmd);

        return {
            "stop": () => {
                let stopEvent: DriveCommand = {
                    type: "driveBase",
                    modifier: { linVel: 0, angVel: 0 }
                }
                this.robotChannel(stopEvent)
            },
            "affirm": () => {
                let affirmEvent: DriveCommand = {
                    type: "driveBase",
                    modifier: { linVel: linVel, angVel: angVel }
                }
                this.robotChannel(affirmEvent)
            }
        }
    }

    incrementalMove(jointName: ValidJoints, increment: number): VelocityCommand {
        let cmd: IncrementalMove = {
            type: "incrementalMove",
            jointName: jointName,
            increment: increment
        };
        this.robotChannel(cmd);

        return {
            "stop": () => {
                this.robotChannel({ type: "stop" })
            }
        }
    }

    setRobotMode(mode: "position" | "navigation") {
        let cmd: setRobotModeCommand = {
            type: "setRobotMode",
            modifier: mode
        };
        this.robotChannel(cmd)
    }

    setCameraPerspective(camera: "overhead" | "realsense" | "gripper", perspective: string) {
        let cmd: CameraPerspectiveCommand = {
            type: "setCameraPerspective",
            camera: camera,
            perspective: perspective
        }
        this.robotChannel(cmd)
    }

    setRobotPose(pose: RobotPose) {
        let cmd: RobotPoseCommand = {
            type: "setRobotPose",
            pose: pose,
        }
        this.robotChannel(cmd)
    }

    setFollowGripper(followGripper: boolean) {
        let cmd: cmd = {
            type: "setFollowGripper",
            followGripper: followGripper
        }
        this.robotChannel(cmd)
    }
}

class RobotSensors extends React.Component {
    private jointState?: RobotPose;
    private inJointLimits: ValidJointStateDict = {};
    private inCollision: ValidJointStateDict = {};
    private functionProviderCallback?: (inJointLimits: ValidJointStateDict, inCollision: ValidJointStateDict) => void;

    constructor(props: {}) {
        super(props)
        this.functionProviderCallback = () => { }
        this.setFunctionProviderCallback = this.setFunctionProviderCallback.bind(this)
    }

    /**
     * Handler for joint state messages with information about if individual 
     * joints are in collision or at their limit.
     * 
     * @param jointValues mapping of joint name to a pair of booleans for 
     *                    [joint is at lower limit, joint is at upper limit]
     * @param effortValues mapping for joint name to pair of booleans for 
     *                     [joint in collision at lower end, joint is in 
     *                     collision at upper end]
     */
    checkValidJointState(jointValues: ValidJointStateDict, effortValues: ValidJointStateDict) {
        // Remove existing values from list
        let change = false;
        Object.keys(jointValues).forEach((k) => {
            const key = k as ValidJoints;

            const same = key in this.inJointLimits ?
                jointValues[key]![0] == this.inJointLimits[key]![0] &&
                jointValues[key]![1] == this.inJointLimits[key]![1] : false;
            // If same value, remove from dict so not passed to callback
            if (same) delete jointValues[key];
            else {
                change = true;
                this.inJointLimits[key] = jointValues[key];
            }
        })
        Object.keys(effortValues).forEach((k) => {
            const key = k as ValidJoints;
            const same = key in this.inCollision ?
                effortValues[key]![0] == this.inCollision[key]![0] &&
                effortValues[key]![1] == this.inCollision[key]![1] : false;
            // If same value, remove from dict so not passed to callback
            if (same) delete effortValues[key];
            else {
                change = true;
                this.inCollision[key] = effortValues[key];
            }
        })

        // Only callback when value has changed
        if (change && this.functionProviderCallback) {
            console.log(jointValues, effortValues);
            this.functionProviderCallback(jointValues, effortValues);
        }
    }

    /**
     * Records a callback from the function provider. The callback is called 
     * whenever a joint state "at limit" or "in collision" changes.
     * 
     * @param callback callback to function provider
     */
    setFunctionProviderCallback(callback: (inJointLimits: ValidJointStateDict, inCollision: ValidJointStateDict) => void) {
        this.functionProviderCallback = callback;
    }
}