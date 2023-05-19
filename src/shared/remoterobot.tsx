import React from 'react'
import { cmd } from 'shared/commands';
import { ValidJointStateDict, SensorData, RobotPose, ValidJoints } from 'shared/util';

export type robotMessageChannel = (message: cmd) => void; 

export class RemoteRobot extends React.Component {
    robotChannel: robotMessageChannel;
    sensors: RobotSensors
    
    constructor(props: {robotChannel: robotMessageChannel}) {
        super(props);
        this.robotChannel = props.robotChannel
        this.sensors = new RobotSensors({})
    }

    driveBase(linVel: number, angVel: number) {
        let cmd: cmd = {
            type: "driveBase",
            modifier: { linVel: linVel, angVel: angVel }
        };
        this.robotChannel(cmd);
        
        return {
            "stop": () => {
                let stopEvent: cmd = {
                    type: "driveBase",
                    modifier: { linVel: 0, angVel: 0 }
                }
                this.robotChannel(stopEvent)
            }
        }
    }

    incrementalMove(jointName: ValidJoints, increment: number) {
        let cmd: cmd = { 
            type: "incrementalMove", 
            jointName: jointName, 
            increment: increment
        }
        this.robotChannel(cmd);

        return {
            "stop": () => {
                this.robotChannel({ type: "stop" })
            }
        }
    }

    setRobotMode(mode: "position" | "navigation") {
        let cmd: cmd = {
            type: "setRobotMode",
            modifier: mode
        }
        this.robotChannel(cmd)
    }
}

class RobotSensors extends React.Component {
    private sensors: { [sensorName: string]: SensorData }
    private jointState?: RobotPose;
    private inJointLimits?: ValidJointStateDict
    private inCollision?: ValidJointStateDict
    private operaterCallback?: (inJointLimits: ValidJointStateDict, inCollision: ValidJointStateDict) => void

    constructor(props: {}) {
        super(props)
        this.sensors = {
            "lift": {},
            "arm": {},
            "wrist": {},
            "gripper": {},
            "head": {},
            "base": {}
        }
        this.operaterCallback = () => {}
        this.setOperatorCallback = this.setOperatorCallback.bind(this)
    }

    checkValidJointState(jointValues: ValidJointStateDict, effortValues: ValidJointStateDict) {
        let isTheSame = 
            this.inJointLimits && 
            Object.keys(jointValues).every((key, index) => 
                jointValues[key as ValidJoints]![0] == this.inJointLimits![key as ValidJoints]![0] &&
                jointValues[key as ValidJoints]![1] == this.inJointLimits![key as ValidJoints]![1]
            ) &&
            this.inCollision &&
            Object.keys(effortValues).every((key, index) => 
                effortValues[key as ValidJoints]![0] == this.inCollision![key as ValidJoints]![0] &&
                effortValues[key as ValidJoints]![1] == this.inCollision![key as ValidJoints]![1]
            )
        this.inJointLimits = jointValues;
        this.inCollision = effortValues;
        if (!isTheSame && this.operaterCallback) {
            this.operaterCallback(this.inJointLimits, this.inCollision)
        }
    }

    setOperatorCallback(operaterCallback: (inJointLimits: ValidJointStateDict, inCollision: ValidJointStateDict) => void) {
        this.operaterCallback = operaterCallback;
    }
}