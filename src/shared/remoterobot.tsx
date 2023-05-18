import React from 'react'
import { cmd } from 'shared/commands';
import { ValidJoints, SensorData, RobotPose } from 'shared/util';

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
    private inJointLimits?: { [key in ValidJoints]?: [boolean, boolean] }
    private operaterCallback?: (inJointLimits: { [key in ValidJoints]?: [boolean, boolean] }) => void

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

    setInJointLimits(values: { [key in ValidJoints]?: [boolean, boolean] }) {
        let isTheSame = this.inJointLimits && Object.keys(values).every((key, index) => 
            values[key as ValidJoints]![0] == this.inJointLimits![key as ValidJoints]![0] &&
            values[key as ValidJoints]![1] == this.inJointLimits![key as ValidJoints]![1]
        )
        this.inJointLimits = values;
        if (!isTheSame && this.operaterCallback) {
            this.operaterCallback(this.inJointLimits)
        }
    }

    setOperatorCallback(operaterCallback: (inJointLimits: { [key in ValidJoints]?: [boolean, boolean] }) => void) {
        console.log(this)
        console.log(this.operaterCallback)
        this.operaterCallback = operaterCallback;
    }
}