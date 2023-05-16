import React from 'react'
import { cmd } from 'utils/commands';
import { ValidJoints, SensorData, RobotPose } from 'utils/util';

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
}

class RobotSensors extends React.Component {
    private sensors: { [sensorName: string]: SensorData }
    private jointState?: RobotPose;

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
    }

    setJointState(state: RobotPose) {
        this.jointState = state;
    }
}