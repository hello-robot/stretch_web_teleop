import React from 'react'
import { cmd } from 'utils/commands';

export type robotMessageChannel = (message: cmd) => void; 

export class RemoteRobot extends React.Component {
    robotChannel: robotMessageChannel;

    constructor(props: {robotChannel: robotMessageChannel}) {
        super(props);
        this.robotChannel = props.robotChannel
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
    // getRemoteStream(streamName: string) {
    //     if (!this.remoteStreams.has(streamName)) throw 'remote stream ' + streamName + ' does not exist!'
    //     return this.remoteStreams.get(streamName)!.stream
    // }
}