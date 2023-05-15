import React from 'react'
import { cmd, VelocityCommand } from 'utils/commands';
import { RemoteStream } from 'utils/util';

export type robotMessageChannel = (message: cmd) => void; 

export class RemoteRobot extends React.Component {
    robotChannel: robotMessageChannel;
    // remoteStreams: Map<string, RemoteStream>

    constructor(props: {robotChannel: robotMessageChannel}) {
        super(props);
        this.robotChannel = props.robotChannel
        // this.remoteStreams = props.remoteStreams
    }

    testCommand(msg: string) {
        let cmd: cmd = {
            msg: msg
        }
        this.robotChannel(cmd)
        console.log("sending command")
    }

    driveWithVelocities(linVel: number, angVel: number) {
        let cmd: cmd = {
            type: "driveBase",
            modifier: {
                linVel: linVel,
                angVel: angVel
            }
        };

        this.robotChannel(cmd);
    }
    // getRemoteStream(streamName: string) {
    //     if (!this.remoteStreams.has(streamName)) throw 'remote stream ' + streamName + ' does not exist!'
    //     return this.remoteStreams.get(streamName)!.stream
    // }
}