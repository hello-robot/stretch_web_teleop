import * as WebRTC from 'react-native-web-webrtc';
import { io, Socket } from "socket.io-client";
import React, { useEffect, useState } from 'react'
import { SignallingMessage } from '../util/util';

const peerConstraints = {
    iceServers: [{
        urls: [
            'stun:stun1.l.google.com:19302',
            'stun:stun2.l.google.com:19302',
        ],
    }],
    iceCandidatePoolSize: 10,
};

export class WebRTCConnection extends React.Component {
    private socket: Socket
    private peerConnection?: RTCPeerConnection
    private onConnectionStart: () => void 
    private peerName: string
    // private onAvailableRobotsChanged: (available_robots: AvailableRobots) => void
    // private onConnectionEnd: () => void

    constructor(props) {
        super(props);
        this.onConnectionStart = props.onConnectionStart
        // this.onAvailableRobotsChanged = (available_robots: AvailableRobots) => { },
        // this.onConnectionEnd = props.onConnectionEnd
        this.createPeerConnection()
        this.peerName = props.peerName
        // this.socket = io('wss://localhost:9090', { transports : ['websocket'] })
    }

    createPeerConnection() {
        try {
            this.peerConnection = new RTCPeerConnection( peerConstraints );
        } catch (e: any) {
            console.error('Failed to create PeerConnection, exception: ' + e.message);
            return;
        }
    }
    
    connectToRobot(robot: string) {
        console.log('attempting to connect to robot =');
        console.log(robot);
        if (this.onConnectionStart) this.onConnectionStart()
    }

    addTrack(track: MediaStreamTrack, stream: MediaStream) {    
        this.peerConnection?.addTrack(track, stream);
        console.log("added track")
    }

    stop() {
        if (!this.peerConnection) throw 'peerConnection is undefined';
        const senders = this.peerConnection.getSenders();
        senders.forEach((sender) => this.peerConnection?.removeTrack(sender));
        this.peerConnection.close();
        this.createPeerConnection()
    }
}
