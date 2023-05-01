import React from 'react'
import { CameraInfo, SignallingMessage } from "../util/util";
import { Server, Socket } from 'socket.io';
import io from 'socket.io-client';
import { Express } from 'express'

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
    private socket;
    private peerConnection?: RTCPeerConnection
    private onConnectionStart: () => void 
    private peerName: string
    private ignoreOffer: boolean = false
    cameraInfo: CameraInfo = {}
    // private onAvailableRobotsChanged: (available_robots: AvailableRobots) => void
    // private onConnectionEnd: () => void

    constructor(props) {
        super(props);
        this.onConnectionStart = props.onConnectionStart
        // this.onAvailableRobotsChanged = (available_robots: AvailableRobots) => { },
        // this.onConnectionEnd = props.onConnectionEnd
        this.createPeerConnection()
        this.peerName = props.peerName

        // this.initSocket().then(() => {
        //     this.socket.on('connect', () => {console.log("socket connected")})
        //     console.log(this.socket.connected)
        // })
        this.socket = io('http://localhost:5000', {transports: ["websocket"]});

        this.socket.on('connect', () => {console.log("socket connected")})

        this.socket.on('connect_error', (error) => {
            console.log('Connection error:', error);
          });

          this.socket.on('connect_timeout', () => {
            console.log('Connection timed out');
          });
          
        this.socket.on('reconnect_attempt', (attemptNumber) => {
            console.log(`Trying to reconnect (attempt ${attemptNumber})...`);
          });
        
          this.socket.on('reconnect_error', (error) => {
            console.log('Reconnection error:', error);
          });
          
        this.socket.on('join', room => {
            console.log('Another peer made a request to join room ' + room);
            console.log('I am ' + peerName + '!');
        });

        this.socket.on('joined', room => {
            if (this.onConnectionStart) this.onConnectionStart()
            console.log('joined: ' + room);
        });

        this.socket.on('signalling', (message: SignallingMessage) => {
            let {candidate, sessionDescription, cameraInfo} = message;
            console.log('Received message:', message);
            if (cameraInfo) {
                if (Object.keys(cameraInfo).length === 0) {
                    console.warn("Received a camera info mapping with no entries")
                }
                this.cameraInfo = cameraInfo
            } else if (candidate !== undefined && this.pc) {
                // Note that the last candidate will be null, so we check whether the candidate variable is
                // defined at all versus being truthy.
                this.peerConnection?.addIceCandidate(candidate).catch(e => {
                    if (!this.ignoreOffer) {
                        console.log("Failure during addIceCandidate(): " + e.name);
                        throw e;
                    }
                });
            } else {
                console.error("Unable to handle message")
            }
        });
    }

    async initSocket() {
        return new Promise((resolve, reject) => {
            try {
                this.socket = io('http://localhost:5000', {transports: ["websocket"]});
                resolve('Created socket')
            } catch {
                reject('Could not create socket')
            }
        });
    }

    createPeerConnection() {
        try {
            this.peerConnection = new RTCPeerConnection( peerConstraints );
            this.peerConnection.onicecandidate = (event) => {
                console.log("ICE candidate available")
                this.sendSignallingMessage({
                    candidate: event.candidate!
                });
            };
        } catch (e: any) {
            console.error('Failed to create PeerConnection, exception: ' + e.message);
            return;
        }
    }
    
    connectToRobot(robot: string) {
        console.log('attempting to connect to robot =');
        console.log(robot);
        // if (this.onConnectionStart) this.onConnectionStart()
        this.socket.emit('join', 'robot')
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

    sendSignallingMessage(message: SignallingMessage) {
        this.socket.emit('signalling', message);
    }
}
