import React from 'react'
import { CameraInfo, SignallingMessage, WebRTCMessage } from "shared/util";
import io, { Socket } from 'socket.io-client';
import { safelyParseJSON, generateUUID } from 'shared/util'

const peerConstraints = {
    iceServers: [{
        urls: [
            'stun:stun1.l.google.com:19302',
        ],
    }]
};

interface WebRTCProps {
    peerRole: "operator" | "robot";
    polite: boolean;
    onTrackAdded?: (ev: RTCTrackEvent) => void;
    onMessage: (message: WebRTCMessage) => void;
    onRobotConnectionStart?: () => void;
    onMessageChannelOpen?: () => void;
    onConnectionEnd?: () => void;
}

export class WebRTCConnection extends React.Component {
    private socket: Socket;
    private peerConnection?: RTCPeerConnection
    private peerRole: string
    private polite: boolean
    private makingOffer = false
    private ignoreOffer: boolean = false
    private isSettingRemoteAnswerPending = false
    private pendingIceCandidates: RTCIceCandidate[]

    cameraInfo: CameraInfo = {}

    private messageChannel?: RTCDataChannel

    private onTrackAdded?: (ev: RTCTrackEvent) => void
    private onMessage: (obj: WebRTCMessage) => void
    private onRobotConnectionStart?: () => void 
    private onMessageChannelOpen?: () => void
    private onConnectionEnd?: () => void

    constructor(props: WebRTCProps) {
        super(props);
        this.peerRole = props.peerRole
        this.polite = props.polite
        this.onRobotConnectionStart = props.onRobotConnectionStart
        this.onMessage = props.onMessage
        this.onTrackAdded = props.onTrackAdded
        this.onMessageChannelOpen = props.onMessageChannelOpen
        this.onConnectionEnd = props.onConnectionEnd
        this.pendingIceCandidates = []

        this.createPeerConnection()

        this.socket = io();

        this.socket.on('connect', () => {
            console.log("socket connected")
        })

        this.socket.on('join', (room: string) => {
            console.log('Another peer made a request to join room ' + room);
            console.log('I am ' + this.peerRole + '!');
            if (this.onRobotConnectionStart) this.onRobotConnectionStart()
        });

        // This is only sent to the operator room
        this.socket.on('available robot', (robot: string) => {
            console.log('available robot: ', robot)
            this.joinRoom(robot)
        })

        this.socket.on('robot available', (available: boolean) => {
            if (available) {
                this.joinRobotRoom()
            } else {
                console.warn('no robot available')
            }
        })

        this.socket.on('bye', () => {
            console.log('Session terminated.');
            this.stop();
        })

        this.socket.on('joined', (room: String) => {
            console.log('I am ' + this.peerRole + '!');
            console.log('joined: ' + room); 
            // if (this.onRobotConnectionStart) this.onRobotConnectionStart()
        });

        this.socket.on('signalling', (message: SignallingMessage) => {
            let {candidate, sessionDescription, cameraInfo} = message;
            console.log('Received message:', message);
            if (cameraInfo) {
                if (Object.keys(cameraInfo).length === 0) {
                    console.warn("Received a camera info mapping with no entries")
                }
                this.cameraInfo = cameraInfo
            }
            if (sessionDescription?.type === 'offer' || sessionDescription?.type === 'answer') {
                if (!this.peerConnection) throw 'peerConnection is undefined';
                const readyForOffer =
                    !this.makingOffer &&
                    (this.peerConnection.signalingState === "stable" || this.isSettingRemoteAnswerPending);
                const offerCollision = sessionDescription.type === "offer" && !readyForOffer;

                this.ignoreOffer = !this.polite && offerCollision;

                if (this.ignoreOffer) {
                    console.error("Ignoring offer")
                    return;
                }
                this.isSettingRemoteAnswerPending = sessionDescription.type === "answer";

                this.peerConnection.setRemoteDescription(sessionDescription).then(async () => {
                    this.isSettingRemoteAnswerPending = false;
                    if (!this.peerConnection) throw 'peerConnection is undefined';
                    if (sessionDescription?.type === "offer") {
                        return this.peerConnection.setLocalDescription();
                    } else {
                        return false;
                    }
                }).then(() => {
                    if (!this.peerConnection?.localDescription) throw 'peerConnection is undefined';
                    this.sendSignallingMessage({sessionDescription: this.peerConnection.localDescription});
                });
            } else if (candidate !== undefined && this.peerConnection) {
                // Note that the last candidate will be null, so we check whether
                // the candidate variable is defined at all versus being truthy.
                // if (this.peerConnection.remoteDescription !== null) {
                this.peerConnection.addIceCandidate(candidate).catch(e => {
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

    /**
     * Called to create bidirectional message channels with the connected peer.
     * Note that the peer is expected to be using this class to manage their end of the connection as well,
     * in which case the necessary event handlers will be installed in `createPeerConnection` during `ondatachannel`.
     */
     openDataChannels() {
        console.log("opened data channels")
        if (!this.peerConnection) throw 'peerConnection undefined'
        this.messageChannel = this.peerConnection.createDataChannel('messages');
        this.messageChannel.onmessage = this.onReceiveMessageCallback.bind(this);
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
            
            this.peerConnection.ontrack = this.onTrackAdded!
            
            this.peerConnection.ondatachannel = event => {
                // The remote has opened a data channel. We'll set up different handlers for the different channels.
                if (event.channel.label === "messages") {
                    this.messageChannel = event.channel;
                    this.messageChannel.onmessage = this.onReceiveMessageCallback.bind(this);
                    let onDataChannelStateChange = () => {
                        if (!this.messageChannel) throw 'messageChannel is undefined';
                        const readyState = this.messageChannel.readyState;
                        console.log('Data channel state is: ' + readyState);
                        if (readyState === 'open') {
                            if (this.onMessageChannelOpen) this.onMessageChannelOpen();
                        }
                    }
                    this.messageChannel.onopen = onDataChannelStateChange;
                    this.messageChannel.onclose = onDataChannelStateChange;
                } else {
                    console.error("Unknown channel opened:", event.channel.label)
                }
            };

            this.peerConnection.onnegotiationneeded = async () => {
                console.log("Negotiation needed")
                try {
                    this.makingOffer = true;
                    if (!this.peerConnection) throw 'peerConnection is undefined';
                    await this.peerConnection.setLocalDescription();
                    if (this.peerConnection.localDescription) {
                        this.sendSignallingMessage({
                            sessionDescription: this.peerConnection.localDescription,
                            cameraInfo: this.cameraInfo
                        });
                    }
                } catch (err) {
                    console.error(err);
                } finally {
                    this.makingOffer = false;
                }
            };

            this.peerConnection.oniceconnectionstatechange = () => {
                if (!this.peerConnection) throw 'peerConnection is undefined';
                if (this.peerConnection.iceConnectionState === "failed") {
                    this.peerConnection.restartIce();
                }
            };

            this.peerConnection.onconnectionstatechange = () => {
                if (!this.peerConnection) throw 'pc is undefined';
                if (this.peerConnection.connectionState === "failed" || this.peerConnection.connectionState === "disconnected") {
                    console.error(this.peerConnection.connectionState, "Resetting the PeerConnection")
                    if (this.onConnectionEnd) this.onConnectionEnd();
                    this.createPeerConnection()
                }
                console.log(this.peerConnection.connectionState)
            };

            this.peerConnection.onicecandidateerror = (event) => {
                console.error('ICE candidate gathering error:', event.errorCode);
            };

            console.log('Created RTCPeerConnection');
        } catch (e: any) {
            console.error('Failed to create PeerConnection, exception: ' + e.message);
            return;
        }
    }
    
    connectionState() {
        return this.peerConnection?.connectionState
    }

    joinRobotRoom() {
        console.log('attempting to join room = robot');
        this.socket.emit('join', 'robot')
    }

    joinOperatorRoom() {
        console.log('attempting to join room = operator');
        this.socket.emit('join', 'operator')
        this.socket.emit('is robot available');
    }

    joinRoom(room: string) {
        console.log('attempting to join room =');
        console.log(room);
        this.socket.emit('join', room)
    }

    addTrack(track: MediaStreamTrack, stream: MediaStream, streamName: string) { 
        this.cameraInfo[stream.id] = streamName   
        if (!this.peerConnection) throw 'pc is undefined';
        this.peerConnection.addTrack(track, stream);
        console.log("added track")
    }

    hangup() {
        // Tell the other end that we're ending the call so they can stop, and get us kicked out of the robot room
        console.warn("Hanging up")
        this.socket.emit('bye', this.peerRole);
        if (!this.peerConnection) throw 'pc is undefined';
        if (this.peerConnection.connectionState === "new") {
            // Don't reset PCs that don't have any state to reset
            return;
        }
        console.log('Hanging up.');
        this.stop();
    }

    stop() {
        if (!this.peerConnection) throw 'peerConnection is undefined';
        this.removeTracks()
        this.peerConnection.close();
        this.createPeerConnection()
    }

    removeTracks() {
        if (!this.peerConnection) throw 'peerConnection is undefined';
        const senders = this.peerConnection.getSenders();
        senders.forEach((sender) => this.peerConnection?.removeTrack(sender));
    }
    
    sendSignallingMessage(message: SignallingMessage) {
        this.socket.emit('signalling', message);
    }

    ////////////////////////////////////////////////////////////
    // RTCDataChannel
    // on Sept. 15, 2017 copied initial code from
    // https://github.com/googlecodelabs/webrtc-web/blob/master/step-03/js/main.js
    // initial code licensed with Apache License 2.0
    ////////////////////////////////////////////////////////////

    sendData(obj: WebRTCMessage | WebRTCMessage[]) {
        if (!this.messageChannel || (this.messageChannel.readyState !== 'open')) {
            //console.warn("Trying to send data, but data channel isn't ready")
            return;
        }
        const data = JSON.stringify(obj);
        this.messageChannel.send(data)
    }

    onReceiveMessageCallback(event: { data: string }) {
        const obj: WebRTCMessage | WebRTCMessage[] = safelyParseJSON(event.data);
        if (this.onMessage) this.onMessage(obj)
    }
}