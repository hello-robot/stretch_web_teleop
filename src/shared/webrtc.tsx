import React from 'react'
import io, { Socket } from 'socket.io-client';

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
    private socket: Socket;
    private peers = {}
    private pendingCandidates = {}
    private peerConnection?: RTCPeerConnection
    private onTrackAdded?: (ev: RTCTrackEvent) => void

    constructor(props) {
        super(props);
        this.onTrackAdded = props.onTrackAdded

        this.socket = io('http://localhost:5000', {transports: ["websocket"]});

        this.socket.on('connect', () => {
            console.log("socket connected")
        });

        this.socket.on('data', (data) => {
            console.log('Data received: ', data);
            this.handleSignalingData(data);
        });
        
        this.socket.on('ready', (msg) => {
            console.log('Ready');
            // Connection with signaling server is ready, and so is local stream
            this.peers[msg.sid] = this.createPeerConnection();
            
            console.log('Send offer');
            this.peers[msg.sid].createOffer().then(
                (sdp) => setAndSendLocalDescription(msg.sid, sdp),
                (error) => {
                    console.error('Send offer failed: ', error);
                }
            );

            this.addPendingCandidates(msg.sid);
        });
    }

    createPeerConnection() {
        this.peerConnection = new RTCPeerConnection( peerConstraints );
        this.peerConnection.onicecandidate = (event) => {
            if (event.candidate) {
                console.log('ICE candidate');
                this.sendData({
                    type: 'candidate',
                    candidate: event.candidate
                });
            }
        };

        this.peerConnection.ontrack = this.onTrackAdded!
    }

    addPendingCandidates(sid) {
        if (sid in this.pendingCandidates) {
            this.pendingCandidates[sid].forEach(candidate => {
                this.peers[sid].addIceCandidate(new RTCIceCandidate(candidate))
            });
        }
    }

    sendAnswer(sid) {
        console.log('Send answer');
        this.peers[sid].createAnswer().then(
            (sdp) => this.setAndSendLocalDescription(sid, sdp),
            (error) => {
                console.error('Send answer failed: ', error);
            }
        );
    };

    setAndSendLocalDescription(sid, sessionDescription) {
        this.peers[sid].setLocalDescription(sessionDescription);
        console.log('Local description set');
        this.sendData({sid, type: sessionDescription.type, sdp: sessionDescription.sdp});
    };

    sendData(data) {
        this.socket.emit('data', data);
    };

    handleSignalingData(data) {
        // let msg = JSON.parse(data);
        console.log(data)
        const sid = data.sid;
        delete data.sid;
        switch (data.type) {
            case 'offer':
                this.peers[sid] = this.createPeerConnection();
                this.peers[sid].setRemoteDescription(new RTCSessionDescription(data));
                this.sendAnswer(sid);
                this.addPendingCandidates(sid);
                break;
            case 'answer':
                this.peers[sid].setRemoteDescription(new RTCSessionDescription(data));
                break;
            case 'candidate':
                if (sid in this.peers) {
                    this.peers[sid].addIceCandidate(new RTCIceCandidate(data.candidate));
                } else {
                    if (!(sid in this.pendingCandidates)) {
                        this.pendingCandidates[sid] = [];
                    }
                    this.pendingCandidates[sid].push(data.candidate)
                }
                break;
        }
    };
}