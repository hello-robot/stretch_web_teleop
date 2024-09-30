import { SignallingMessage } from "shared/util";
import { BaseSignaling, SignalingProps } from "./Signaling";
import io, { Socket } from "socket.io-client";


export class LocalSignaling extends BaseSignaling {
    private socket: Socket;    

    constructor(props: SignalingProps) {
        super(props);
        this.socket = io();
        this.socket.on("connect", () => {
            console.log("Connected to local socket");
        });
        this.socket.on("bye", () => {
            this.onGoodbye();
        });
        this.socket.on("signalling", (signal: SignallingMessage) => {
            this.onSignal(signal);
        });
    }

    public join_as_robot(): Promise<boolean> {
        return new Promise<boolean>((resolve) => {
            this.socket.emit("join_as_robot", (response) => {
                resolve(response.success);
            });
        });
    }

    public join_as_operator(): Promise<boolean> {
        return new Promise<boolean>((resolve) => {
            this.socket.emit("join_as_operator", (response) => {
                resolve(response.success);
            });
        });
    }

    public send(signal: SignallingMessage): void {
        this.socket.emit("signalling", signal);
    }
}