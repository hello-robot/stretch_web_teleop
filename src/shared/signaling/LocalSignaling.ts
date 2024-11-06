import { SignallingMessage } from "shared/util";
import { BaseSignaling, SignalingProps } from "./Signaling";
import io, { Socket } from "socket.io-client";


export class LocalSignaling extends BaseSignaling {
    private socket: Socket;
    private role: string;

    constructor(props: SignalingProps) {
        super(props);
        this.socket = io();
        this.socket.on("connect", () => {
            console.log("Connected to local socket");
        });
        this.socket.on("signalling", (signal: SignallingMessage) => {
            this.onSignal(signal);
        });
        this.socket.on("bye", () => {
            this.onGoodbye();
        });
        this.socket.on("joined", () => {
            console.log(`Operator has joined the room. My role: ${this.role}.`);
            if (this.onRobotConnectionStart) this.onRobotConnectionStart();
        });
    }

    public configure(): Promise<void> {
        return new Promise<void>((resolve) => {
            resolve();
        });
    }

    public join_as_robot(): Promise<boolean> {
        return new Promise<boolean>((resolve) => {
            this.socket.emit("join_as_robot", (response) => {
                if (response.success) {
                    this.role = "robot";
                }
                resolve(response.success);
            });
        });
    }

    public join_as_operator(): Promise<boolean> {
        return new Promise<boolean>((resolve) => {
            this.socket.emit("join_as_operator", (response) => {
                if (response.success) {
                    this.role = "operator";
                }
                resolve(response.success);
            });
        });
    }

    public leave(): void {
        console.log(`Leaving. My role: ${this.role}.`);
        this.socket.emit("bye", this.role);
    }

    public send(signal: SignallingMessage): void {
        this.socket.emit("signalling", signal);
    }
}
