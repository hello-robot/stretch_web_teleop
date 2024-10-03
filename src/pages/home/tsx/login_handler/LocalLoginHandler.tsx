import { LoginHandler } from "./LoginHandler";
import io, { Socket } from "socket.io-client";


export class LocalLoginHandler extends LoginHandler {
    private socket: Socket;

    constructor(onLoginHandlerReadyCallback: () => void) {
        super(onLoginHandlerReadyCallback);
        this.socket = io();
        this.socket.on("connect", () => {
            console.log("Connected to local socket");
        });

        // Allow the initialization process to complete before invoking the callback
        setTimeout(() => {
            this.onReadyCallback();
        }, 0);
    }

    public loginState(): string {
        return "authenticated";
    }
}
