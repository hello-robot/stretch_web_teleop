import { LoginHandler } from "./LoginHandler";
import io, { Socket } from "socket.io-client";


export class LocalLoginHandler extends LoginHandler {
    private socket: Socket;
    private _loginState: string;

    constructor(onLoginHandlerReadyCallback: () => void) {
        super(onLoginHandlerReadyCallback);
        this._loginState = "not_authenticated";
        this.socket = io();
        this.socket.on("connect", () => {
            console.log("Connected to local socket");
        });

        this.logout = this.logout.bind(this);
        // Allow the initialization process to complete before invoking the callback
        setTimeout(() => {
            this.onReadyCallback();
        }, 0);
    }

    public loginState(): string {
        return this._loginState;
    }

    public listRooms(): Promise<boolean> {
        return new Promise<boolean>((resolve) => {
            this.socket.emit("list_rooms", (rooms) => {
                resolve(rooms);
            });
        });
    }

    public logout() {
        this._loginState = "not_authenticated";
        this.onReadyCallback();
    }

    public login(username: string, password: string, remember_me: boolean) {
        this._loginState = "authenticated";
        this.onReadyCallback();
    }
}
