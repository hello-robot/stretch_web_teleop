import { LoginHandler } from "./LoginHandler";
import io, { Socket } from "socket.io-client";

export class LocalLoginHandler extends LoginHandler {
    private socket: Socket;
    private _loginState: string;

    constructor(onLoginHandlerReadyCallback: () => void) {
        super(onLoginHandlerReadyCallback);
        this._loginState = "authenticated";
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

    public listRooms(resultCallback) {
        this.socket.emit("list_rooms");
        this.socket.on("update_rooms", (ret) => {
            let robo_info = ret["robot_id"];
            robo_info["is_active"] = true;
            resultCallback("robot_id", robo_info);
        });
    }

    public logout(): Promise<undefined> {
        return new Promise<undefined>((resolve, reject) => {
            this._loginState = "not_authenticated";
            this.onReadyCallback();
            resolve(undefined);
        });
    }

    public login(
        username: string,
        password: string,
        remember_me: boolean,
    ): Promise<undefined> {
        return new Promise<undefined>((resolve, reject) => {
            this._loginState = "authenticated";
            this.onReadyCallback();
            resolve(undefined);
        });
    }

    public forgot_password(username: string): Promise<undefined> {
        return new Promise<undefined>((resolve, reject) => {
            reject(
                Error("LocalLoginHandler.forgot_password() is not implemented"),
            );
            // resolve(undefined);
        });
    }
}
