import { SignallingMessage } from "shared/util";
import { BaseSignaling, SignalingProps } from "./Signaling";
import { initializeApp, FirebaseOptions } from "firebase/app";
import {
    getAuth,
    signInWithEmailAndPassword,
    onAuthStateChanged,
    sendPasswordResetEmail,
    signOut,
    setPersistence,
    browserLocalPersistence,
    browserSessionPersistence,
    Auth,
} from "firebase/auth";
import { getDatabase, ref, onValue, Database } from "firebase/database";
import io, { Socket } from "socket.io-client";


export class FirebaseSignaling extends BaseSignaling {
    private auth: Auth;
    private _loginState: string;
    private db: Database;
    private uid: string;
    private socket: Socket;
    private role: string;

    constructor(props: SignalingProps, config: FirebaseOptions,) {
        super(props);
        this._loginState = "not_authenticated";
        const app = initializeApp(config);
        this.auth = getAuth(app);
        this.db = getDatabase(app);

        onAuthStateChanged(this.auth, (user) => {
            this.uid = user ? user.uid : undefined;
            this._loginState = user ? "authenticated" : "not_authenticated";
        });

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
            while (this._loginState !== "authenticated") {}
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
