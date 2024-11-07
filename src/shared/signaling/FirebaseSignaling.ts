import { SignallingMessage } from "shared/util";
import { BaseSignaling, SignalingProps } from "./Signaling";
import { initializeApp, FirebaseOptions } from "firebase/app";
import {
    getAuth,
    onAuthStateChanged,
    Auth,
} from "firebase/auth";
import { getDatabase, ref, onValue, get, update, Database } from "firebase/database";


export class FirebaseSignaling extends BaseSignaling {
    private auth: Auth;
    private _loginState: string;
    private db: Database;
    private uid: string;
    private role: string;
    private prevSignal;
    private room_uid: string;

    constructor(props: SignalingProps, config: FirebaseOptions,) {
        super(props);
        this._loginState = "not_authenticated";
        this.prevSignal = {
            active: undefined,
            candidate: undefined,
            sessionDescription: undefined,
            cameraInfo: undefined
        };
        const app = initializeApp(config);
        this.auth = getAuth(app);
        this.db = getDatabase(app);

        onAuthStateChanged(this.auth, (user) => {
            this.uid = user ? user.uid : undefined;
            this._loginState = user ? "authenticated" : "not_authenticated";
        });
    }

    private _get_room_uid(room_name: string): Promise<string> {
        return new Promise<string>((resolve) => {
            if (this.role === "robot") {
                resolve(this.uid);
            } else if (this.role === "operator") {
                get(ref(this.db, "assignments/" + this.uid + "/robots")).then((snapshot) => {
                    let robots = snapshot.val();
                    Object.entries(robots).forEach(([robo_uid, is_active]) => {
                        get(ref(this.db, "robots/" + robo_uid)).then((snapshot2) => {
                            let robo_info = snapshot2.val();
                            if (robo_info["name"] === room_name) {
                                resolve(robo_uid);
                            }
                        });
                    });
                });
            }
        });
    }

    public configure(room_name: string): Promise<void> {
        return new Promise<void>((resolve) => {
            // wait to be authenticated
            while (this._loginState !== "authenticated") {}

            // get my role + set up listeners
            get(ref(this.db, "assignments/" + this.uid + "/role")).then((snapshot) => {
                this.role = snapshot.val();
                console.log(`My role: ${this.role}`);
                if (!["robot", "operator"].includes(this.role)) {
                    console.error("ERROR: invalid role");
                    throw new Error("Invalid role");
                }

                this._get_room_uid(room_name).then((room_uid) => {
                    this.room_uid = room_uid;
                    let opposite_role = this.role === "robot" ? "operator" : "robot";
                    onValue(ref(this.db, "rooms/" + this.room_uid + "/" + opposite_role), (snapshot) => {
                        let currSignal = snapshot.val();
                        let changes = {};
                        for (const key in currSignal) {
                            if (currSignal[key] !== this.prevSignal[key]) {
                                changes[key] = currSignal[key];
                            }
                        }
                        this.prevSignal = currSignal;

                        console.log("changes", changes);
                        if (!Object.keys(changes).includes("active") &&
                                (Object.keys(changes).includes("candidate") ||
                                 Object.keys(changes).includes("sessionDescription") ||
                                 Object.keys(changes).includes("cameraInfo"))) {
                                    this.onSignal(changes);
                        }
                        if (Object.keys(changes).includes("active") && !changes["active"]) {
                            console.log("bye");
                            update(ref(this.db, "robots/" + this.uid), {
                                status: "online",
                            });
                            this.onGoodbye();
                        }
                        if (this.role === "robot" && Object.keys(changes).includes("active") && changes["active"]) {
                            console.log(`Operator has joined the room. My role: ${this.role}.`);
                            update(ref(this.db, "robots/" + this.uid), {
                                status: "occupied",
                            });
                            if (this.onRobotConnectionStart) this.onRobotConnectionStart();
                        }
                    });

                    resolve();
                });
            });
        });
    }

    public join_as_robot(): Promise<boolean> {
        return new Promise<boolean>((resolve) => {
            // TODO: check that not already active before entering room
            update(ref(this.db, "rooms/" + this.room_uid + "/" + this.role), {
                active: true
            }).then(() => {
                update(ref(this.db, "robots/" + this.uid), {
                    status: "online",
                });
                resolve(true)
            });
        });
    }

    public join_as_operator(): Promise<boolean> {
        return new Promise<boolean>((resolve) => {
            // TODO: check that not already active before entering room
            update(ref(this.db, "rooms/" + this.room_uid + "/" + this.role), {
                active: true
            }).then(() => {
                resolve(true)
            });
        });
    }

    public leave(): void {
        console.log(`Leaving. My role: ${this.role}.`);
        // TODO: check that am in room, before exiting it
        update(ref(this.db, "rooms/" + this.room_uid + "/" + this.role), {
            active: false
        });
    }

    public send(signal: SignallingMessage): void {
        update(ref(this.db, "rooms/" + this.room_uid + "/" + this.role), signal);
    }
}
