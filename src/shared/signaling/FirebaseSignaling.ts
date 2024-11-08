import { SignallingMessage } from "shared/util";
import { BaseSignaling, SignalingProps } from "./Signaling";
import { initializeApp, FirebaseOptions } from "firebase/app";
import {
    getAuth,
    onAuthStateChanged,
    Auth,
} from "firebase/auth";
import { getDatabase, ref, onValue, get, set, update, Database } from "firebase/database";

// TODO: use lodash isequal
function isEqual(obj1, obj2) {
    if (!isObject(obj1) && !isObject(obj2)) {
        return obj1 === obj2;
    }
    var props1 = Object.getOwnPropertyNames(obj1);
    var props2 = Object.getOwnPropertyNames(obj2);
    if (props1.length != props2.length) {
        return false;
    }
    for (var i = 0; i < props1.length; i++) {
        let val1 = obj1[props1[i]];
        let val2 = obj2[props1[i]];
        let isObjects = isObject(val1) && isObject(val2);
        if (isObjects && !isEqual(val1, val2) || !isObjects && val1 !== val2) {
            return false;
        }
    }
    return true;
}
function isObject(object) {
  return object != null && typeof object === 'object';
}


export class FirebaseSignaling extends BaseSignaling {
    private auth: Auth;
    private _loginState: string;
    private db: Database;
    private uid: string;
    private role: string;
    private prevSignal;
    private room_uid: string;
    private is_joined: boolean;

    constructor(props: SignalingProps, config: FirebaseOptions,) {
        super(props);
        this._loginState = "not_authenticated";
        const app = initializeApp(config);
        this.auth = getAuth(app);
        this.db = getDatabase(app);
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
                        })
                        .catch((error) => {
                            // We can ignore the robots the operator cannot access
                            // console.error(error.message, "Cannot access: ", "robots/" + robo_uid);
                        });
                    });
                });
            }
        });
    }

    public configure(room_name: string): Promise<void> {
        return new Promise<void>((resolve) => {
            // wait to be authenticated
            onAuthStateChanged(this.auth, (user) => {
                this.uid = user ? user.uid : undefined;
                this._loginState = user ? "authenticated" : "not_authenticated";

                if (this._loginState === "authenticated") {
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
                                if (this.is_joined) {
                                    // Filter out what's changed
                                    let currSignal = snapshot.val();
                                    let changes = {};
                                    for (const key in currSignal) {
                                        if (!this.prevSignal || !(key in this.prevSignal) || !isEqual(currSignal[key], this.prevSignal[key])) {
                                            changes[key] = currSignal[key];
                                        }
                                    }
                                    this.prevSignal = currSignal;

                                    // Trigger callbacks based on what's changed
                                    if (Object.keys(changes).includes("candidate") ||
                                        Object.keys(changes).includes("sessionDescription") ||
                                        Object.keys(changes).includes("cameraInfo")) {
                                        if (Object.keys(changes).includes("active")) {
                                            delete changes["active"];
                                        }
                                        this.onSignal(changes);
                                    }
                                    if (Object.keys(changes).includes("active") && !changes["active"]) {
                                        console.log("bye");
                                        if (this.role === "robot") {
                                            update(ref(this.db, "robots/" + this.uid), {
                                                status: "online",
                                            });
                                        }
                                        this.onGoodbye();
                                    }
                                    if (this.role === "robot" && Object.keys(changes).includes("active") && changes["active"]) {
                                        console.log(`Operator has joined the room. My role: ${this.role}.`);
                                        update(ref(this.db, "robots/" + this.uid), {
                                            status: "occupied",
                                        });
                                        if (this.onRobotConnectionStart) this.onRobotConnectionStart();
                                    }
                                }
                            });

                            resolve();
                        });
                    });
                }
            });
        });
    }

    public join_as_robot(): Promise<boolean> {
        return new Promise<boolean>((resolve) => {
            get(ref(this.db, "rooms/" + this.room_uid + "/" + this.role + "/active")).then((snapshot) => {
                let is_active = snapshot.val();
                if (false) { // TODO: onwindowunload is flaky. is_active might stay true when the robot browser exits. For now, let's ignore if firebase says theres already a robot in the room.
                    console.log("Another robot is already active");
                    resolve(false);
                } else {
                    set(ref(this.db, "rooms/" + this.room_uid + "/" + this.role), {
                        active: true
                    }).then(() => {
                        this.is_joined = true;
                        update(ref(this.db, "robots/" + this.uid), {
                            status: "online",
                        });
                        resolve(true)
                    });
                }
            });
        });
    }

    public join_as_operator(): Promise<boolean> {
        return new Promise<boolean>((resolve) => {
            let opposite_role = this.role === "robot" ? "operator" : "robot";
            get(ref(this.db, "rooms/" + this.room_uid + "/" + opposite_role + "/active")).then((snapshot) => {
                let is_robot_active = snapshot.val();
                if (!is_robot_active) {
                    console.log("Robot is not active");
                    resolve(false);
                } else {
                    get(ref(this.db, "rooms/" + this.room_uid + "/" + this.role + "/active")).then((snapshot2) => {
                        let is_operator_active = snapshot2.val();
                        if (is_operator_active) {
                            console.log("Another operator is already active");
                            resolve(false);
                        } else {
                            set(ref(this.db, "rooms/" + this.room_uid + "/" + this.role), {
                                active: true
                            }).then(() => {
                                this.is_joined = true;
                                resolve(true)
                            });
                        }
                    });
                }
            });
        });
    }

    public leave(): void {
        if (this.is_joined) {
            this.is_joined = false;
            console.log(`Leaving. My role: ${this.role}.`);
            set(ref(this.db, "rooms/" + this.room_uid + "/" + this.role), {
                active: false
            });
            if (this.role === "robot") {
                update(ref(this.db, "robots/" + this.uid), {
                    status: "offline",
                });
            }
        }
    }

    public send(signal: SignallingMessage): void {
        if (this.is_joined) {
            update(ref(this.db, "rooms/" + this.room_uid + "/" + this.role), signal);
        }
    }
}
