import { LoginHandler } from "./LoginHandler";
import { initializeApp, FirebaseOptions } from "firebase/app";
import { getAuth, signInWithEmailAndPassword, onAuthStateChanged, sendPasswordResetEmail, signOut, setPersistence, browserLocalPersistence, browserSessionPersistence, Auth } from "firebase/auth";
import { getDatabase, ref, onValue, Database } from "firebase/database";


export class FirebaseLoginHandler extends LoginHandler {
    private auth: Auth;
    private _loginState: string;
    private db: Database;
    private uid: string;

    constructor(onLoginHandlerReadyCallback: () => void, config: FirebaseOptions) {
        super(onLoginHandlerReadyCallback);
        this._loginState = "not_authenticated";
        const app = initializeApp(config);
        this.auth = getAuth(app);
        this.db = getDatabase(app);

        onAuthStateChanged(this.auth, (user) => {
            this.uid = user ? user.uid : undefined;
            this._loginState = user ? "authenticated" : "not_authenticated";
            this.onReadyCallback();
        });
    }

    public loginState(): string {
        return this._loginState;
    }

    public listRooms(resultCallback) {
        if (this.uid === undefined) {
            return; // TODO(binit): throw an exception? reject a promise?
        }

        onValue(ref(this.db, 'rooms/' + this.uid + '/robots'), (snapshot) => {
            resultCallback(snapshot.val());
        });
    }

    public logout(): Promise<undefined> {
        // Tutorial here:
        // https://firebase.google.com/docs/auth/web/password-auth#next_steps

        return new Promise<undefined>((resolve, reject) => {
            signOut(this.auth)
                .then(() => { resolve(undefined) })
                .catch(reject);
        });
    }

    public login(username: string, password: string, remember_me: boolean): Promise<undefined> {
        // Tutorial here:
        // https://firebase.google.com/docs/auth/web/start?hl=en#sign_in_existing_users
        // Auth State Persistence tutorial here:
        // https://firebase.google.com/docs/auth/web/auth-state-persistence

        return new Promise<undefined>((resolve, reject) => {
            setPersistence(this.auth, remember_me ? browserLocalPersistence : browserSessionPersistence)
                .then(() => {
                    signInWithEmailAndPassword(this.auth, username, password)
                        .then((userCredential) => {
                            resolve(undefined);
                        })
                        .catch(reject);
                })
                .catch(reject);
        });
    }

    public forgot_password(username: string): Promise<undefined> {
        // Tutorial here:
        // https://firebase.google.com/docs/auth/web/manage-users?hl=en#send_a_password_reset_email

        return new Promise<undefined>((resolve, reject) => {
            sendPasswordResetEmail(this.auth, username)
                .then(() => { resolve(undefined) })
                .catch(reject);
        });
    }
}
