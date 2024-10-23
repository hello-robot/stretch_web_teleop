import { LoginHandler } from "./LoginHandler";
import { initializeApp, FirebaseOptions } from "firebase/app";
import { getAuth, signInWithEmailAndPassword, onAuthStateChanged, sendPasswordResetEmail, signOut, setPersistence, browserLocalPersistence, browserSessionPersistence, Auth } from "firebase/auth";


export class FirebaseLoginHandler extends LoginHandler {
    private auth: Auth;
    private _loginState: string;

    constructor(onLoginHandlerReadyCallback: () => void, config: FirebaseOptions) {
        super(onLoginHandlerReadyCallback);
        this._loginState = "not_authenticated";
        const app = initializeApp(config);
        this.auth = getAuth(app);

        onAuthStateChanged(this.auth, (user) => {
            this._loginState = user ? "authenticated" : "not_authenticated";
            this.onReadyCallback();
        });
    }

    public loginState(): string {
        return this._loginState;
    }

    public listRooms(resultCallback) {
        // TODO(binit)
    }

    public logout() {
        // Tutorial here:
        // https://firebase.google.com/docs/auth/web/password-auth#next_steps

        signOut(this.auth)
            .then(() => {
                // signed out succcessfully TODO
            })
            .catch((error) => {
                // TODO
            });
    }

    public login(username: string, password: string, remember_me: boolean) {
        // Tutorial here:
        // https://firebase.google.com/docs/auth/web/start?hl=en#sign_in_existing_users
        // Auth State Persistence tutorial here:
        // https://firebase.google.com/docs/auth/web/auth-state-persistence

        setPersistence(this.auth, remember_me ? browserLocalPersistence : browserSessionPersistence)
            .then(() => {
                signInWithEmailAndPassword(this.auth, username, password)
                    .then((userCredential) => {
                        // TODO
                    })
                    .catch((sign_in_error) => {
                        // TODO
                    });
            })
            .catch((set_persistence_error) => {
                // TODO
            });
    }

    public forgot_password(username: string): Promise<undefined> {
        // Tutorial here:
        // https://firebase.google.com/docs/auth/web/manage-users?hl=en#send_a_password_reset_email

        return new Promise<undefined>((resolve, reject) => {
            sendPasswordResetEmail(this.auth, username)
                .catch(reject);
        });
    }
}
