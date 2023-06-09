import { StorageHandler } from "./StorageHandler";
import { LayoutDefinition } from "../utils/component_definitions";

import { FirebaseOptions, FirebaseError, initializeApp, FirebaseApp } from "firebase/app";
import { Auth, getAuth, User, signInWithPopup, GoogleAuthProvider, onAuthStateChanged } from 'firebase/auth'
import { Database, getDatabase, child, get, ref, update, push } from 'firebase/database'

/** Uses Firebase to store data. */
export class FirebaseStorageHandler extends StorageHandler {
    private config: FirebaseOptions;
    private app: FirebaseApp;
    private database: Database;
    private auth: Auth;
    private GAuthProvider: GoogleAuthProvider;

    private userEmail: string;
    private uid: string;
    private layouts: { [name: string]: LayoutDefinition };
    private currentLayout: LayoutDefinition | null;

    constructor(onStorageHandlerReadyCallback: () => void, config: FirebaseOptions) {
        super(onStorageHandlerReadyCallback);
        this.config = config;
        this.app = initializeApp(this.config);
        this.database = getDatabase(this.app);
        this.auth = getAuth(this.app);
        this.GAuthProvider = new GoogleAuthProvider();

        this.userEmail = ""
        this.uid = ""
        this.layouts = {};
        this.currentLayout = null;
        onAuthStateChanged(this.auth, (user) => this.handleAuthStateChange(user));

        this.signInWithGoogle()
    }

    private handleAuthStateChange(user: User | null) {
        if (user) {
            this.uid = user.uid;
            this.userEmail = user.email!;

            this.getUserDataFirebase().then(async (userData) => {
                this.layouts = userData.layouts;
                this.currentLayout = userData.currentLayout
                console.log(userData.currentLayout)
                this.onReadyCallback()
            }).catch((error) => {
                console.log("Detected that FirebaseModel isn't initialized for user ", this.uid);
                this.onReadyCallback()
            })
        }
    }

    private async getUserDataFirebase() {
        const snapshot = await get(child(ref(this.database), '/users/' + (this.uid)))

        if (snapshot.exists()) {
            return snapshot.val();
        } else {
            throw "No data available";
        }
    }

    public signInWithGoogle() {
        if (this.userEmail == "") {
            signInWithPopup(this.auth, this.GAuthProvider)
                .then((result) => {
                    const credential = GoogleAuthProvider.credentialFromResult(result);
                    const token = credential!.accessToken;
                    const user = result.user;
                    return Promise.resolve()
                })
                .catch(this.handleError);
        }
    }

    private handleError(error: FirebaseError) {
        const errorCode = error.code;
        const errorMessage = error.message;
        console.error("firebaseError: " + errorCode + ": " + errorMessage);
        console.trace();
        return Promise.reject()
    }

    public loadCustomLayout(layoutName: string): LayoutDefinition {
        let layout = this.layouts![layoutName]
        if (!layout) throw Error(`Could not load custom layout ${layoutName}`);
        return JSON.parse(JSON.stringify(layout));
    }

    public saveCustomLayout(layout: LayoutDefinition, layoutName: string): void {
        this.layouts[layoutName] = layout
        this.writeLayouts(this.layouts)
    }

    public saveCurrentLayout(layout: LayoutDefinition): void {
        this.currentLayout = layout

        let updates: any = {};
        updates['/users/' + (this.uid) + '/currentLayout'] = layout;
        update(ref(this.database), updates);
    }
    
    public loadCurrentLayout(): LayoutDefinition | null {
        return this.currentLayout
    }

    public getCustomLayoutNames(): string[] {
        if (!this.layouts) return []
        return Object.keys(this.layouts)
    }

    private async writeLayouts(layouts: { [name: string]: LayoutDefinition }) {
        this.layouts = layouts;

        let updates: any = {};
        updates['/users/' + (this.uid) + '/layouts'] = layouts;
        return update(ref(this.database), updates);
    }
}