import { MAIN_BRANCH_LAYOUT } from "../default_layouts/main_branch";
import { STUDY_BRANCH_LAYOUT } from "../default_layouts/study_branch";
import { LayoutDefinition } from "./componentdefinitions";

import { FirebaseOptions, FirebaseError, initializeApp, FirebaseApp } from "firebase/app";
import { Auth, getAuth, User, signInWithPopup, GoogleAuthProvider, onAuthStateChanged } from 'firebase/auth'
import { Database, getDatabase, child, get, ref, update, push } from 'firebase/database'

export type DefaultLayoutName = "Study branch" | "Main branch";
export const DEFAULT_LAYOUTS: { [key in DefaultLayoutName]: LayoutDefinition } = {
    "Study branch": STUDY_BRANCH_LAYOUT,
    "Main branch": MAIN_BRANCH_LAYOUT
}

export abstract class StorageHandler {
    constructor(props: {onStorageHandlerReadyCallback: () => void}) {
        this.onReadyCallback = props.onStorageHandlerReadyCallback.bind(this)
    }

    public abstract loadCustomLayout(layoutName: string): LayoutDefinition;

    public abstract saveCustomLayout(layout: LayoutDefinition, layoutName: string): void;

    public abstract saveCurrentLayout(layout: LayoutDefinition): void;

    public abstract loadCurrentLayout(): LayoutDefinition | null;

    public onReadyCallback: () => void;

    public abstract getCustomLayoutNames(): string[];

    public loadCurrentLayoutOrDefault(): LayoutDefinition {
        const currentLayout = this.loadCurrentLayout();
        if (!currentLayout) return Object.values(DEFAULT_LAYOUTS)[0];
        console.log('loading saved layout')
        return currentLayout;
    }

    public getDefaultLayoutNames(): string[] {
        return Object.keys(DEFAULT_LAYOUTS);
    }

    public loadDefaultLayout(layoutName: DefaultLayoutName): LayoutDefinition {
        return DEFAULT_LAYOUTS[layoutName];
    }
}

export class LocalStorageHandler extends StorageHandler {
    public static CURRENT_LAYOUT_KEY = "user_custom_layout";    
    public static LAYOUT_NAMES_KEY = "user_custom_layout_names"

    constructor(props: {onStorageHandlerReadyCallback: () => void}) {
        super(props)
        // Allow the initialization process to complete before invoking the callback
        setTimeout(() => {
            this.onReadyCallback();
        }, 0);
    }

    public loadCustomLayout(layoutName: string): LayoutDefinition {
        const storedJson = localStorage.getItem(layoutName);
        if (!storedJson) throw Error(`Could not load custom layout ${layoutName}`);
        return JSON.parse(storedJson);
    }

    public saveCustomLayout(layout: LayoutDefinition, layoutName: string): void {
        const layoutNames = this.getCustomLayoutNames();
        layoutNames.push(layoutName);
        localStorage.setItem(LocalStorageHandler.LAYOUT_NAMES_KEY, JSON.stringify(layoutNames));
        localStorage.setItem(layoutName, JSON.stringify(layout));
    }

    public saveCurrentLayout(layout: LayoutDefinition): void {
        localStorage.setItem(LocalStorageHandler.CURRENT_LAYOUT_KEY, JSON.stringify(layout));
    }

    public loadCurrentLayout(): LayoutDefinition | null {
        const storedJson = localStorage.getItem(LocalStorageHandler.CURRENT_LAYOUT_KEY);
        if (!storedJson) return null;
        return JSON.parse(storedJson);
    }

    public getCustomLayoutNames(): string[] {
        const storedJson = localStorage.getItem(LocalStorageHandler.LAYOUT_NAMES_KEY);
        if (!storedJson) return [];
        return JSON.parse(storedJson);
    }
}

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

    constructor(props: {onStorageHandlerReadyCallback: () => void, config: FirebaseOptions}) {
        super(props)
        this.config = props.config
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
                console.log(userData.layouts)
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
        return Object.keys(this.layouts)
    }

    // async loadDefaultLayouts() {
    //     this.writeLayouts(JSON.parse(JSON.stringify(DEFAULT_LAYOUTS)))
    // }

    // private async writeCurrentLayout(layout: LayoutDefinition) {
    //     this.currentLayout = layout

    //     let updates: any = {};
    //     updates['/users/' + (this.uid) + '/currentLayout'] = layout;
    //     return update(ref(this.database), updates);
    // }

    private async writeLayouts(layouts: { [name: string]: LayoutDefinition }) {
        this.layouts = layouts;

        let updates: any = {};
        updates['/users/' + (this.uid) + '/layouts'] = layouts;
        return update(ref(this.database), updates);
    }
}