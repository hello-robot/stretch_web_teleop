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
    private static CURRENT_LAYOUT_KEY = "user_custom_layout";
    private static LAYOUT_NAMES_KEY = "user_custom_layout_names"

    constructor(props: {onStorageHandlerReadyCallback: () => void}) {
        super(props)
        this.onReadyCallback()
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
    private currentLayout: LayoutDefinition;

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
        this.currentLayout = STUDY_BRANCH_LAYOUT
        onAuthStateChanged(this.auth, (user) => this.handleAuthStateChange(user));
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
                console.trace(error)
                console.warn("Detected that FirebaseModel isn't initialized. Loading default layouts.");
                this.loadDefaultLayouts();
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
        this.writeCurrentLayout(layout)
        return JSON.parse(JSON.stringify(layout));
    }

    public saveLayout(layout: LayoutDefinition, layoutName: string): void {
        this.layouts[layoutName] = layout
        this.writeLayouts(this.layouts)
    }

    async loadDefaultLayouts() {
        this.writeLayouts(JSON.parse(JSON.stringify(DEFAULT_LAYOUTS)))
        return this.writeCurrentLayout(STUDY_BRANCH_LAYOUT)
    }

    private async writeCurrentLayout(layout: LayoutDefinition) {
        this.currentLayout = layout

        let updates: any = {};
        updates['/users/' + (this.uid) + '/currentLayout'] = layout;
        return update(ref(this.database), updates);
    }

    private async writeLayouts(layouts: { [name: string]: LayoutDefinition }) {
        this.layouts = layouts;

        let updates: any = {};
        updates['/users/' + (this.uid) + '/layouts'] = layouts;
        return update(ref(this.database), updates);
    }
}