import { MAIN_BRANCH_LAYOUT } from "../default_layouts/main_branch";
import { STUDY_BRANCH_LAYOUT } from "../default_layouts/study_branch";
import { LayoutDefinition } from "./componentdefinitions";

import { FirebaseOptions, FirebaseError, initializeApp, FirebaseApp  } from "firebase/app";
import { Auth, getAuth, User, signInWithPopup, GoogleAuthProvider, onAuthStateChanged } from 'firebase/auth'
import { Database, getDatabase, child, get, ref, update, push } from 'firebase/database'

export type DefaultLayoutName = "Study branch" | "Main branch";
export const DEFAULT_LAYOUTS: { [key in DefaultLayoutName]: LayoutDefinition } = {
    "Study branch": STUDY_BRANCH_LAYOUT,
    "Main branch": MAIN_BRANCH_LAYOUT
}

const USER_LAYOUT_SAVE_KEY = "user_custom_layout";

export abstract class StorageHandler {

    public abstract loadLayout(layoutName?: string): LayoutDefinition;

    public abstract saveLayout(layout: LayoutDefinition, layoutName?: string): void;
}

export class LocalStorageHandler extends StorageHandler {
    public loadLayout(layoutName?: string): LayoutDefinition {
        console.log('loading layout' + layoutName)
        if (!layoutName) {
            const storedJson = localStorage.getItem(USER_LAYOUT_SAVE_KEY);
            if (storedJson) {
                console.log('loaded stored layout');
                console.log(storedJson);
                return JSON.parse(storedJson);
            }
            return STUDY_BRANCH_LAYOUT;
        }
        if (!(layoutName in DEFAULT_LAYOUTS))
            throw Error(`could not find layout name ${DEFAULT_LAYOUTS}`);
        console.log('returning default layout')
        return DEFAULT_LAYOUTS[layoutName as DefaultLayoutName]!;
    }

    public saveLayout(layout: LayoutDefinition, layoutName?: string): void {
        localStorage.setItem(layoutName || USER_LAYOUT_SAVE_KEY, JSON.stringify(layout));
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
    private onReadyCallback: () => void

    constructor(props: {config: FirebaseOptions, onStorageHandlerReadyCallback: () => void}) {
        super()
        this.config = props.config
        this.app = initializeApp(this.config);
		this.database = getDatabase(this.app);
		this.auth = getAuth(this.app);
        this.GAuthProvider = new GoogleAuthProvider();

        this.userEmail = ""
        this.uid = ""
        this.layouts = {};
        this.currentLayout = STUDY_BRANCH_LAYOUT
        this.onReadyCallback = props.onStorageHandlerReadyCallback.bind(this)
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

    public loadLayout(layoutName?: string): LayoutDefinition {
        if (!layoutName) {
            this.writeCurrentLayout(this.layouts!["Study branch"])
            return JSON.parse(JSON.stringify(this.layouts!["Study branch"]))
        }
        this.writeCurrentLayout(this.layouts![layoutName])
        return JSON.parse(JSON.stringify(this.layouts![layoutName]));
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