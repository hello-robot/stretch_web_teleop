import { StorageHandler } from "./StorageHandler";
import { LayoutDefinition } from "../utils/component_definitions";

import { FirebaseOptions, FirebaseError, initializeApp, FirebaseApp } from "firebase/app";
import { Auth, getAuth, User, signInWithPopup, GoogleAuthProvider, onAuthStateChanged } from 'firebase/auth'
import { Database, getDatabase, child, get, ref, update, push } from 'firebase/database'
import { RobotPose } from "shared/util";
import ROSLIB from "roslib";

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
    private poses: { [name: string]: RobotPose };
    private mapPoses: { [name: string]: ROSLIB.Transform };
    private recordings: { [name: string]: RobotPose[] };

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
        this.poses = {};
        this.mapPoses = {}
        this.recordings = {}
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

    public getPoseNames(): string[] {
        if (!this.poses) return []
        return Object.keys(this.poses)
    }

    public savePose(name: string, jointState: RobotPose) {
        this.poses[name] = jointState
        this.writePoses(this.poses)
    }

    public getPose(poseName: string): RobotPose {
        let pose = this.poses![poseName]
        if (!pose) throw Error(`Could not load pose ${poseName}`);
        return JSON.parse(JSON.stringify(pose));
    }

    public deletePose(poseName: string): void {
        let pose = this.poses![poseName]
        if (!pose) throw Error(`Could not delete pose ${poseName}`);
        delete this.poses[poseName]
        this.writePoses(this.poses)
    }

    private async writePoses(poses: { [name: string]: RobotPose }) {
        this.poses = poses;

        let updates: any = {};
        updates['/users/' + (this.uid) + '/poses'] = poses;
        return update(ref(this.database), updates);
    }

    public getMapPoseNames(): string[] {
        if (!this.mapPoses) return []
        return Object.keys(this.poses)
    }

    public saveMapPose(name: string, pose: ROSLIB.Transform) {
        this.mapPoses[name] = pose
        this.writeMapPoses(this.mapPoses)
    }

    private async writeMapPoses(poses: { [name: string]: ROSLIB.Transform }) {
        this.mapPoses = poses;

        let updates: any = {};
        updates['/users/' + (this.uid) + '/map_poses'] = poses;
        return update(ref(this.database), updates);
    }

    public getMapPose(poseName: string): ROSLIB.Transform {
        let pose = this.mapPoses![poseName]
        if (!pose) throw Error(`Could not load pose ${poseName}`);
        return JSON.parse(JSON.stringify(pose));
    }

    public getMapPoses(): ROSLIB.Transform[] {
        const poseNames = this.getMapPoseNames()
        var poses: ROSLIB.Transform[] = []
        poseNames.forEach(poseName => {
            const pose = this.getMapPose(poseName)
            poses.push(pose)
        })
        return poses
    }

    public deleteMapPose(poseName: string): void {
        let pose = this.mapPoses![poseName]
        if (!pose) throw Error(`Could not delete pose ${poseName}`);
        delete this.mapPoses[poseName]
        this.writeMapPoses(this.mapPoses)
    }

    public getRecordingNames(): string[] {
        if (!this.recordings) return []
        return Object.keys(this.recordings)
    }

    public getRecording(recordingName: string): RobotPose[] {
        let recording = this.recordings![recordingName]
        if (!recording) throw Error(`Could not load recording ${recordingName}`);
        return JSON.parse(JSON.stringify(recording));
    }

    public savePoseRecording(recordingName: string, poses: RobotPose[]): void {
        this.recordings[recordingName] = poses
        this.writeRecordings(this.recordings)
    }

    private async writeRecordings(recordings: { [name: string]: RobotPose[] }) {
        this.recordings = recordings;

        let updates: any = {};
        updates['/users/' + (this.uid) + '/recordings'] = recordings;
        return update(ref(this.database), updates);
    }

    public deleteRecording(recordingName: string): void {
        let recording = this.recordings![recordingName]
        if (!recording) throw Error(`Could not delete recording ${recordingName}`);
        delete this.recordings[recordingName]
        this.writeRecordings(this.recordings)
    }
}