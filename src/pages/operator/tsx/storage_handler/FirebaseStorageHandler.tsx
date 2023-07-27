import { StorageHandler } from "./StorageHandler";
import { LayoutDefinition } from "../utils/component_definitions";

import { FirebaseOptions, FirebaseError, initializeApp, FirebaseApp } from "firebase/app";
import { Auth, getAuth, User, signInWithPopup, GoogleAuthProvider, onAuthStateChanged } from 'firebase/auth'
import { Database, getDatabase, child, get, ref, update, push } from 'firebase/database'
import { ArucoMarkersInfo, RobotPose } from "shared/util";
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
    private markerNames: string[];
    private markerIDs: string[];
    private markerInfo: ArucoMarkersInfo;

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
        this.markerNames = []
        this.markerIDs = []
        this.markerInfo = {} as ArucoMarkersInfo
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
                this.poses = userData.poses
                this.mapPoses = userData.map_poses
                this.recordings = userData.recordings
                this.markerNames = userData.marker_names
                this.markerIDs = userData.marker_ids
                this.markerInfo = userData.marker_info

                if (!this.markerNames) {
                    this.markerNames = this.loadDefaultArucoMarkerNames()
                    this.markerIDs = this.loadDefaultArucoMarkerIDs()
                    this.markerInfo = this.loadDefaultArucoMarkers()
                    this.writeMarkers()
                }
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

    public saveMarker(markerID: string, markerName: string): void {
        this.markerNames.push(markerName)
        this.markerIDs.push(markerID)
        this.markerInfo.aruco_marker_info[markerID] = {
            length_mm: 47,
            use_rgb_only: false,
            name: markerName,
            link: null
        }
        this.writeMarkers()
    }

    public deleteMarker(markerName: string): void {
        if (!this.markerNames.includes(markerName)) throw Error(`Could not delete marker ${markerName}`);
        let index = this.markerNames.indexOf(markerName)
        let markerID = this.markerIDs![index]
        if (!this.markerIDs.includes(markerID)) throw Error(`Could not delete marker ${markerName}`);
        
        delete this.markerNames[index]
        delete this.markerIDs[index]
        delete this.markerInfo.aruco_marker_info[markerID]
        this.writeMarkers()
    }

    public getArucoMarkerNames(): string[] {
        return JSON.parse(JSON.stringify(this.markerNames));
    }

    public getArucoMarkerIDs(): string[] {
        return JSON.parse(JSON.stringify(this.markerIDs));
    }

    public getArucoMarkerInfo(): ArucoMarkersInfo {
        return JSON.parse(JSON.stringify(this.markerInfo));
    }

    public saveRelativePose(markerID: string, pose: ROSLIB.Transform): void {
        if (!this.markerIDs.includes(markerID)) throw Error(`Could not save pose to marker ${markerID}`);
        this.markerInfo.aruco_marker_info[markerID].pose = pose
        this.writeMarkers()
    }

    private async writeMarkers() {
        let updates: any = {};
        updates['/users/' + (this.uid) + '/markerNames'] = this.markerNames;
        updates['/users/' + (this.uid) + '/markerIDs'] = this.markerIDs;
        updates['/users/' + (this.uid) + '/markerInfo'] = this.markerInfo;
        return update(ref(this.database), updates);
    }

}