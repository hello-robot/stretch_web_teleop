import { StorageHandler } from "./StorageHandler";
import { LayoutDefinition } from "../utils/component_definitions";

import {
  FirebaseOptions,
  FirebaseError,
  initializeApp,
  FirebaseApp,
} from "firebase/app";
import {
  Auth,
  getAuth,
  User,
  signInWithPopup,
  GoogleAuthProvider,
  onAuthStateChanged,
} from "firebase/auth";
import {
  Database,
  getDatabase,
  child,
  get,
  ref,
  update,
  push,
} from "firebase/database";
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
  private mapPoseTypes: { [name: string]: string };
  private recordings: { [name: string]: RobotPose[] };
  private textToSpeech: string[];
  private markerNames: string[];
  private markerIDs: string[];
  private markerInfo: ArucoMarkersInfo;

  constructor(
    onStorageHandlerReadyCallback: () => void,
    config: FirebaseOptions,
  ) {
    super(onStorageHandlerReadyCallback);
    this.config = config;
    this.app = initializeApp(this.config);
    this.database = getDatabase(this.app);
    this.auth = getAuth(this.app);
    this.GAuthProvider = new GoogleAuthProvider();

    this.userEmail = "";
    this.uid = "";
    this.layouts = {};
    this.currentLayout = null;
    this.poses = {};
    this.mapPoses = {};
    this.mapPoseTypes = {};
    this.recordings = {};
    this.textToSpeech = [];
    this.markerNames = [];
    this.markerIDs = [];
    this.markerInfo = {} as ArucoMarkersInfo;
    onAuthStateChanged(this.auth, (user) => this.handleAuthStateChange(user));

    this.signInWithGoogle();
  }

  private handleAuthStateChange(user: User | null) {
    if (user) {
      this.uid = user.uid;
      this.userEmail = user.email!;

      this.getUserDataFirebase()
        .then(async (userData) => {
          this.layouts = userData.layouts;
          this.currentLayout = userData.currentLayout;
          this.mapPoses = userData.map_poses;
          this.mapPoseTypes = userData.map_pose_types;
          this.recordings = userData.recordings;
          this.textToSpeech = userData.text_to_speech;

          this.onReadyCallback();
        })
        .catch((error) => {
          console.log(
            "Detected that FirebaseModel isn't initialized for user ",
            this.uid,
          );
          this.onReadyCallback();
        });
    }
  }

  private async getUserDataFirebase() {
    const snapshot = await get(child(ref(this.database), "/users/" + this.uid));

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
          return Promise.resolve();
        })
        .catch(this.handleError);
    }
  }

  private handleError(error: FirebaseError) {
    const errorCode = error.code;
    const errorMessage = error.message;
    console.error("firebaseError: " + errorCode + ": " + errorMessage);
    console.trace();
    return Promise.reject();
  }

  public loadCustomLayout(layoutName: string): LayoutDefinition {
    let layout = this.layouts![layoutName];
    if (!layout) throw Error(`Could not load custom layout ${layoutName}`);
    return JSON.parse(JSON.stringify(layout));
  }

  public saveCustomLayout(layout: LayoutDefinition, layoutName: string): void {
    this.layouts[layoutName] = layout;
    this.writeLayouts(this.layouts);
  }

  public saveCurrentLayout(layout: LayoutDefinition): void {
    this.currentLayout = layout;

    let updates: any = {};
    updates["/users/" + this.uid + "/currentLayout"] = layout;
    update(ref(this.database), updates);
  }

  public loadCurrentLayout(): LayoutDefinition | null {
    return this.currentLayout;
  }

  public getCustomLayoutNames(): string[] {
    if (!this.layouts) return [];
    return Object.keys(this.layouts);
  }

  private async writeLayouts(layouts: { [name: string]: LayoutDefinition }) {
    this.layouts = layouts;

    let updates: any = {};
    updates["/users/" + this.uid + "/layouts"] = layouts;
    return update(ref(this.database), updates);
  }

  public getMapPoseNames(): string[] {
    if (!this.mapPoses) return [];
    return Object.keys(this.mapPoses);
  }

  public saveMapPose(name: string, pose: ROSLIB.Transform, poseType: string) {
    this.mapPoses[name] = pose;
    this.mapPoseTypes[name] = poseType;
    this.writeMapPoses(this.mapPoses);
    this.writeMapPoseTypes(this.mapPoseTypes);
  }

  private async writeMapPoses(poses: { [name: string]: ROSLIB.Transform }) {
    this.mapPoses = poses;

    let updates: any = {};
    updates["/users/" + this.uid + "/map_poses"] = poses;
    return update(ref(this.database), updates);
  }

  private async writeMapPoseTypes(poseTypes: { [name: string]: string }) {
    this.mapPoseTypes = poseTypes;

    let updates: any = {};
    updates["/users/" + this.uid + "/map_pose_types"] = poseTypes;
    return update(ref(this.database), updates);
  }

  public getMapPose(poseName: string): ROSLIB.Transform {
    let pose = this.mapPoses![poseName];
    if (!pose) throw Error(`Could not load pose ${poseName}`);
    return JSON.parse(JSON.stringify(pose));
  }

  public getMapPoses(): ROSLIB.Transform[] {
    const poseNames = this.getMapPoseNames();
    var poses: ROSLIB.Transform[] = [];
    poseNames.forEach((poseName) => {
      const pose = this.getMapPose(poseName);
      poses.push(pose);
    });
    return poses;
  }

  public getMapPoseTypes(): string[] {
    if (!this.mapPoseTypes) return [];
    return Object.keys(this.mapPoseTypes);
  }

  public deleteMapPose(poseName: string): void {
    let pose = this.mapPoses![poseName];
    if (!pose) throw Error(`Could not delete pose ${poseName}`);
    delete this.mapPoses[poseName];
    delete this.mapPoseTypes[poseName];
    this.writeMapPoses(this.mapPoses);
    this.writeMapPoseTypes(this.mapPoseTypes);
  }

  public getRecordingNames(): string[] {
    if (!this.recordings) return [];
    return Object.keys(this.recordings);
  }

  public getRecording(recordingName: string): RobotPose[] {
    let recording = this.recordings![recordingName];
    if (!recording) throw Error(`Could not load recording ${recordingName}`);
    return JSON.parse(JSON.stringify(recording));
  }

  public savePoseRecording(recordingName: string, poses: RobotPose[]): void {
    this.recordings[recordingName] = poses;
    this.writeRecordings(this.recordings);
  }

  private async writeRecordings(recordings: { [name: string]: RobotPose[] }) {
    this.recordings = recordings;

    let updates: any = {};
    updates["/users/" + this.uid + "/recordings"] = recordings;
    return update(ref(this.database), updates);
  }

  public deleteRecording(recordingName: string): void {
    let recording = this.recordings![recordingName];
    if (!recording) throw Error(`Could not delete recording ${recordingName}`);
    delete this.recordings[recordingName];
    this.writeRecordings(this.recordings);
  }

  /**
   * NOTE: The below four text-to-speech functions have NOT been tested.
   */

  public getSavedTexts(): string[] {
    if (!this.textToSpeech) return [];
    return this.textToSpeech;
  }

  public saveText(text: string): void {
    if (this.textToSpeech.includes(text)) return;
    this.textToSpeech.push(text);
    this.writeTextToSpeech(this.textToSpeech);
  }

  private async writeTextToSpeech(textToSpeech: string[]) {
    this.textToSpeech = textToSpeech;

    let updates: any = {};
    updates["/users/" + this.uid + "/text_to_speech"] = textToSpeech;
    return update(ref(this.database), updates);
  }

  public deleteText(text: string): void {
    if (!this.textToSpeech.includes(text)) return;
    const index = this.textToSpeech.indexOf(text);
    this.textToSpeech.splice(index, 1);
    this.writeTextToSpeech(this.textToSpeech);
  }
}
