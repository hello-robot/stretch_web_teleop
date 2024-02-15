import { StorageHandler } from "./StorageHandler";
import { LayoutDefinition } from "../utils/component_definitions";
import { ArucoMarkersInfo, RobotPose } from "shared/util";
import ROSLIB from "roslib";

/** Uses browser local storage to store data. */
export class LocalStorageHandler extends StorageHandler {
    public static CURRENT_LAYOUT_KEY = "user_custom_layout";    
    public static LAYOUT_NAMES_KEY = "user_custom_layout_names";
    public static POSE_NAMES_KEY = "user_pose_names";
    public static MAP_POSE_NAMES_KEY = "user_map_pose_names";
    public static MAP_POSE_TYPES_KEY = "user_map_pose_types";
    public static POSE_RECORDING_NAMES_KEY = "user_pose_recording_names";

    constructor(onStorageHandlerReadyCallback: () => void) {
        super(onStorageHandlerReadyCallback);
        // Allow the initialization process to complete before invoking the callback
        setTimeout(() => {
            this.getCustomLayoutNames()
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

    public saveMapPose(poseName: string, pose: ROSLIB.Transform, poseType: string) {
        const poseNames = this.getMapPoseNames();
        const poseTypes = this.getMapPoseTypes();
        // If pose name does not exist add the name, type and pose, otherwise replace the 
        // type and pose for the given name
        if (!poseNames.includes(poseName)) {
            poseNames.push(poseName);
            poseTypes.push(poseType)
        } else {
            let idx = poseNames.indexOf(poseName)
            poseTypes[idx] = poseType
        }
        localStorage.setItem(LocalStorageHandler.MAP_POSE_NAMES_KEY, JSON.stringify(poseNames));
        localStorage.setItem(LocalStorageHandler.MAP_POSE_TYPES_KEY, JSON.stringify(poseTypes));
        localStorage.setItem('map_' + poseName, JSON.stringify(pose));
    }

    public getMapPoseNames(): string[] {
        const storedJson = localStorage.getItem(LocalStorageHandler.MAP_POSE_NAMES_KEY);
        if (!storedJson) return [];
        return JSON.parse(storedJson)
    }

    public getMapPose(poseName: string): ROSLIB.Transform {
        const storedJson = localStorage.getItem('map_' + poseName);
        if (!storedJson) throw Error(`Could not load pose ${poseName}`);
        return JSON.parse(storedJson);
    }

    public getMapPoses(): ROSLIB.Transform[] {
        const poseNames = this.getMapPoseNames()
        var poses: ROSLIB.Transform[] = []
        poseNames.forEach(poseName => {
            const pose = this.getMapPose(poseName)
            poses.push(pose)
        });
        return poses
    }

    public getMapPoseTypes(): string[] {
        const storedJson = localStorage.getItem(LocalStorageHandler.MAP_POSE_TYPES_KEY);
        if (!storedJson) return [];
        return JSON.parse(storedJson)
    }

    public deleteMapPose(poseName: string): void {
        const poseNames = this.getMapPoseNames();
        if (!poseNames.includes(poseName)) return;
        localStorage.removeItem('map_' + poseName)
        const index = poseNames.indexOf(poseName)
        poseNames.splice(index, 1)
        const poseTypes = this.getMapPoseTypes()
        poseTypes.splice(index, 1)
        localStorage.setItem(LocalStorageHandler.MAP_POSE_NAMES_KEY, JSON.stringify(poseNames));
        localStorage.setItem(LocalStorageHandler.MAP_POSE_TYPES_KEY, JSON.stringify(poseTypes));
    }

    public getRecordingNames(): string[] {
        const storedJson = localStorage.getItem(LocalStorageHandler.POSE_RECORDING_NAMES_KEY);
        if (!storedJson) return [];
        return JSON.parse(storedJson)
    }

    public getRecording(recordingName: string): RobotPose[] {
        const storedJson = localStorage.getItem('recording_' + recordingName);
        if (!storedJson) throw Error(`Could not load recording ${recordingName}`);
        return JSON.parse(storedJson);
    }

    public savePoseRecording(recordingName: string, poses: RobotPose[]): void {
        const recordingNames = this.getRecordingNames();
        if (!recordingNames.includes(recordingName)) recordingNames.push(recordingName);
        localStorage.setItem(LocalStorageHandler.POSE_RECORDING_NAMES_KEY, JSON.stringify(recordingNames));
        localStorage.setItem('recording_' + recordingName, JSON.stringify(poses));
    }

    public deleteRecording(recordingName: string): void {
        const recordingNames = this.getRecordingNames();
        if (!recordingNames.includes(recordingName)) return;
        localStorage.removeItem('recording_' + recordingName)
        const index = recordingNames.indexOf(recordingName)
        recordingNames.splice(index, 1)
        localStorage.setItem(LocalStorageHandler.POSE_RECORDING_NAMES_KEY, JSON.stringify(recordingNames));
    }
}