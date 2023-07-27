import ROSLIB from "roslib";
import { MAIN_BRANCH_LAYOUT } from "../default_layouts/MAIN_BRANCH_LAYOUT";
import { STRETCH2CLIENT_LAYOUT } from "../default_layouts/STRETCH2CLIENT_LAYOUT";
import { STUDY_BRANCH_LAYOUT } from "../default_layouts/STUDY_BRANCH_LAYOUT";
import { LayoutDefinition } from "operator/tsx/utils/component_definitions";
import { ArucoMarkersInfo, RobotPose } from "shared/util";
import { ARUCO_MARKER_INFO } from "../utils/aruco_markers_dict";

/** Type for all the possible names of default layouts. */
export type DefaultLayoutName = "Button Pad Overlays" | "Button Pad Panel" | "Button Grid/Joystick/Voice Commands";

/** Object with all the default layouts. */
export const DEFAULT_LAYOUTS: { [key in DefaultLayoutName]: LayoutDefinition } = {
    "Button Pad Overlays": STUDY_BRANCH_LAYOUT,
    "Button Pad Panel": MAIN_BRANCH_LAYOUT,
    "Button Grid/Joystick/Voice Commands": STRETCH2CLIENT_LAYOUT
}

/** 
 * Handles logic to store data, specifically maintain state between browser
 * reloads and save user custom layouts.
 */
export abstract class StorageHandler {
    /** 
     * Callback to execute once the storage is ready, for example after the 
     * user has signed into Firebase.
     */
    public onReadyCallback: () => void;

    constructor(onStorageHandlerReadyCallback: () => void) {
        this.onReadyCallback = onStorageHandlerReadyCallback.bind(this)
    }

    /**
     * Loads a user saved custom layout.
     * @param layoutName name of the layout to load
     * @returns the layout defintion
     */
    public abstract loadCustomLayout(layoutName: string): LayoutDefinition;

    /**
     * Saves a layout to the storage device.
     * @param layout the definition of the layout to save
     * @param layoutName the name of the layout
     */
    public abstract saveCustomLayout(layout: LayoutDefinition, layoutName: string): void;

    /**
     * Saves the current layout to preserve state between reloading the browser.
     * @param layout the current layout
     */
    public abstract saveCurrentLayout(layout: LayoutDefinition): void;

    /**
     * Loads the last used layout to preserve state between reloading the browser.
     */
    public abstract loadCurrentLayout(): LayoutDefinition | null;

    /**
     * Gets the list of all the user's saved layouts
     * @returns list of layout names
     */
    public abstract getCustomLayoutNames(): string[];

    /**
     * Save the joint state and its identifier
     * @param name the name of the pose
     * @param jointState the joint state to save
     */
    public abstract savePose(poseName: string, jointState: RobotPose): void;

    /**
     * Removes the pose from storage
     * @param name the name of the pose
     */
    public abstract deletePose(poseName: string): void;

    /**
     * Get the list of all saved poses
     * @returns list of all saved poses
     */
    public abstract getPoseNames(): string[];

    /**
     * Gets the pose associated with the given name
     * @param name the name of the pose
     * @returns a pose associated with the given name
     */
    public abstract getPose(poseName: string): RobotPose;

    /**
     * Save the map pose and its identifier
     * @param name the name of the pose
     * @param pose the pose on the map to save
     */
    public abstract saveMapPose(poseName: string, pose: ROSLIB.Transform): void;

    /**
     * Get an array of all saved map poses
     * @returns array of all saved map poses
     */
    public abstract getMapPoseNames(): string[];

    /**
     * Gets the map pose associated with the given name
     * @param name the name of the map pose
     * @returns a map pose associated with the given name
     */
    public abstract getMapPose(poseName: string): ROSLIB.Transform;

    /**
     * Gets an array of all saved poses
     * @returns an array of all saved poses  
     */
    public abstract getMapPoses(): ROSLIB.Transform[];

    /**
     * Removes the map pose from storage
     * @param name the name of the map pose
     */
    public abstract deleteMapPose(poseName: string): void;

    /**
     * Get the list of all saved pose sequence recordings
     * @returns list of all saved pose sequence recordings
     */
    public abstract getRecordingNames(): string[];

    /**
     * Gets the recording associated with the given name
     * @param recordingName the name of the recording
     * @returns a recording associated with the given name
     */
    public abstract getRecording(recordingName: string): RobotPose[];

    /**
     * Save the pose sequence and its identifier
     * @param recordingName the name of the recording
     * @param poses the pose sequence to save
     */
    public abstract savePoseRecording(recordingName: string, poses: RobotPose[]): void;

    /**
     * Removes the recording from storage
     * @param recordingName the name of the recording
     */
    public abstract deleteRecording(recordingName: string): void;

    /**
     * Gets the last saved state from the user's layout, or gets the default 
     * layout if the user has no saved state.
     * @returns layout definition for the layout that should be loaded into the
     *          operator page.
     */
    public loadCurrentLayoutOrDefault(): LayoutDefinition {
        const currentLayout = this.loadCurrentLayout();
        if (!currentLayout) return Object.values(DEFAULT_LAYOUTS)[1];
        console.log('loading saved layout')
        return currentLayout;
    }

    /**
     * Gets all the default layout names
     * @returns list of default layout names
     */
    public getDefaultLayoutNames(): string[] {
        return Object.keys(DEFAULT_LAYOUTS);
    }

    /**
     * Gets a default layout
     * @param layoutName default layout to load
     * @returns the layout definition for the default layout
     */
    public loadDefaultLayout(layoutName: DefaultLayoutName): LayoutDefinition {
        return DEFAULT_LAYOUTS[layoutName];
    }

    /**
     * Save the aruco marker and its identifier
     * @param markerID the ID of the aruco marker
     * @param markerName the name of the aruco marker
     */
    public abstract saveMarker(markerID: string, markerName: string): void;

    /**
     * Removes the aruco maker from storage
     * @param name the name of the aruco marker
     */
    public abstract deleteMarker(markerName: string): void;

    /**
     * Get the list of all saved aruco markers
     * @returns list of all saved aruco markers
     */
    public abstract getArucoMarkerNames(): string[];

     /**
     * Get the list of all saved aruco markers IDs
     * @returns list of all saved aruco markers IDs
     */
     public abstract getArucoMarkerIDs(): string[];

    /**
     * Get the list of all saved aruco marker info
     * @returns list of all saved aruco marker info
     */
    public abstract getArucoMarkerInfo(): ArucoMarkersInfo;

    /**
     * Save the relative pose for the given aruco marker
     * @param markerID the ID of the aruco marker
     * @param pose the relative pose to save
     */
    public abstract saveRelativePose(markerID: string, pose: ROSLIB.Transform): void;

    public loadDefaultArucoMarkers(): ArucoMarkersInfo {
        return ARUCO_MARKER_INFO
    }

    public loadDefaultArucoMarkerIDs(): string[] {
        // return Object.keys(ARUCO_MARKER_INFO.aruco_marker_info)
        
        // Only return ID for docking station
        // Other IDs are currently not applicable for aruco navigation
        return['245']
    }

    public loadDefaultArucoMarkerNames(): string[] {
        // let names: string[] = []
        // let markerInfo = Object.values(ARUCO_MARKER_INFO.aruco_marker_info)
        // markerInfo.forEach((info) => {
        //     names.push(info.name)
        // })
        // return names
        
        // Only return ID for docking station
        // Other IDs are currently not applicable for aruco navigation
        return ['docking_station']
    }
}