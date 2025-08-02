import ROSLIB from "roslib";
import { BASIC_LAYOUT } from "../default_layouts/SIMPLE_LAYOUT";
import { PROGRAM_EDITOR_LAYOUT } from "../default_layouts/PROGRAM_EDITOR_LAYOUT";
import { EXECUTION_MONITOR_LAYOUT } from "../default_layouts/EXECUTION_MONITOR_LAYOUT";
import { LayoutDefinition } from "operator/tsx/utils/component_definitions";
import { RobotPose } from "shared/util";
import { ARUCO_MARKER_INFO } from "../utils/aruco_markers_dict";

/** Type for all the possible names of default layouts. */
export type DefaultLayoutName = "Basic Layout" | "Program Editor Layout" | "Execution Monitor Layout";

/** Object with all the default layouts. */
export const DEFAULT_LAYOUTS: { [key in DefaultLayoutName]: LayoutDefinition } =
    {
        "Basic Layout": BASIC_LAYOUT,
        "Program Editor Layout": PROGRAM_EDITOR_LAYOUT,
        "Execution Monitor Layout": EXECUTION_MONITOR_LAYOUT,
    };

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
        this.onReadyCallback = onStorageHandlerReadyCallback.bind(this);
    }

    /**
     * Loads a user saved custom layout.
     * @param layoutName name of the layout to load
     * @returns the layout definition
     */
    public abstract loadCustomLayout(layoutName: string): LayoutDefinition;

    /**
     * Saves a layout to the storage device.
     * @param layout the definition of the layout to save
     * @param layoutName the name of the layout
     */
    public abstract saveCustomLayout(
        layout: LayoutDefinition,
        layoutName: string,
    ): void;

    /**
     * Saves the current layout to preserve state between reloading the browser.
     * @param layout the current layout
     * @param mode the program mode (Demonstrate, Create Program, Run Program)
     */
    public abstract saveCurrentLayout(layout: LayoutDefinition, mode?: string): void;

    /**
     * Loads the last used layout to preserve state between reloading the browser.
     * @param mode the program mode (Demonstrate, Create Program, Run Program)
     */
    public abstract loadCurrentLayout(mode?: string): LayoutDefinition | null;

    /**
     * Gets the list of all the user's saved layouts
     * @returns list of layout names
     */
    public abstract getCustomLayoutNames(): string[];

    /**
     * Save the map pose and its identifier
     * @param name the name of the pose
     * @param pose the pose on the map to save
     */
    public abstract saveMapPose(
        poseName: string,
        pose: ROSLIB.Transform,
        poseType: string,
    ): void;

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
     * Get an array of all the saved map pose types (map or aruco)
     * @returns an array of all saved map pose types
     */
    public abstract getMapPoseTypes(): string[];

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
    public abstract savePoseRecording(
        recordingName: string,
        poses: RobotPose[],
    ): void;

    /**
     * Removes the recording from storage
     * @param recordingName the name of the recording
     */
    public abstract deleteRecording(recordingName: string): void;

    /**
     * Gets all the text to speech messages saved by the user.
     * @returns list of all saved text to speech messages
     */
    public abstract getSavedTexts(): string[];

    /**
     * Saves a text to speech message to the storage device.
     * @param text the text to save
     */
    public abstract saveText(text: string): void;

    /**
     * Deletes a text to speech message from the storage device.
     * @param text the text to delete
     */
    public abstract deleteText(text: string): void;

    /**
     * Gets the last saved state from the user's layout, or gets the default
     * layout if the user has no saved state.
     * @param mode the program mode (Demonstrate, Create Program, Run Program)
     * @returns layout definition for the layout that should be loaded into the
     *          operator page.
     */
    public loadCurrentLayoutOrDefault(mode?: string): LayoutDefinition {
        const currentLayout = this.loadCurrentLayout(mode);
        if (!currentLayout) return Object.values(DEFAULT_LAYOUTS)[0];
        console.log("loading saved layout for mode:", mode);
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
        return JSON.parse(JSON.stringify(DEFAULT_LAYOUTS[layoutName]));
    }
}
