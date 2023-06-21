import { MAIN_BRANCH_LAYOUT } from "../default_layouts/MAIN_BRANCH_LAYOUT";
import { STRETCH2CLIENT_LAYOUT } from "../default_layouts/STRETCH2CLIENT_LAYOUT";
import { STUDY_BRANCH_LAYOUT } from "../default_layouts/STUDY_BRANCH_LAYOUT";
import { LayoutDefinition } from "operator/tsx/utils/component_definitions";
import { RobotPose } from "shared/util";

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
     * Gets the last saved state from the user's layout, or gets the default 
     * layout if the user has no saved state.
     * @returns layout definition for the layout that should be loaded into the
     *          operator page.
     */
    public loadCurrentLayoutOrDefault(): LayoutDefinition {
        const currentLayout = this.loadCurrentLayout();
        if (!currentLayout) return Object.values(DEFAULT_LAYOUTS)[0];
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
}