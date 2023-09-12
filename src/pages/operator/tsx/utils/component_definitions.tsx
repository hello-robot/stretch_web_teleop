/**
 * @summary Definitions to describe different components to render
 */

import ROSLIB from "roslib";
import { StorageHandler } from "../storage_handler/StorageHandler";

/** Enumerator for the possible action modes */
export enum ActionMode {
    StepActions = 'Step-Actions',
    PressRelease = 'Press-Release',
    ClickClick = 'Click-Click'
}

/**
 * High-level type of the component
 */
export enum ComponentType {
    Layout = "Layout",
    Panel = "Panel",
    SingleTab = "Tab",
    CameraView = "Camera View",
    ButtonPad = "Button Pad",
    PredictiveDisplay = "Predictive Display",
    ButtonGrid = "Button Grid",
    VirtualJoystick = "Joystick",
    Map = "Map"
}

/**
 * ID for the video stream, one for each of the cameras
 */
export enum CameraViewId {
    overhead = "Overhead",
    realsense = "Realsense",
    gripper = "Gripper"
}

/**
 * ID for a button pad describes the shape and button functions of the button pad
 */
export enum ButtonPadId {
    // Drive = "Drive",
    ManipRealsense = "Drive/Arm/Gripper/Wrist",
    Gripper = "Gripper",
    ManipOverhead = "Drive/Arm/Wrist",
    Base = "Drive",
    Camera = "Camera",
    Wrist = "Wrist",
    Arm = "Arm",
    ArmMobile = "Arm Mobile",
    GripperMobile = "Gripper Mobile",
    DriveMobile = "Drive Mobile"
}


/**
 * Identifier for the subtype of the component 
 * (e.g. which video stream camera, or which button pad)
 * @note any new components with ID fields should be added to this type
 */
export type ComponentId = CameraViewId | ButtonPadId;

/**
 * Definition for any interface component. Any video stream, button pad, 
 * tabs, etc. definition will have these fields.
 */
export type ComponentDefinition = {
    /** Indicates the type of the component */
    type: ComponentType;
    /** Indicates the identifier for the sub-type of the component */
    id?: ComponentId;
}

/**
 * Definition for a button pad component
 */
export type ButtonPadDefinition = ComponentDefinition & {
    /** Indicates the shape and functions on the button pad*/
    id: ButtonPadId;
}

export type ParentComponentDefinition = ComponentDefinition & {
    children: ComponentDefinition[];
}

export type LayoutDefinition = ParentComponentDefinition & {
    displayVoiceControl: boolean;
    displayPoseLibrary: boolean;
    displayMovementRecorder: boolean;
    displayArucoMarkers: boolean;
    displayLabels: boolean;
    actionMode: ActionMode;
}

/**
 * Definition for a single tab in a tabs component
 */
export type TabDefinition = ParentComponentDefinition & {
    /** The label that appears at the top of the tabs object. */
    label: string;
}

/**
 * Definition for a tabs component
 */
export type PanelDefinition = ParentComponentDefinition & {
    /** List of definitions for individual tabs */
    children: TabDefinition[];
}

/**
 * Definition for a video stream component
 */
export type CameraViewDefinition = ParentComponentDefinition & {
    /** Indicates the camera video of the video stream */
    id: CameraViewId;
}

/**
 * Definition for the overhead stream component
 * 
 * @note these modifications to the overhead view are implemented in the
 * backend, so if multiple overhead streams are visible to the user 
 * simultaneously, any change to this defintion for one view will impact
 * all views.
 */
export type OverheadVideoStreamDef = CameraViewDefinition & {
    /** 
     * If true, the view should be cropped and rotated to focus on the gripper.
     * Otherwise, camera view should be unchanged
     * */
    gripperView?: boolean;
}

/**
 * Definition for the Realsense video stream component
 * 
 * @note these modifications to the Realsense view are implemented in the
 * backend, so if multiple Realsense streams are visible to the user 
 * simultaneously, any change to this defintion for one view will impact
 * all views.
 */
export type RealsenseVideoStreamDef = CameraViewDefinition & {
    /**
     * If the Realsense camera should pan and tilt to keep the gripper centered 
     * in the view.
     */
    followGripper?: boolean;
    /**
     * If the AR depth ring should be shown to indicate the extent of the 
     * reachable area.
     */
    depthSensing?: boolean;
    /**
     * If the aruco markers overlay should be displayed
     */
    arucoMarkers?: boolean;
}

/**
 * Definition for the map component
 */
export type MapDefinition = ComponentDefinition & {
    /**
     * Enable/disable the click listener on the map for settings a goal 
     */
    selectGoal?: boolean
}