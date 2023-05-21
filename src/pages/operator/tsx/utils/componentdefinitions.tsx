/**
 * @summary Definitions to describe different components to render
 */

/**
 * High-level type of the component
 */
export enum ComponentType {
    Layout = "Layout",
    Tabs = "Tabs",
    SingleTab = "Single Tab",
    VideoStream = "Video Stream",
    ButtonPad = "Button Pad",
    PredictiveDisplay = "Predictive Display"
}

/**
 * ID for the video stream, one for each of the cameras
 */
export enum VideoStreamId {
    overhead = "Overhead",
    realsense = "Realsense",
    gripper = "Gripper"
}

/**
 * ID for a button pad, describes the shape of the button pad
 */
export enum ButtonPadId {
    overhead = "Overhead",
    realsense = "Realsense",
    gripper = "Gripper"
}

/**
 * Definition for any interface component. Any video stream, button pad, 
 * tabs, etc. definition will have these fields.
 */
export type ComponentDefinition = {
    /** Indicates the type of the component */
    type: ComponentType;
}

/**
 * Definition for a button pad component
 */
export type ButtonPadDef = ComponentDefinition & {
    /** Indicates the shape and functions on the button pad*/
    id: ButtonPadId;
}

export type ParentComponentDefinition = ComponentDefinition & {
    children: ComponentDefinition[];
}

/**
 * Definition for a single tab in a tabs component
 */
export type SingleTabDef = ParentComponentDefinition & {
    /** The label that appears at the top of the tabs object. */
    label: string;
}

/**
 * Definition for a tabs component
 */
export type TabsDef = ParentComponentDefinition & {
    /** List of definitions for individual tabs */
    children: SingleTabDef[];
}

/**
 * Definition for a video stream component
 */
export type VideoStreamDef = ParentComponentDefinition & {
    /** Indicates the camera video of the video stream */
    id: VideoStreamId;
}