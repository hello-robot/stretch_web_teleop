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
    ButtonPad = "Button Pad"
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
    gripper = "Gripper",
    PredictiveDisplay = "Predictive Display"
}

/**
 * Definition for any interface component. Any video stream, button pad, 
 * tabs, etc. definition will have these fields.
 */
export type ComponentDefinition = {
    type: ComponentType;
    id?: any;
}

/**
 * Definition for a button pad component
 */
export type ButtonPadDef = ComponentDefinition & {
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
export type VideoStreamDef = ComponentDefinition & {
    id: VideoStreamId;
    /** The button pad to overlay, if undefined then no overlay. */
    children: ButtonPadDef[];
}