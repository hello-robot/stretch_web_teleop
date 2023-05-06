/**
 * Definitions to describe different components to render
 */



/**
 * High-level type of the component
 */
export enum ComponentType {
    Tabs,
    VideoStream,
    ButtonPad
}

/**
 * ID for the video stream, one for each of the cameras
 */
export enum VideoStreamId {
    overhead,
    realsense,
    gripper
}

/**
 * ID for a button pad, describes the shape of the button pad
 */
export enum ButtonPadId {
    overhead,
    realsense,
    gripper
}

/**
 * Definition for any interface component. Any video stream, button pad, 
 * tabs, etc. definition will have these fields.
 */
export interface CompDef {
    type: ComponentType;
    id: any;
}

/**
 * Definition for a single tab in a tabs component
 */
export interface SingleTabDef {
    /** The label that appears at the top of the tabs object. */
    label: string;
    /** List of definitions for what's in this tab */
    contents: CompDef[];
}

/**
 * Definition for a tabs component
 */
export interface TabsDef extends CompDef {
    /** List of definitions for individual tabs */
    tabs: SingleTabDef[];
}

/**
 * Definition for a button pad component
 */
export interface ButtonPadDef extends CompDef {}

/**
 * Definition for a video stream component
 */
export interface VideoStreamDef extends CompDef {
    /** The button pad to overlay, if undefined then no overlay. */
    buttonPadDef?: ButtonPadDef;
}