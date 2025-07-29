/**
 * @summary Definitions to describe different components to render
 */

/** Enumerator for the possible action modes */
export enum ActionMode {
    StepActions = "Step-Actions",
    PressAndHold = "Press-And-Hold",
    ClickClick = "Click-Click",
}

/**
 * High-level type of the component
 */
export enum ComponentType {
    Layout = "Layout",
    LayoutGrid = "Layout Grid",
    Panel = "Panel",
    SingleTab = "Tab",
    CameraView = "Camera View",
    ButtonPad = "Button Pad",
    PredictiveDisplay = "Predictive Display",
    ButtonGrid = "Button Grid",
    VirtualJoystick = "Joystick",
    Map = "Map",
    RunStopButton = "Run Stop Button",
    BatteryGuage = "Battery Gauge",
    RosbagRecorder = "Rosbag Recorder",
    ProgramEditor = "Program Editor",
    Library = "Library"
}

/**
 * ID for the video stream, one for each of the cameras
 */
export enum CameraViewId {
    overhead = "Overhead",
    realsense = "Realsense",
    gripper = "Gripper",
}

/**
 * ID for a button pad describes the shape and button functions of the button pad
 */
export enum ButtonPadId {
    // Drive = "Drive",
    Base = "Drive",
    Arm = "Arm & Lift",
    DexWrist = "Dex Wrist",
    GripperLift = "Gripper & Lift",
    ManipRealsense = "Drive/Arm/Gripper/Wrist",
    Camera = "Camera",
    // Wrist = "Wrist",
}

export enum ButtonPadIdMobile {
    Arm = "Arm Mobile",
    Gripper = "Gripper Mobile",
    Drive = "Drive Mobile",
}

/**
 * Identifier for the subtype of the component
 * (e.g. which video stream camera, or which button pad)
 * @note any new components with ID fields should be added to this type
 */
export type ComponentId = CameraViewId | ButtonPadId | ButtonPadIdMobile;

/**
 * Definition for any interface component. Any video stream, button pad,
 * tabs, etc. definition will have these fields.
 */
export type ComponentDefinition = {
    /** Indicates the type of the component */
    type: ComponentType;
    /** Indicates the identifier for the sub-type of the component */
    id?: ComponentId;
};

/**
 * Definition for a button pad component
 */
export type ButtonPadDefinition = ComponentDefinition & {
    /** Indicates the shape and functions on the button pad*/
    id: ButtonPadId | ButtonPadIdMobile;
};

export type ParentComponentDefinition = ComponentDefinition & {
    children: ComponentDefinition[];
};

export type LayoutDefinition = ComponentDefinition & {
    displayMovementRecorder?: boolean;
    displayTextToSpeech?: boolean;
    displayRosbagRecorder?: boolean;
    displayLabels: boolean;
    actionMode: ActionMode;
    children: LayoutGridDefinition[];
};

export type LayoutGridDefinition = ComponentDefinition & {
    children: PanelDefinition[];
};

/**
 * Definition for a tabs component
 */
export type PanelDefinition = ComponentDefinition & {
    /** List of definitions for individual tabs */
    children: TabDefinition[];
};

/**
 * Definition for a single tab in a tabs component
 */
export type TabDefinition = ParentComponentDefinition & {
    /** The label that appears at the top of the tabs object. */
    label: string;
};

/**
 * Definition for a video stream component
 */
export type CameraViewDefinition = ParentComponentDefinition & {
    /** Indicates the camera video of the video stream */
    id: CameraViewId;
    /** Whether to display the default buttons under the camera view */
    displayButtons: boolean;
};

/**
 * Definition for the gripper stream component
 *
 * @note these modifications to the overhead view are implemented in the
 * backend, so if multiple overhead streams are visible to the user
 * simultaneously, any change to this definition for one view will impact
 * all views.
 */
export type GripperVideoStreamDef = CameraViewDefinition & {
    /**
     * Whether to display the expanded gripper view or the default one
     */
    expandedGripperView?: boolean;
    /**
     * If pixels within the graspable region should be highlighted.
     */
    depthSensing?: boolean;
};

/**
 * Definition for the fixed overhead stream component
 *
 * @note these modifications to the overhead view are implemented in the
 * backend, so if multiple overhead streams are visible to the user
 * simultaneously, any change to this definition for one view will impact
 * all views.
 */
export type FixedOverheadVideoStreamDef = CameraViewDefinition & {
    /**
     * Predictive display toggle
     */
    predictiveDisplay?: boolean;
};

/**
 * Definition for the adjustable overhead stream component
 *
 * @note these modifications to the overhead view are implemented in the
 * backend, so if multiple overhead streams are visible to the user
 * simultaneously, any change to this definition for one view will impact
 * all views.
 */
export type AdjustableOverheadVideoStreamDef = CameraViewDefinition & {
    /**
     * If the Realsense camera should pan and tilt to keep the gripper centered
     * in the view.
     */
    followGripper?: boolean;
    /**
     * Predictive display toggle
     */
    predictiveDisplay?: boolean;
};

/**
 * Definition for the Realsense video stream component
 *
 * @note these modifications to the Realsense view are implemented in the
 * backend, so if multiple Realsense streams are visible to the user
 * simultaneously, any change to this definition for one view will impact
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
     * If the user should be allowed to click on the image and have the robot
     * move to the pre-grasp position relative to the clicked object.
     */
    selectObjectForMoveToPregrasp?: boolean;
    /**
     * If the user has toggled on the human body pose estimate overlay.
     */
    bodyPoseAR?: boolean;
};

/**
 * Definition for the map component
 */
export type MapDefinition = ComponentDefinition & {
    /**
     * Enable/disable the click listener on the map for settings a goal
     */
    selectGoal?: boolean;
};

/**
 * Definition for the run stop button
 */
export type RunStopDefinition = ComponentDefinition;

/**
 * Definition for the program editor component
 */
export type ProgramEditorDefinition = ComponentDefinition;

export type LibraryDefinition = ComponentDefinition;
