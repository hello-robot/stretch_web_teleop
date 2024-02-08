import { LayoutDefinition, ComponentType, ActionMode, CameraViewId, ButtonPadId, OverheadVideoStreamDef, TabDefinition, PanelDefinition } from "../utils/component_definitions";

export const REMOTE_CAREGIVER_LAYOUT: LayoutDefinition = {
    // All components have a type
    type: ComponentType.Layout,
    // If voice control should be displayed on the operator page
    displayVoiceControl: false,
    displayMovementRecorder: false,
    // The state of the action mode dropdown
    actionMode: ActionMode.PressAndHold,
    displayPoseLibrary: false,
    displayArucoMarkers: false,
    displayLabels: true,
    // The customizable components in the layout
    children: [
        {
            // Tab contains a single camera view
            type: ComponentType.CameraView,
            // From the Stretch overhead camera
            id: CameraViewId.overhead,
        } as OverheadVideoStreamDef
    ]
}