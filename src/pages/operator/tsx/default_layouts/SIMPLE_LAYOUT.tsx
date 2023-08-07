import { LayoutDefinition, ComponentType, ActionMode, CameraViewId, ButtonPadId, OverheadVideoStreamDef, TabDefinition, PanelDefinition } from "../utils/component_definitions";

export const SIMPLE_LAYOUT: LayoutDefinition = {
    // All components have a type
    type: ComponentType.Layout,
    // If voice control should be displayed on the operator page
    displayVoiceControl: false,
    displayMovementRecorder: false,
    // The state of the action mode dropdown
    actionMode: ActionMode.StepActions,
    displayPoseLibrary: false,
    displayArucoMarkers: false,
    displayLabels: true,
    // The customizable components in the layout
    children: [
        {
            // The layout contains a single panel
            type: ComponentType.Panel,
            children: [
                // The panel contains a single tab
                {
                    type: ComponentType.SingleTab,
                    // The title of the tab is "Tab One"
                    label: 'Tab One',
                    children: [
                        {
                            // Tab contains a single camera view
                            type: ComponentType.CameraView,
                            // From the Stretch overhead camera
                            id: CameraViewId.overhead,
                            children: [
                                {
                                    // Camera view has a button pad overlay
                                    type: ComponentType.ButtonPad,
                                    // Button pad to control the base 
                                    id: ButtonPadId.Drive
                                }
                            ]
                        } as OverheadVideoStreamDef
                    ]
                } as TabDefinition
            ]
        } as PanelDefinition
    ]
}