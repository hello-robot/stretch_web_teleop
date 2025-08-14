import {
    ComponentType,
    CameraViewId,
    ButtonPadId,
    CameraViewDefinition,
    ButtonPadDefinition,
    PanelDefinition,
    TabDefinition,
    LayoutDefinition,
    ActionMode,
    LayoutGridDefinition,
} from "../utils/component_definitions";

/**
 * Program Editor Layout
 */
export const PROGRAM_EDITOR_LAYOUT: LayoutDefinition = {
    type: ComponentType.Layout,
    displayMovementRecorder: false,
    displayTextToSpeech: false,
    displayLabels: true,
    actionMode: ActionMode.PressAndHold,
    children: [
        {
            type: ComponentType.LayoutGrid,
            flex: 2, 
            children: [
                {
                    type: ComponentType.Panel,
                    children: [
                        {
                            type: ComponentType.SingleTab,
                            label: "Program Editor",
                            children: [
                                {
                                    type: ComponentType.ProgramEditor,
                                },
                            ],
                        },
                    ],
                } as PanelDefinition,
            ],
        } as LayoutGridDefinition,
        {
            type: ComponentType.LayoutGrid,
            flex: 1, 
            children: [
                {
                    type: ComponentType.Panel,
                    children: [
                        {
                            type: ComponentType.SingleTab,
                            label: "Library",
                            children: [
                                {
                                    type: ComponentType.Library,
                                },
                            ],
                        },
                    ],
                } as PanelDefinition,
            ],
        } as LayoutGridDefinition,
    ],
}; 