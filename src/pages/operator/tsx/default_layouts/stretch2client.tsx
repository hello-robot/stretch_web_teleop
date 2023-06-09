import { ComponentType, VideoStreamId, ButtonPadId, VideoStreamDef, ButtonPadDef, TabsDef, SingleTabDef, LayoutDefinition, ActionMode, OverheadVideoStreamDef, RealsenseVideoStreamDef } from "../utils/component_definitions";

/**
 * Layout from Stretch to the Client paper
 */
export const STRETCH2CLIENT_LAYOUT: LayoutDefinition = {
    type: ComponentType.Layout,
    displayVoiceControl: true,
    actionMode: ActionMode.StepActions,
    children: [
        {
            type: ComponentType.Panel,
            children: [
                {
                    type: ComponentType.SingleTab,
                    label: 'Buttons Interface',
                    children: [
                        {
                            type: ComponentType.ButtonGrid
                        }
                    ]
                },
                {
                    type: ComponentType.SingleTab,
                    label: 'Variety Interface',
                    children: [
                        // Realsense camera
                        {
                            type: ComponentType.VirtualJoystick
                        },
                        {
                            type: ComponentType.ButtonGrid
                        }
                    ]
                }
            ]
        } as TabsDef
    ]
}