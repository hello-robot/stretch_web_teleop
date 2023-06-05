import { ComponentType, VideoStreamId, ButtonPadId, VideoStreamDef, ButtonPadDef, TabsDef, SingleTabDef, LayoutDefinition, ActionMode, OverheadVideoStreamDef, RealsenseVideoStreamDef } from "../utils/componentdefinitions";

/**
 * Layout from Stretch to the Client paper
 */
export const STRETCH2CLIENT_LAYOUT: LayoutDefinition = {
    type: ComponentType.Layout,
    displayVoiceControl: false,
    actionMode: ActionMode.StepActions,
    children: [
        {
            type: ComponentType.Tabs,
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
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.realsense,
                            followGripper: false,
                            depthSensing: false,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.Drive,
                                } as ButtonPadDef
                            ]
                        } as RealsenseVideoStreamDef,
                        {
                            type: ComponentType.ButtonGrid
                        }
                    ]
                }
            ]
        } as TabsDef
    ]
}