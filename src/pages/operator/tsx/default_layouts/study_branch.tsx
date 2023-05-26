import { ComponentType, VideoStreamId, ButtonPadId, VideoStreamDef, ButtonPadDef, TabsDef, SingleTabDef, LayoutDefinition, ActionMode } from "../utils/componentdefinitions";

/**
 * Default layout to load on start
 */
export const STUDY_BRANCH_LAYOUT: LayoutDefinition = {
    type: ComponentType.Layout,
    displayVoiceControl: false,
    actionMode: ActionMode.StepActions,
    children: [
        {
            type: ComponentType.Tabs,
            children: [
                {
                    type: ComponentType.SingleTab,
                    label: 'Navigation',
                    children: [
                        // Overhead camera
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.overhead,
                            children: [
                                {
                                    type: ComponentType.PredictiveDisplay,
                                    // type: ComponentType.ButtonPad,
                                    // id: ButtonPadId.overhead
                                }
                            ]
                        } as VideoStreamDef,
                        // Realsense camera
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.realsense,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.Drive,
                                } as ButtonPadDef
                            ]
                        } as VideoStreamDef
                    ]
                },
                {
                    type: ComponentType.SingleTab,
                    label: 'Manipulation',
                    children: [
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.overhead,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.ManipOverhead
                                }
                            ]
                        } as VideoStreamDef,
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.realsense,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.ManipRealsense,
                                } as ButtonPadDef
                            ]
                        } as VideoStreamDef,
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.gripper,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.Gripper
                                }
                            ]
                        } as VideoStreamDef
                    ]
                } as SingleTabDef
            ]
        } as TabsDef
    ]
}