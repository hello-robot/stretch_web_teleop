import { ComponentType, VideoStreamId, ButtonPadId, VideoStreamDef, ButtonPadDef, TabsDef, SingleTabDef, LayoutDefinition, ActionMode, OverheadVideoStreamDef, RealsenseVideoStreamDef } from "../utils/componentdefinitions";

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
                            gripperView: false,
                            children: [
                                {
                                    // type: ComponentType.PredictiveDisplay,
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.Drive
                                }
                            ]
                        } as OverheadVideoStreamDef,
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
                        } as RealsenseVideoStreamDef
                    ]
                },
                {
                    type: ComponentType.SingleTab,
                    label: 'Manipulation',
                    children: [
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.overhead,
                            gripperView: true,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.ManipOverhead
                                }
                            ]
                        } as OverheadVideoStreamDef,
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.realsense,
                            followGripper: false,
                            depthSensing: false,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.ManipRealsense,
                                } as ButtonPadDef
                            ]
                        } as RealsenseVideoStreamDef,
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