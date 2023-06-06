import { ComponentType, VideoStreamId, ButtonPadId, VideoStreamDef, ButtonPadDef, TabsDef, SingleTabDef, LayoutDefinition, ActionMode } from "../utils/componentdefinitions";

/**
 * Layout from the main branch
 */
export const MAIN_BRANCH_LAYOUT: LayoutDefinition = {
    type: ComponentType.Layout,
    displayVoiceControl: false,
    actionMode: ActionMode.ClickClick,
    children: [
        {
            type: ComponentType.Panel,
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
                                // {
                                //     type: ComponentType.ButtonPad,
                                //     id: ButtonPadId.ManipOverhead
                                // }
                            ]
                        } as VideoStreamDef,
                        // Realsense camera
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.realsense,
                            children: [
                                // {
                                //     type: ComponentType.ButtonPad,
                                //     id: ButtonPadId.Drive,
                                // } as ButtonPadDef
                            ]
                        } as VideoStreamDef,
                        // Gripper camera
                        // {
                        //     type: ComponentType.VideoStream,
                        //     id: VideoStreamId.gripper,
                        //     children: [
                        //         // {
                        //         //     type: ComponentType.ButtonPad,
                        //         //     id: ButtonPadId.Gripper
                        //         // }
                        //     ]
                        // } as VideoStreamDef
                    ]
                },
                {
                    type: ComponentType.SingleTab,
                    label: 'Manipulation',
                    children: [
                        // Overhead camera
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.overhead,
                            children: [
                                // {
                                //     type: ComponentType.PredictiveDisplay,
                                //     // type: ComponentType.ButtonPad,
                                //     // id: ButtonPadId.overhead
                                // }
                            ]
                        } as VideoStreamDef,
                        // Realsense camera
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.realsense,
                            children: [
                                // {
                                //     type: ComponentType.ButtonPad,
                                //     id: ButtonPadId.Drive,
                                // } as ButtonPadDef
                            ]
                        } as VideoStreamDef,
                        // Gripper camera
                        {
                            type: ComponentType.VideoStream,
                            id: VideoStreamId.gripper,
                            children: [
                                // {
                                //     type: ComponentType.ButtonPad,
                                //     id: ButtonPadId.Gripper
                                // }
                            ]
                        } as VideoStreamDef
                    ]
                }
            ]
        } as TabsDef,
        {
            type: ComponentType.Panel,
            children: [
                {
                    type: ComponentType.SingleTab,
                    label: 'Base',
                    children: [
                        {
                            type: ComponentType.ButtonPad,
                            id: ButtonPadId.Base,
                        } as ButtonPadDef
                    ]
                } as SingleTabDef,
                {
                    type: ComponentType.SingleTab,
                    label: 'Camera',
                    children: [
                        {
                            type: ComponentType.ButtonPad,
                            id: ButtonPadId.Camera,
                        } as ButtonPadDef
                    ]
                } as SingleTabDef,
                {
                    type: ComponentType.SingleTab,
                    label: 'Wrist',
                    children: [
                        {
                            type: ComponentType.ButtonPad,
                            id: ButtonPadId.Wrist,
                        } as ButtonPadDef
                    ]
                } as SingleTabDef,
                {
                    type: ComponentType.SingleTab,
                    label: 'Arm',
                    children: [
                        {
                            type: ComponentType.ButtonPad,
                            id: ButtonPadId.Arm,
                        } as ButtonPadDef
                    ]
                } as SingleTabDef
            ]
        } as TabsDef
    ]
}