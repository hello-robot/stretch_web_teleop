import { ComponentType, CameraViewId, ButtonPadId, CameraViewDefinition, ButtonPadDefinition, PanelDefinition, TabDefinition, LayoutDefinition, ActionMode } from "../utils/component_definitions";

/**
 * Layout from the main branch
 */
export const MAIN_BRANCH_LAYOUT: LayoutDefinition = {
    type: ComponentType.Layout,
    displayVoiceControl: false,
    displayPoseLibrary: true,
    displayPoseRecorder: true,
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
                            type: ComponentType.CameraView,
                            id: CameraViewId.overhead,
                            children: [
                                // {
                                //     type: ComponentType.ButtonPad,
                                //     id: ButtonPadId.ManipOverhead
                                // }
                            ]
                        } as CameraViewDefinition,
                        // Realsense camera
                        {
                            type: ComponentType.CameraView,
                            id: CameraViewId.realsense,
                            children: [
                                // {
                                //     type: ComponentType.ButtonPad,
                                //     id: ButtonPadId.Drive,
                                // } as ButtonPadDef
                            ]
                        } as CameraViewDefinition,
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
                            type: ComponentType.CameraView,
                            id: CameraViewId.overhead,
                            children: [
                                // {
                                //     type: ComponentType.PredictiveDisplay,
                                //     // type: ComponentType.ButtonPad,
                                //     // id: ButtonPadId.overhead
                                // }
                            ]
                        } as CameraViewDefinition,
                        // Realsense camera
                        {
                            type: ComponentType.CameraView,
                            id: CameraViewId.realsense,
                            children: [
                                // {
                                //     type: ComponentType.ButtonPad,
                                //     id: ButtonPadId.Drive,
                                // } as ButtonPadDef
                            ]
                        } as CameraViewDefinition,
                        // Gripper camera
                        {
                            type: ComponentType.CameraView,
                            id: CameraViewId.gripper,
                            children: [
                                // {
                                //     type: ComponentType.ButtonPad,
                                //     id: ButtonPadId.Gripper
                                // }
                            ]
                        } as CameraViewDefinition
                    ]
                }
            ]
        } as PanelDefinition,
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
                        } as ButtonPadDefinition
                    ]
                } as TabDefinition,
                {
                    type: ComponentType.SingleTab,
                    label: 'Camera',
                    children: [
                        {
                            type: ComponentType.ButtonPad,
                            id: ButtonPadId.Camera,
                        } as ButtonPadDefinition
                    ]
                } as TabDefinition,
                {
                    type: ComponentType.SingleTab,
                    label: 'Wrist',
                    children: [
                        {
                            type: ComponentType.ButtonPad,
                            id: ButtonPadId.Wrist,
                        } as ButtonPadDefinition
                    ]
                } as TabDefinition,
                {
                    type: ComponentType.SingleTab,
                    label: 'Arm',
                    children: [
                        {
                            type: ComponentType.ButtonPad,
                            id: ButtonPadId.Arm,
                        } as ButtonPadDefinition
                    ]
                } as TabDefinition
            ]
        } as PanelDefinition
    ]
}