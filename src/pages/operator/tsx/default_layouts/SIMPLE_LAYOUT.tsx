import { ComponentType, CameraViewId, ButtonPadId, CameraViewDefinition, ButtonPadDefinition, PanelDefinition, TabDefinition, LayoutDefinition, ActionMode, LayoutGridDefinition } from "../utils/component_definitions";

/**
 * Basic Layout
 */
export const BASIC_LAYOUT: LayoutDefinition = {
    type: ComponentType.Layout,
    displayVoiceControl: false,
    displayPoseLibrary: false,
    displayMovementRecorder: false,
    displayArucoMarkers: false,
    displayLabels: true,
    actionMode: ActionMode.PressRelease,
    children: [
        {
            type: ComponentType.LayoutGrid,
            children: [
                {
                    type: ComponentType.Panel,
                    children: [
                        {
                            type: ComponentType.SingleTab,
                            label: 'Camera Views',
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
                } as PanelDefinition
            ]    
        } as LayoutGridDefinition,
        {
            type: ComponentType.LayoutGrid,
            children: [
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
                            label: 'Wrist & Gripper',
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.WristGripper,
                                } as ButtonPadDefinition
                            ]
                        } as TabDefinition,
                        {
                            type: ComponentType.SingleTab,
                            label: 'Arm & Lift',
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.Arm,
                                } as ButtonPadDefinition
                            ]
                        } as TabDefinition
                    ]
                } as PanelDefinition,
                {
                    type: ComponentType.Panel,
                    children: [
                        {
                            type: ComponentType.SingleTab,
                            label: 'Safety',
                            children: [
                                {
                                    type: ComponentType.RunStopButton
                                },
                                {
                                    type: ComponentType.BatteryGuage
                                }
                            ]
                        }
                    ] 
                } as PanelDefinition
            ]
        }  as LayoutGridDefinition
    ]
}