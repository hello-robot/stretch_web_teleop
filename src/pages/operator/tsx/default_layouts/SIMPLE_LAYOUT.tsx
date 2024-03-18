import { ComponentType, CameraViewId, ButtonPadId, CameraViewDefinition, ButtonPadDefinition, PanelDefinition, TabDefinition, LayoutDefinition, ActionMode, LayoutGridDefinition } from "../utils/component_definitions";

/**
 * Basic Layout
 */
export const BASIC_LAYOUT: LayoutDefinition = {
    type: ComponentType.Layout,
    displayMovementRecorder: false,
    displayLabels: true,
    actionMode: ActionMode.PressAndHold,
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
                                    displayButtons: true,
                                    children: []
                                } as CameraViewDefinition,
                                {
                                    type: ComponentType.CameraView,
                                    id: CameraViewId.gripper,
                                    displayButtons: true,
                                    children: []
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
                                    displayIcons: true
                                } as ButtonPadDefinition
                            ]
                        } as TabDefinition,
                        {
                            type: ComponentType.SingleTab,
                            label: 'Wrist & Gripper',
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.DexWrist,
                                    displayIcons: true
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
                                    displayIcons: true
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