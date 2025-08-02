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
 * Execution Monitor Layout
 */
export const EXECUTION_MONITOR_LAYOUT: LayoutDefinition = {
    type: ComponentType.Layout,
    displayMovementRecorder: false,
    displayTextToSpeech: false,
    displayLabels: true,
    actionMode: ActionMode.PressAndHold,
    children: [
        {
            type: ComponentType.LayoutGrid,
            flex: 0.25, 
            children: [
                {
                    type: ComponentType.Panel,
                    children: [
                        {
                            type: ComponentType.SingleTab,
                            label: "Camera Views",
                            children: [
                                // Overhead camera
                                {
                                    type: ComponentType.CameraView,
                                    id: CameraViewId.overhead,
                                    displayButtons: true,
                                    children: [],
                                } as CameraViewDefinition,
                                {
                                    type: ComponentType.CameraView,
                                    id: CameraViewId.gripper,
                                    displayButtons: true,
                                    children: [],
                                } as CameraViewDefinition,
                            ],
                        },
                    ],
                } as PanelDefinition,
            ],
        } as LayoutGridDefinition,
        {
            type: ComponentType.LayoutGrid,
            flex: 2,
            children: [
                {
                    type: ComponentType.Panel,
                    children: [
                        {
                            type: ComponentType.SingleTab,
                            label: "Execution Monitor",
                            children: [
                                {
                                    type: ComponentType.ExecutionMonitor,
                                },
                            ],
                        },
                    ],
                } as PanelDefinition,
            ],
        } as LayoutGridDefinition,
        {
            type: ComponentType.LayoutGrid,
            flex: 0.5, 
            children: [
                {
                    type: ComponentType.Panel,
                    children: [
                        {
                            type: ComponentType.SingleTab,
                            label: "Wrist & Gripper",
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.DexWrist,
                                } as ButtonPadDefinition,
                            ],
                        } as TabDefinition,
                        {
                            type: ComponentType.SingleTab,
                            label: "Arm & Lift",
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.Arm,
                                } as ButtonPadDefinition,
                            ],
                        } as TabDefinition,
                    ],
                } as PanelDefinition,
                {
                    type: ComponentType.Panel,
                    children: [
                        {
                            type: ComponentType.SingleTab,
                            label: "Safety",
                            children: [
                                {
                                    type: ComponentType.RunStopButton,
                                },
                                {
                                    type: ComponentType.BatteryGuage,
                                },
                            ],
                        },
                    ],
                } as PanelDefinition,
            ],
        } as LayoutGridDefinition,
    ],
}; 