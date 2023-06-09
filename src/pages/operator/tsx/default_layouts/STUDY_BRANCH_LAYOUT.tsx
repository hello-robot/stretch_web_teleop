import { ComponentType, CameraViewId, ButtonPadId, CameraViewDefinition, ButtonPadDefinition, PanelDefinition, TabDefinition, LayoutDefinition, ActionMode, OverheadVideoStreamDef, RealsenseVideoStreamDef } from "../utils/component_definitions";

/**
 * Default layout to load on start
 */
export const STUDY_BRANCH_LAYOUT: LayoutDefinition = {
    type: ComponentType.Layout,
    displayVoiceControl: false,
    actionMode: ActionMode.StepActions,
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
                            type: ComponentType.CameraView,
                            id: CameraViewId.realsense,
                            followGripper: false,
                            depthSensing: false,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.Drive,
                                } as ButtonPadDefinition
                            ]
                        } as RealsenseVideoStreamDef
                    ]
                },
                {
                    type: ComponentType.SingleTab,
                    label: 'Manipulation',
                    children: [
                        {
                            type: ComponentType.CameraView,
                            id: CameraViewId.overhead,
                            gripperView: true,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.ManipOverhead
                                }
                            ]
                        } as OverheadVideoStreamDef,
                        {
                            type: ComponentType.CameraView,
                            id: CameraViewId.realsense,
                            followGripper: false,
                            depthSensing: false,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.ManipRealsense,
                                } as ButtonPadDefinition
                            ]
                        } as RealsenseVideoStreamDef,
                        {
                            type: ComponentType.CameraView,
                            id: CameraViewId.gripper,
                            children: [
                                {
                                    type: ComponentType.ButtonPad,
                                    id: ButtonPadId.Gripper
                                }
                            ]
                        } as CameraViewDefinition
                    ]
                } as TabDefinition
            ]
        } as PanelDefinition
    ]
}