import { ComponentDef, ComponentType, VideoStreamId, ButtonPadId, VideoStreamDef, ButtonPadDef, TabsDef } from "./componentdefinitions";

/**
 * Default layout to load on start
 */
export const DEFAULT_LAYOUT: ComponentDef[] = [
    // Video stream tabs
    {
        type: ComponentType.Tabs,
        id: 'tabs',
        tabs: [{
            label: 'Manipulation',
            contents: [
                {
                    type: ComponentType.VideoStream,
                    id: VideoStreamId.overhead,
                    buttonPadDef: {
                        type: ComponentType.ButtonPad,
                        id: ButtonPadId.overhead
                    }
                } as VideoStreamDef,
                {
                    type: ComponentType.VideoStream,
                    id: VideoStreamId.realsense,
                    buttonPadDef: {
                        type: ComponentType.ButtonPad,
                        id: ButtonPadId.realsense,
                    } as ButtonPadDef
                } as VideoStreamDef,
                {
                    type: ComponentType.VideoStream,
                    id: VideoStreamId.gripper,
                    buttonPadDef: {
                        type: ComponentType.ButtonPad,
                        id: ButtonPadId.gripper,
                    } as ButtonPadDef
                } as VideoStreamDef,
            ]
        },
        {
            label: 'Navigation',
            contents: [
        {
                    type: ComponentType.VideoStream,
                    id: VideoStreamId.overhead
                } as VideoStreamDef,
                {
                    type: ComponentType.VideoStream,
                    id: VideoStreamId.realsense
                } as VideoStreamDef,
            ]
        }]
    } as TabsDef,
    {
        type: ComponentType.Tabs,
        id: 'tabs',
        tabs: [{
            label: 'Tab1',
            contents: [
                // {
                //     type: ComponentType.VideoStream,
                //     id: VideoStreamId.overhead,
                //     buttonPadDef: {
                //         type: ComponentType.ButtonPad,
                //         id: ButtonPadId.overhead
                //     }
                // } as VideoStreamDef,
                // {
                //     type: ComponentType.VideoStream,
                //     id: VideoStreamId.realsense,
                //     buttonPadDef: {
                //         type: ComponentType.ButtonPad,
                //         id: ButtonPadId.realsense,
                //     } as ButtonPadDef
                // } as VideoStreamDef,
                // {
                //     type: ComponentType.VideoStream,
                //     id: VideoStreamId.gripper,
                //     buttonPadDef: {
                //         type: ComponentType.ButtonPad,
                //         id: ButtonPadId.gripper,
                //     } as ButtonPadDef
                // } as VideoStreamDef,
            ]
        },
        {
            label: 'Tab2',
            contents: []
        }]
    } as TabsDef
]