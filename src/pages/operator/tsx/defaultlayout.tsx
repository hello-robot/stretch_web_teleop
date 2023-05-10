import { ComponentType, VideoStreamId, ButtonPadId, VideoStreamDef, ButtonPadDef, TabsDef, SingleTabDef, ParentComponentDefinition, ComponentDefinition } from "./componentdefinitions";

/**
 * Default layout to load on start
 */
export const DEFAULT_LAYOUT: ParentComponentDefinition = {
    type: ComponentType.Layout,
    children: [{
        type: ComponentType.Tabs,
        children: [{
            type: ComponentType.SingleTab,
            label: 'Manipulation',
            children: [{
                type: ComponentType.VideoStream,
                id: VideoStreamId.overhead,
                children: [
                    {
                        type: ComponentType.ButtonPad,
                        id: ButtonPadId.overhead
                    }
                ]
            } as VideoStreamDef,
            {
                type: ComponentType.VideoStream,
                id: VideoStreamId.realsense,
                children: [
                    {
                        type: ComponentType.ButtonPad,
                        id: ButtonPadId.realsense,
                    } as ButtonPadDef
                ]
            } as VideoStreamDef,
            {
                type: ComponentType.VideoStream,
                id: VideoStreamId.gripper,
                children: [
                    {
                        type: ComponentType.ButtonPad,
                        id: ButtonPadId.gripper
                    }
                ]
            } as VideoStreamDef,
            ]
        } as SingleTabDef,
        {
            type: ComponentType.SingleTab,
            label: 'Navigation',
            children: [
            ]
        }]
    } as TabsDef,
    {
        type: ComponentType.Tabs,
        children: [{
            type: ComponentType.SingleTab,
            label: 'Tab1',
            children: [
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
            type: ComponentType.SingleTab,
            label: 'Tab2',
            children: [{
                type: ComponentType.VideoStream,
                id: VideoStreamId.gripper,
                children: [{
                    type: ComponentType.ButtonPad,
                    id: ButtonPadId.gripper,
                } as ButtonPadDef]
            } as VideoStreamDef]
        }]
    } as TabsDef]
}