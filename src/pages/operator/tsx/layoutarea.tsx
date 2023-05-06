import React from "react";
import "../css/layoutarea.css"
import { ButtonPadDef, ButtonPadId, CompDef, ComponentType, TabsDef, VideoStreamDef, VideoStreamId } from 'operator/tsx/componentdefinitions'
import { renderComponentList } from "./render";

const DropZone = ({ }) => {
    return (
        <div className="drop-zone" >
        </div>
    )
}

const defaultLayout: CompDef[] = [
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
    } as TabsDef
]

interface LayoutAreaProps {
    customizing: boolean,
    layout?: CompDef[],
    buttonPadFunctions: any
}

export class LayoutArea extends React.Component {

    layout: CompDef[];

    constructor(props: LayoutAreaProps) {
        super(props);
        this.layout = props.layout || defaultLayout;
    }

    render(): React.ReactNode {
        return (
            <div id="layout-area" >
                {renderComponentList(this.layout)}
            </div>
        )
    }
}


