import React from "react";
import "../css/layoutarea.css"
import { Tabs } from './tabs'
import { ButtonPadDef, ButtonPadId, CompDef, ComponentType, SingleTabDef, TabsDef, VideoStreamDef, VideoStreamId } from './interfacecomponents'
import { gripperStream, navigationStream, realsenseStream } from "./index";
import { VideoStream, VideoStreamComponent } from "./videostreams";
import { ExampleButtonPads } from "./buttonpads";

interface LayoutAreaProps {
    customizing: boolean,
    layout?: CompDef[],
    buttonPadFunctions: any
}

const DropZone = ({ }) => {
    return (
        <div className="drop-zone" >
        </div>
    )
}

function renderTabs(comp: CompDef): React.ReactNode {
    return <Tabs tabsDef={comp as TabsDef} />
}

function renderButtonPad(comp: CompDef): React.ReactNode {
    switch (comp.id as ButtonPadId) {
        case ButtonPadId.overhead:
            return ExampleButtonPads[0];
        case ButtonPadId.realsense:
            return ExampleButtonPads[1];
        case ButtonPadId.gripper:
            return ExampleButtonPads[2];
        default:
            return <div>ERROR</div>;
    }
}

function renderVideoStream(videoDef: VideoStreamDef): React.ReactNode {
    let stream: VideoStream;
    switch (videoDef.id as VideoStreamId) {
        case VideoStreamId.overhead:
            stream = navigationStream;
            break;
        case VideoStreamId.realsense:
            stream = realsenseStream;
            break;
        case VideoStreamId.gripper:
            stream = gripperStream;
            break;
        default:
            return <div>ERROR</div>
    }
    const buttonPadDef = videoDef.buttonPadDef;
    const buttonPad = buttonPadDef ? renderButtonPad(buttonPadDef) : undefined;
    return <VideoStreamComponent stream={stream} buttonPad={buttonPad}/>
}

/**
 * Takes a definition for a component and returns the react component.
 * @note switch on the component definition's `type` field
 * @param comp component definition to render
 * @returns rendered component
 */
function renderComponent(comp: CompDef): React.ReactNode {
    switch (comp.type) {
        case ComponentType.Tabs:
            return renderTabs(comp)
        case ComponentType.ButtonPad:
            return renderButtonPad(comp);
        case ComponentType.VideoStream:
            return renderVideoStream(comp as VideoStreamDef);
        default:
            console.error(`unknow component type: ${comp.type}`)
            return <div>ERROR</div>;
    }
}

export function renderComponentList(comps: CompDef[], path?: string) {
    return comps.map((comp: CompDef, index: number) => {
        const curPath = (path ? path + "-" : "") + `${index}`;
        return (
            <React.Fragment key={`${comp.id}-${index}`} >
                {/* <DropZone /> */}
                {renderComponent(comp)}
            </React.Fragment>
        );
    });
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

export class LayoutArea extends React.Component {

    layout: CompDef[];

    constructor(props: LayoutAreaProps) {
        super(props);
        this.layout = props.layout || defaultLayout;
        this.state = {
            customizing: props.customizing
        };
    }

    render(): React.ReactNode {
        return (
            <div id="layout-area" >
                {renderComponentList(this.layout)}
            </div>
        )
    }
}


