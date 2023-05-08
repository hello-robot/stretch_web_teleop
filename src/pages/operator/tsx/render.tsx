/**
 * @summary Functions for rendering components based on their definitions.
 */
import React from "react";
// TODO: these streams need to be accessed through webrtc
import { navigationStream, realsenseStream, gripperStream } from "robot/tsx";
import { ExampleButtonPads } from "./buttonpads";
import { CompDef, TabsDef, ButtonPadId, VideoStreamDef, VideoStreamId, ComponentType, ButtonPadDef } from "./componentdefinitions";
import { Tabs } from "./tabs";
import { VideoStream, VideoStreamComponent } from "./videostreams";


function renderTabs(tabsDef: TabsDef): React.ReactNode {
    return <Tabs tabsDef={tabsDef} />
}

function renderButtonPad(buttonPadDef: ButtonPadDef): React.ReactNode {
    switch (buttonPadDef.id) {
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
    return <VideoStreamComponent stream={stream} buttonPad={buttonPad} />
}

/**
 * Takes a definition for a component and returns the react component.
 * @note switch on the component definition's `type` field
 * @param comp component definition to render
 * @returns rendered component
 */
export function renderComponent(comp: CompDef): React.ReactNode {
    switch (comp.type) {
        case ComponentType.Tabs:
            return renderTabs(comp as TabsDef)
        case ComponentType.ButtonPad:
            return renderButtonPad(comp as ButtonPadDef);
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
