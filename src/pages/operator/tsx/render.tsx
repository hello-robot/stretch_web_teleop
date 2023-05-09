/**
 * @summary Functions for rendering components based on their definitions.
 */

import React from "react";
// TODO: these streams need to be accessed through webrtc
import { navigationStream, realsenseStream, gripperStream } from "robot/tsx";
import { UserInteractionFunction as BF, ButtonPad, ButtonPadProps, ButtonPadShape, ButtonProps } from "./buttonpads";
import { Tabs } from "./tabs";
import { VideoStream, VideoStreamComponent } from "./videostreams";
import { CustomizableComponentProps } from "./customizablecomponent";
import { ButtonPadId, VideoStreamDef, VideoStreamId, ComponentType, ComponentDef } from "./componentdefinitions";
import { FunctionProvider } from "./functionprovider";


function renderTabs(props: CustomizableComponentProps) {
    return (
        <Tabs {...props} />
    );
}

function renderButtonPad(cProps: CustomizableComponentProps, videoStreamParent?: VideoStreamDef) {
    let functions: BF[];
    let shape: ButtonPadShape;
    switch (cProps.definition.id) {
        case ButtonPadId.overhead:
            functions = [
                BF.BaseForward,
                BF.BaseRotateRight,
                BF.BaseReverse,
                BF.BaseRotateLeft
            ];
            shape = ButtonPadShape.Directional;
            break;
        case ButtonPadId.realsense:
            functions = [
                BF.WristRotateIn,
                BF.WristRotateOut,
                BF.ArmExtend,
                BF.ArmRetract,
                BF.BaseForward,
                BF.BaseReverse,
                BF.ArmLift,
                BF.ArmLower,
                BF.GripperClose,
                BF.GripperOpen
            ]
            shape = ButtonPadShape.Realsense;
            break;
        case ButtonPadId.gripper:
            functions = [
                BF.ArmLift,
                BF.ArmLower,
                BF.WristRotateIn,
                BF.WristRotateOut,
                BF.GripperOpen,
                BF.GripperClose,
            ]
            shape = ButtonPadShape.Gripper;
            break;
        default:
            throw new Error(`unknow video stream id: ${cProps.definition.id}`);
    }
    const buttonsProps = functions.map((funct: BF) => {
        return {
            ...cProps.functionProvider(funct),
            label: "" + funct
        } as ButtonProps;
    })
    const props: ButtonPadProps = {
        ...cProps,
        buttonsProps: buttonsProps,
        buttonPadShape: shape,
        videoStreamParent: videoStreamParent
    }
    return <ButtonPad {...props} />
}

function renderVideoStream(props: CustomizableComponentProps) {
    let stream: VideoStream;
    switch (props.definition.id as VideoStreamId) {
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
            throw new Error(`unknow video stream id: ${props.definition.id}`);
    }
    const buttonPadDef = props.definition;
    const buttonPadProps: CustomizableComponentProps = {
        definition: buttonPadDef,
        customizing: props.customizing,
        path: props.path + '-0',
        functionProvider: props.functionProvider
    }
    const buttonPad = buttonPadDef ? renderButtonPad(buttonPadProps, props.definition) : undefined;
    return <VideoStreamComponent stream={stream} buttonPad={buttonPad} />
}

/**
 * Takes a definition for a component and returns the react component.
 * @note switch on the component definition's `type` field
 * @param comp component definition to render
 * @returns rendered component
 */
export function renderComponent(props: CustomizableComponentProps) {
    switch (props.definition.type) {
        case ComponentType.Tabs:
            return renderTabs(props)
        case ComponentType.ButtonPad:
            return renderButtonPad(props);
        case ComponentType.VideoStream:
            return renderVideoStream(props);
        default:
            throw new Error(`unknow component type: ${props.definition.type}`);
    }
}

/**
 * Renders a list of component definitions horizontally
 * @param comps list of component definitions to render
 * @param customizing if the interface is in customization mode
 * @param functionProvider see {@link FunctionProvider}
 * @param path 
 * @returns 
 */
export function renderComponentList(comps: ComponentDef[], customizing: boolean, functionProvider: FunctionProvider, path?: string) {
    return comps.map((compDef: ComponentDef, index: number) => {
        const curPath = (path ? path + "-" : "") + `${index}`;
        const props: CustomizableComponentProps = {
            definition: compDef,
            customizing: customizing,
            path: curPath,
            functionProvider: functionProvider
        }
        return (
            <React.Fragment key={`${compDef.id}-${index}`} >
                {/* <DropZone/> */}
                {renderComponent(props)}
            </React.Fragment>
        );
    });
}
