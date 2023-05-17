/**
 * @summary Functions for rendering components based on their definitions.
 */

import React from "react";
import { UserInteractionFunction as BF, ButtonPad, ButtonPadProps, ButtonPadShape, ButtonProps, UserInteractionFunction } from "./layoutcomponents/buttonpads";
import { VideoStreamComponent } from "./layoutcomponents/videostreams";
import { CustomizableComponent, CustomizableComponentProps, SharedState } from "./layoutcomponents/customizablecomponent";
import { ButtonPadId, VideoStreamDef, VideoStreamId, ComponentDefinition, ParentComponentDefinition, ComponentType } from "./utils/componentdefinitions";
import { DropZone } from "./layoutcomponents/dropzone";
import { PredictiveDisplay } from "./layoutcomponents/predictivedisplay";

export function renderButtonPad(cProps: CustomizableComponentProps, videoStreamParent?: VideoStreamDef) {
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
        case ButtonPadId.PredictiveDisplay:
            return (
            <PredictiveDisplay 
                {...cProps}
                onClick={(len, ang) => console.log('length', len, 'angle', ang) }
            />
            )
        default:
            throw new Error(`unknow video stream id: ${cProps.definition.id}`);
    }
    const buttonsProps = functions.map((funct: BF) => {
        return {
            ...cProps.sharedState.functionProvider(funct),
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

export function renderVideoStream(props: CustomizableComponentProps) {
    let stream: MediaStream;
    switch (props.definition.id as VideoStreamId) {
        case VideoStreamId.overhead:
            stream = props.sharedState.remoteStreams.get('overhead')!.stream;
            break;
        case VideoStreamId.realsense:
            stream = props.sharedState.remoteStreams.get('realsense')!.stream;
            break;
        case VideoStreamId.gripper:
            stream = props.sharedState.remoteStreams.get('gripper')!.stream;
            break;
        default:
            throw new Error(`unknow video stream id: ${props.definition.id}`);
    }

    // Render button pad overlay
    const videoStreamDef = props.definition as VideoStreamDef;
    
    // Check video stream has at most one child
    if (videoStreamDef.children.length > 1) throw new Error(`VideoStream definition should not have more than one child`);
    let buttonPad: JSX.Element | undefined = undefined;
    if (videoStreamDef.children.length > 0) {
        const buttonPadDef = (props.definition as VideoStreamDef).children[0];
        
        // Check video stream child is of type button pad
        if (buttonPadDef.type != ComponentType.ButtonPad) throw new Error(`VideoStream component cannot have component type ${buttonPadDef.type} as a child`);
        const buttonPadProps: CustomizableComponentProps = {
            definition: buttonPadDef,
            path: props.path + '-0',
            sharedState: props.sharedState
        }
        buttonPad = renderButtonPad(buttonPadProps, props.definition as VideoStreamDef);
    }

    // let videoStream = <VideoControl key={stream.id} stream={stream}/>
    return (
        <VideoStreamComponent
            stream={stream}
            buttonPad={buttonPad}
            {...props}
        />
    );
}

export type ComponentListProps = {
    path: string,
    sharedState: SharedState,
    definition: ParentComponentDefinition
}

export const ComponentList = (props: ComponentListProps) => {
    const { path } = props;
    const components = props.definition.children;
    return (
        <>
            {components.map((compDef: ComponentDefinition, index: number) => {
                const curPath = (path ? path + "-" : "") + `${index}`;
                const cProps: CustomizableComponentProps = {
                    definition: compDef,
                    path: curPath,
                    sharedState: props.sharedState
                }
                return (
                    <React.Fragment key={`${index}`} >
                        <DropZone
                            path={curPath}
                            sharedState={props.sharedState}
                            parentDef={props.definition}
                        />
                        <CustomizableComponent {...cProps} />
                    </React.Fragment>
                );
            })}
            <DropZone
                path={(path ? path + "-" : "") + components.length}
                sharedState={props.sharedState}
                parentDef={props.definition}
            />
        </>
    )
}
