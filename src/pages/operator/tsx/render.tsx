/**
 * @summary Functions for rendering components based on their definitions.
 */

import React from "react";
// TODO: these streams need to be accessed through webrtc
import { navigationStream, realsenseStream, gripperStream } from "robot/tsx";
import { UserInteractionFunction as BF, ButtonPad, ButtonPadProps, ButtonPadShape, ButtonProps } from "./buttonpads";
import { VideoStream, VideoStreamComponent } from "./videostreams";
import { CustomizableComponent, CustomizableComponentProps, SharedState } from "./customizablecomponent";
import { ButtonPadId, VideoStreamDef, VideoStreamId, ComponentDef } from "./componentdefinitions";
import { DropZone } from "./dropzone";

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
        path: props.path + '-0',
        sharedState: props.sharedState
    }
    const buttonPad = buttonPadDef ? renderButtonPad(buttonPadProps, props.definition) : undefined;
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
    components: ComponentDef[],
    sharedState: SharedState,
    parentDef?: ComponentDef
}

export const ComponentList = (props: ComponentListProps) => {
    const { path, components } = props;
    return (
        <>
            {components.map((compDef: ComponentDef, index: number) => {
                const curPath = (path ? path + "-" : "") + `${index}`;
                const cProps: CustomizableComponentProps = {
                    definition: compDef,
                    path: curPath,
                    sharedState: props.sharedState
                }
                return (
                    <React.Fragment key={`${compDef.id}-${index}`} >
                        <DropZone
                            path={curPath}
                            sharedState={props.sharedState}
                            parentDef={props.parentDef}
                        />
                        <CustomizableComponent {...cProps} />
                    </React.Fragment>
                );
            })}
            <DropZone
                path={(path ? path + "-" : "") + components.length}
                sharedState={props.sharedState}
                parentDef={props.parentDef}
            />
        </>
    )
}
