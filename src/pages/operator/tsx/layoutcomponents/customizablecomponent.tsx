import { ComponentDefinition, ComponentType } from "../utils/componentdefinitions";
import { DropZoneState } from "./dropzone";
import { FunctionProvider } from "../utils/functionprovider";
import { renderButtonPad, renderVideoStream } from "../render";
import { Tabs } from "./tabs";
import { RemoteStream, ValidJointStateDict } from "shared/util";

/** State required for all elements */
export type SharedState = {
    customizing: boolean,
    /** Called when user clicks on a component */
    onSelect: (def: ComponentDefinition, path?: string) => void,
    /** Remote robot video streams */
    remoteStreams: Map<string, RemoteStream>
    /** State required for all dropzones */
    dropZoneState: DropZoneState,
    /** Path to the active component */
    activePath?: string,
    /** Whether joints are in limits */
    inJointLimits?: ValidJointStateDict
    /** Whether joints are in collision */
    inCollision?: ValidJointStateDict
};

/** Properties for any of the customizable components: tabs, video streams, or
 * button pads.
 */
export type CustomizableComponentProps = {
    path: string,
    definition: ComponentDefinition;
    sharedState: SharedState,
}

/**
 * Takes a definition for a component and returns the react component.
 * @note switch on the component definition's `type` field
 * @returns rendered component
 */
export const CustomizableComponent = (props: CustomizableComponentProps) => {
    if (!props.definition.type) {
        throw new Error(`Component at ${props.path} is missing type`);
    }
    switch (props.definition.type) {
        case ComponentType.Tabs:
            return <Tabs {...props} />
        case ComponentType.ButtonPad:
            return renderButtonPad(props);
        case ComponentType.VideoStream:
            return renderVideoStream(props);
        default:
            throw new Error(`unknow component type: ${props.definition.type}`);
    }
}