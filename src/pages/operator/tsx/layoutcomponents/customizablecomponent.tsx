import { ComponentDefinition, ComponentType } from "../utils/componentdefinitions";
import { DropZoneState } from "./dropzone";
import { Tabs } from "./tabs";
import { RemoteStream, ValidJointStateDict } from "shared/util";
import { ButtonPad } from "./buttonpads";
import { VideoStreamComponent } from "./videostreams";
import { PredictiveDisplay } from "./predictivedisplay";

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
    /**
     * Path to the component
     * @example "0-2" would represent the 2nd child of the 0th element in the layout
     */
    path: string,
    /** 
     * Definition of the component (all the info required to know that type 
     * of component to render
     */
    definition: ComponentDefinition;
    /** see {@link SharedState} */
    sharedState: SharedState,
}

/**
 * Takes a definition for a component and returns the react component.
 * 
 * @note switch on the component definition's `type` field
 * @returns rendered component
 */
export const CustomizableComponent = (props: CustomizableComponentProps) => {
    if (!props.definition.type) {
        throw new Error(`Component at ${props.path} is missing type`);
    }

    // switch on the component type to render specific type of component
    switch (props.definition.type) {
        case ComponentType.Tabs:
            return <Tabs {...props} />
        case ComponentType.ButtonPad:
            return <ButtonPad {...props} />;
        case ComponentType.VideoStream:
            return <VideoStreamComponent {...props} />;
        case ComponentType.PredictiveDisplay:
            return <PredictiveDisplay {...props} />;
        default:
            throw Error(`unknow component type: ${props.definition.type}`);
    }
}