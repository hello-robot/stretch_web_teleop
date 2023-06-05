import { ComponentDefinition, ComponentType } from "../utils/componentdefinitions";
import { DropZoneState } from "./dropzone";
import { Tabs } from "./tabs";
import { RemoteStream } from "shared/util";
import { ButtonPad } from "./buttonpads";
import { VideoStreamComponent } from "./videostreamcomponent";
import { PredictiveDisplay } from "./predictivedisplay";
import { ButtonStateMap } from "../functionprovider/buttonpads";
import { ButtonGrid } from "./ButtonGrid";

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
    /** Mapping of each button pad function to a {@link ButtonState} */
    buttonStateMap?: ButtonStateMap
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
        case ComponentType.ButtonGrid:
            return <ButtonGrid {...props} />;
        default:
            throw Error(`CustomizableComponent cannot render component of unknown type: ${props.definition.type}\nYou may need to add a case for this component in the switch statement in CustomizableComponent.`);
    }
}