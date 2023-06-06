import React from "react";
import { CustomizableComponent, CustomizableComponentProps, SharedState } from "./customizablecomponent";
import { DropZone } from "./dropzone";
import { ParentComponentDefinition, ComponentDefinition } from "../utils/componentdefinitions";

/** Properties for {@link ComponentList} */
export type ComponentListProps = {
    /** Path of the container element (e.g. the path to the tabs structure rendering) */
    path: string,
    sharedState: SharedState,
    definition: ParentComponentDefinition
}

/**
 * Creates a {@link CustomizableComponent}s list with {@link DropZone}s between.
 * 
 * @param props {@link ComponentListProps}
 */
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
