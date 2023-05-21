import React from "react";
import "operator/css/layoutarea.css"
import { ParentComponentDefinition } from 'operator/tsx/utils/componentdefinitions'
import { ComponentList, ComponentListProps } from "operator/tsx/layoutcomponents/componentlist";
import { SharedState } from "./layoutcomponents/customizablecomponent";

/** Properties for {@link LayoutArea} */
type LayoutAreaProps = {
    /** Layout structure to render */
    layout: ParentComponentDefinition;
    sharedState: SharedState;
}

/** Main area of the interface where the user can add, remove, or rearrange elements. */
export const LayoutArea = (props: LayoutAreaProps) => {
    const componentListProps: ComponentListProps = {
        definition: props.layout,
        path: "",
        sharedState: props.sharedState
    }
    return (
        <div id="layout-area" >
            <ComponentList {...componentListProps} />
        </div>
    )
}


