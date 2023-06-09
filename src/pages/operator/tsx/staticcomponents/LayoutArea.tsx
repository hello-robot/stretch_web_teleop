import React from "react";
import { ParentComponentDefinition } from 'operator/tsx/utils/componentdefinitions'
import { SharedState } from "../layoutcomponents/CustomizableComponent";
import { ComponentList, ComponentListProps } from "../layoutcomponents/ComponentList";
import "operator/css/LayoutArea.css"

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


