import React from "react";
import "../css/layoutarea.css"
import { ComponentDef } from 'operator/tsx/componentdefinitions'
import { renderComponentList } from "./render";
import { DEFAULT_LAYOUT } from "./defaultlayout"
import { FunctionProvider } from "./functionprovider";

/** Properties for {@link LayoutArea} */
interface LayoutAreaProps {
    /** If the interface is in customization mode */
    customizing: boolean;
    /** Layout structure to render */
    layout?: ComponentDef[];
    functionProvider: FunctionProvider;
}

/** Main area of the interface where the user can add, remove, or rearrange elements. */
export const LayoutArea = (props: LayoutAreaProps) => {
    const layout: ComponentDef[] = props.layout || DEFAULT_LAYOUT;
    return (
        <div id="layout-area" >
            {renderComponentList(layout, props.customizing, props.functionProvider)}
        </div>
    )
}


