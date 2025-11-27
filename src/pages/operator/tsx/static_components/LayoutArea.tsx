import React from "react";
import {
    ComponentDefinition,
    LayoutDefinition,
    LayoutGridDefinition,
    PanelDefinition,
    ParentComponentDefinition,
} from "operator/tsx/utils/component_definitions";
import { SharedState } from "../layout_components/CustomizableComponent";
import {
    ComponentList,
    ComponentListProps,
} from "../layout_components/ComponentList";
import "operator/css/LayoutArea.css";
import { DropZone } from "../layout_components/DropZone";

/** Properties for {@link LayoutArea} */
type LayoutAreaProps = {
    /** Layout structure to render */
    layout: LayoutDefinition;
    sharedState: SharedState;
};

/** Main area of the interface where the user can add, remove, or rearrange elements. */
export const LayoutArea = (props: LayoutAreaProps) => {
    // const componentListProps: ComponentListProps = {
    //     definition: props.layout,
    //     path: "",
    //     sharedState: props.sharedState
    // }
    const panelColumn = props.layout.children;
    const dropZoneIdx = 0;
    return (
        <>
            {panelColumn.map((compDef: LayoutGridDefinition, index: number) => {
                return (
                    // compDef.children.length > 0 ?
                    <>
                        <DropZone
                            path={`${index}`}
                            sharedState={props.sharedState}
                            parentDef={props.layout}
                        />
                        <div className="layout-area">
                            <ComponentList
                                key={"layout-grid-" + `${index}`}
                                {...({
                                    definition: compDef,
                                    path: `${index}`,
                                    sharedState: props.sharedState,
                                } as ComponentListProps)}
                            />
                        </div>
                    </>
                    // :
                    // <></>
                );
            })}
            <DropZone
                path={`${props.layout.children.length}`}
                sharedState={props.sharedState}
                parentDef={props.layout}
            />
        </>
    );
};
