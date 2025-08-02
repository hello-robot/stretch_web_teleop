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
import "operator/css/ExecutionMonitorLayout.css";
import { DropZone } from "../layout_components/DropZone";

/** Properties for {@link LayoutArea} */
type LayoutAreaProps = {
    /** Layout structure to render */
    layout: LayoutDefinition;
    sharedState: SharedState;
};

/** Main area of the interface where the user can add, remove, or rearrange elements. */
export const LayoutArea = (props: LayoutAreaProps) => {
    // Check if this is the ExecutionMonitor layout by looking for ExecutionMonitor components
    const hasExecutionMonitor = (layout: LayoutDefinition): boolean => {
        for (const grid of layout.children) {
            for (const panel of grid.children) {
                for (const tab of panel.children) {
                    for (const component of tab.children) {
                        if (component.type === "Execution Monitor") {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    };

    const isExecutionMonitorLayout = hasExecutionMonitor(props.layout);
    const layoutClass = isExecutionMonitorLayout ? "execution-monitor-layout" : "";

    const panelColumn = props.layout.children;
    const dropZoneIdx = 0;
    return (
        <div className={layoutClass}>
            {panelColumn.map((compDef: LayoutGridDefinition, index: number) => {
                return (
                    // compDef.children.length > 0 ?
                    <>
                        <DropZone
                            path={`${index}`}
                            sharedState={props.sharedState}
                            parentDef={props.layout}
                        />
                        <div id="layout-area">
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
        </div>
    );
};
