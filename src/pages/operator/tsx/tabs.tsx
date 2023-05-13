import React from "react";
import { ComponentType, ParentComponentDefinition, SingleTabDef, TabsDef } from "./componentdefinitions"
import { ComponentList, ComponentListProps } from "./render";
import "../css/tabs.css"
import { CustomizableComponentProps } from "./customizablecomponent";

export const Tabs = (props: CustomizableComponentProps) => {
    // Index of the active tab
    const [active, setActive] = React.useState(0);
    const definition = props.definition as TabsDef;
    const activeTabDef = definition.children[active] as ParentComponentDefinition;
    if (activeTabDef.type != ComponentType.SingleTab) {
        throw new Error(`Tabs element at path ${props.path} has child of type ${activeTabDef.type}`)
    }
    const flex = Math.max(activeTabDef.children.length, 1);
    const componentListProps: ComponentListProps = {
        path: props.path + '-' + active,
        sharedState: props.sharedState,
        // Use active tab as the definition for what to render
        definition: activeTabDef
    }
    return (
        <div className="tabs-component" style={{flex: `${flex} ${flex} 0`}}>
            <div className="tabs-header">
                {definition.children.map((tabDef: SingleTabDef, idx: number) => {
                    const isActive = active === idx;
                    return (
                        <button
                            key={`${tabDef.label}-${idx}`}
                            className={"tab-button" + (isActive ? " active" : "")}
                            onClick={() => setActive(idx)}
                        >
                            {tabDef.label}
                        </button>
                    );
                })
                }
            </div>
            <div className="tabs-content">
                <ComponentList {...componentListProps} />
            </div>
        </div>
    )
}