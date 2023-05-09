import React from "react";
import { SingleTabDef, TabsDef } from "./componentdefinitions"
import { ComponentList, ComponentListProps } from "./render";
import "../css/tabs.css"
import { CustomizableComponentProps } from "./customizablecomponent";

export const Tabs = (props: CustomizableComponentProps) => {
    // Index of the active tab
    const [active, setActive] = React.useState(0);
    const definition = props.definition as TabsDef;
    const componentListProps: ComponentListProps = {
        path: props.path,
        components: definition.tabs[active].contents,
        sharedState: props.sharedState,
        parentDef: props.definition
    }
    return (
        <div className="tabs-component" >
            <div className="tabs-header">
                {definition.tabs.map((tabDef: SingleTabDef, idx: number) => {
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
                <ComponentList {...componentListProps}/>
            </div>
        </div>
    )
}