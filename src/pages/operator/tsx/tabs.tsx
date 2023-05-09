import React from "react";
import { SingleTabDef, TabsDef } from "./componentdefinitions"
import { renderComponentList } from "./render";
import "../css/tabs.css"
import { CustomizableComponentProps } from "./customizablecomponent";

interface TabsProps extends CustomizableComponentProps {}

export const Tabs = (props: TabsProps) => {
    // Index of the active tab
    const [active, setActive] = React.useState(0);
    const definition = props.definition as TabsDef;
    const contents = renderComponentList(definition.tabs[active].contents, props.customizing, props.functionProvider, props.path)
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
                {contents}
            </div>
        </div>
    )
}