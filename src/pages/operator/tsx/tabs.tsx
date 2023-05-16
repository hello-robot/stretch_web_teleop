import React from "react";
import { ComponentType, ParentComponentDefinition, SingleTabDef, TabsDef } from "./componentdefinitions"
import { ComponentList, ComponentListProps } from "./render";
import "../css/tabs.css"
import { CustomizableComponentProps } from "./customizablecomponent";
import { Tab, Tabs, Card } from 'react-bootstrap';
import "bootstrap/dist/css/bootstrap.min.css";

export const InterfaceTabs = (props: CustomizableComponentProps) => {
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
        <Card border="light">
            <Tabs defaultActiveKey="Manipulation" justify>
                    {definition.children.map((tabDef: SingleTabDef, idx: number) => {
                        const isActive = active === idx;
                        return (
                            // <Card border="light" style={{ width: '18rem' }}>
                            <Tab eventKey={`${tabDef.label}`} title={tabDef.label}>
                                <div className="tabs-content">
                                    <ComponentList {...componentListProps} />
                                </div>
                            </Tab>
                            // </Card>
                        )
                    })}
            </Tabs>
        </Card>
        // <div className="tabs-z" style={{flex: `${flex} ${flex} 0`}}>
        //     <div className="tabs-header">
        //         {definition.children.map((tabDef: SingleTabDef, idx: number) => {
        //             const isActive = active === idx;
        //             return (
        //                 <button
        //                     key={`${tabDef.label}-${idx}`}
        //                     className={"tab-button" + (isActive ? " active" : "")}
        //                     onClick={() => setActive(idx)}
        //                 >
        //                     {tabDef.label}
        //                 </button>
        //             );
        //         })
        //         }
        //     </div>
        //     <div className="tabs-content">
        //         <ComponentList {...componentListProps} />
        //     </div>
        // </div>
    )
}