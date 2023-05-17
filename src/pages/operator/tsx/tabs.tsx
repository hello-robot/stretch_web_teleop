import React from "react";
import { ComponentType, ParentComponentDefinition, SingleTabDef, TabsDef } from "./componentdefinitions"
import { ComponentList, ComponentListProps } from "./render";
import "../css/tabs.css"
import { CustomizableComponentProps } from "./customizablecomponent";
import { ChangeEvent } from "react";

export const Tabs = (props: CustomizableComponentProps) => {
    // Index of the active tab
    const [active, setActive] = React.useState(0);
    const definition = props.definition as TabsDef;
    const activeTabDef = definition.children[active] as ParentComponentDefinition;
    const [showTabModal, setShowTabModal] = React.useState(false);
    if (activeTabDef.type != ComponentType.SingleTab) {
        throw new Error(`Tabs element at path ${props.path} has child of type ${activeTabDef.type}`)
    }
    // Should take up screen size proportional to number of children
    const flex = Math.max(activeTabDef.children.length, 1);
    /** Props for rendering the children elements inside the selected tab */
    const componentListProps: ComponentListProps = {
        path: props.path + '-' + active,
        sharedState: props.sharedState,
        // Use active tab as the definition for what to render
        definition: activeTabDef
    }
    /**
     * Creates a definition for the new tab and adds it to the layout
     * @param name the label for the new tab
     */
    const addTab = (name: string) => {
        // Define new tab
        const newTabDef = {
            type: ComponentType.SingleTab,
            label: name,
            children: []
        } as SingleTabDef;
        // Add it as a new child
        definition.children.push(newTabDef);
        // Set as selected element (and rerender)
        props.sharedState.onSelect(newTabDef, undefined);
    }

    return (
        <div className="tabs-component" style={{ flex: `${flex} ${flex} 0` }}>
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
                {props.sharedState.customizing ?
                    <button
                        className="tab-button material-icons"
                        onClick={() => setShowTabModal(true)}
                    >
                        add_circle
                    </button> : undefined}
            </div>
            <div className="tabs-content">
                <ComponentList {...componentListProps} />
            </div>
            {showTabModal ?
                <NewTabModal
                    setShow={setShowTabModal}
                    addTab={addTab} />
                : undefined
            }
        </div>
    )
}

/** Modal for selecting a new tab */
const NewTabModal = (props: {
    setShow: (show: boolean) => void,
    addTab: (name: string) => void
}) => {
    const [text, setText] = React.useState("");
    /** Hides the popup and adds the new tab */
    const onAccept = () => {
        const newLabel = text.length > 0 ? text : 'new tab';
        props.setShow(false);
        props.addTab(newLabel);
    }
    /** Records new label typed into the box */
    const handleChange = (e: ChangeEvent<HTMLInputElement>) => {
        setText(e.target.value);
    }
    return (
        <>
            <div id="new-tab-modal">
                <label htmlFor="new-tab-name"><b>New Tab Label</b></label>
                <input type="text" id="new-tab-name" name="new-tab-name"
                    value={text} onChange={handleChange}
                    placeholder="label for the new tab"
                />
                <div id="bottom-buttons">
                    <button id="cancel" onClick={() => props.setShow(false)}>Cancel</button>
                    <button id="accept" onClick={onAccept} style={{ float: "right" }}>Accept</button>
                </div>
            </div>
            <div onClick={() => props.setShow(false)} id="popup-background"></div>
        </>
    )
}