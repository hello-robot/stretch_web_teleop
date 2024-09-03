import React from "react";
import {
    ComponentType,
    ParentComponentDefinition,
    TabDefinition,
    PanelDefinition,
} from "../utils/component_definitions";
import { className } from "shared/util";
import { PopupModal } from "../basic_components/PopupModal";
import { ComponentListProps, ComponentList } from "./ComponentList";
import { DropZone } from "./DropZone";
import {
    CustomizableComponentProps,
    isSelected,
} from "./CustomizableComponent";
import "operator/css/Panel.css";
import AddCircleIcon from "@mui/icons-material/AddCircle";

/*
TODO:
implement behavior:
- delete of all tabs deletes the tabs element
*/

/**
 * Customizable tabs component which displays the labels of the tabs at the top,
 * and renders the contents of the active tab inside.
 * @param props {@link CustomizableComponentProps}
 */
export const Panel = (props: CustomizableComponentProps) => {
    // Index of the active tab
    let [activeTab, setActiveTab] = React.useState(0);
    // If should show the popup to name a new tab
    const [showTabModal, setShowTabModal] = React.useState(false);
    const definition = props.definition as PanelDefinition;
    const countChildren = definition.children.length;

    // Handle case where active tab was moved or deleted, just use last remaining tab
    if (activeTab >= countChildren) {
        setActiveTab(countChildren - 1);
        activeTab = countChildren - 1;
    }

    const activeTabDef = definition.children[activeTab] as TabDefinition;
    if (!activeTabDef) {
        throw Error(
            `Tabs at: ${props.path}\nActive tab not defined\nActive tab: ${activeTab}`,
        );
    }
    if (activeTabDef.type != ComponentType.SingleTab) {
        throw Error(
            `Tabs element at path ${props.path} has child of type ${activeTabDef.type}`,
        );
    }

    // Should take up screen size proportional to number of children
    let flex = Math.max(activeTabDef.children.length + 1, 1);
    if (activeTabDef.label === "Safety") flex = 1;
    else if (
        activeTabDef.children.some((c) => c.type === ComponentType.ButtonGrid)
    ) {
        flex = 3;
    }

    /** Props for rendering the children elements inside the active tab */
    const componentListProps: ComponentListProps = {
        path: props.path + "-" + activeTab,
        sharedState: props.sharedState,
        // Use active tab as the definition for what to render
        definition: activeTabDef,
    };

    /**
     * Creates a definition for the new tab and adds it to the layout
     * @param name the label for the new tab
     */
    function addTab(name: string) {
        // Define new tab
        const newTabDef = {
            type: ComponentType.SingleTab,
            label: name,
            children: [],
        } as TabDefinition;
        // Add it as a new child
        definition.children.push(newTabDef);
        // Set as selected element (and rerender)
        props.sharedState.onSelect(newTabDef, undefined);
    }

    /**
     * Callback when a tab label is clicked on. Sets the tab as active. If already
     * active and in customize mode, selects the tab for customization.
     * @param idx index of the clicked tab
     */
    function clickTab(idx: number) {
        // If customizing and tab already active
        if (props.sharedState.customizing && idx === activeTab) {
            // Mark tab as selected
            const tabPath = props.path + "-" + idx;
            props.sharedState.onSelect(definition.children[idx], tabPath);
            return;
        }
        // Set tab as active
        setActiveTab(idx);
    }

    /**
     * Handle click on tab content during customization mode. Marks this entire
     * tabs component as selected.
     */
    function selectContent() {
        props.sharedState.onSelect(props.definition, props.path);
    }

    // Add onClick listener to tab content in customization mode
    const selectProp = props.sharedState.customizing
        ? {
              onClick: selectContent,
          }
        : {};

    /**
     * Checks if this tabs or one of its immediate children is currently selected
     *
     * @returns null if currently selected component is not either this tabs
     * or one of it's immediate SingleTab children, -1 if the selected component
     * is this entire tabs component, or the index of the selected single tab
     * child.
     */
    function checkChildTabSelected(): number | null {
        const selectedPath = props.sharedState.selectedPath;
        if (!selectedPath) return null; // nothing is selected/active
        const activeSplitPath = selectedPath.split("-");
        const thisSplitPath = props.path.split("-");
        const activeChild = thisSplitPath.every(
            (val, index) => val === activeSplitPath[index],
        );
        if (!activeChild) return null; // active path is not a child element
        // The paths are exactly the same, the entire Tabs structure is selected
        if (activeSplitPath.length == thisSplitPath.length) return -1;
        // Path points to a child of a tab
        if (activeSplitPath.length - 1 > thisSplitPath.length) return null;
        // Return the child index
        return +activeSplitPath.slice(-1);
    }
    const childTabSelected: number | null = checkChildTabSelected();

    /**
     * Maps children list to a set of buttons with labels for switching tabs
     * @param tabDef definition of the child single tab component
     * @param idx index of the child component in the children array
     * @returns A button to switch tabs
     */
    function mapTabLabels(tabDef: TabDefinition, idx: number) {
        const active = activeTab === idx;
        const selected = childTabSelected === idx;
        return (
            <React.Fragment key={`${idx}`}>
                <DropZone
                    path={props.path + "-" + idx}
                    sharedState={props.sharedState}
                    parentDef={props.definition}
                />
                <button
                    key={`${tabDef.label}-${idx}`}
                    className={className("tab-button", { active, selected })}
                    onClick={() => clickTab(idx)}
                >
                    {tabDef.label}
                </button>
            </React.Fragment>
        );
    }

    const thisSelected = childTabSelected === -1;

    return (
        <div
            className={className("tabs-component", {
                customizing: props.sharedState.customizing,
                selected: thisSelected,
            })}
            style={{ flex: `${flex} ${flex} 0` }}
        >
            <div className="tabs-header">
                {definition.children.map(mapTabLabels)}
                <DropZone
                    path={(props.path ? props.path + "-" : "") + countChildren}
                    sharedState={props.sharedState}
                    parentDef={props.definition}
                />
                {
                    // In customization mode show an extra plus to add a new tab
                    props.sharedState.customizing ? (
                        <button
                            className="tab-button add-tab"
                            onClick={() => setShowTabModal(true)}
                        >
                            <AddCircleIcon />
                        </button>
                    ) : undefined
                }
            </div>
            <div className="tabs-content" {...selectProp}>
                <ComponentList {...componentListProps} />
            </div>
            <NewTabModal
                show={showTabModal}
                setShow={setShowTabModal}
                addTab={addTab}
            />
        </div>
    );
};

/** Modal for creating a new tab on a panel component. */
const NewTabModal = (props: {
    show: boolean;
    setShow: (show: boolean) => void;
    addTab: (name: string) => void;
}) => {
    const [text, setText] = React.useState("");
    function handleAccept() {
        if (text.length > 0) {
            props.addTab(text);
        }
    }

    return (
        <PopupModal
            setShow={props.setShow}
            show={props.show}
            onAccept={handleAccept}
            id="new-tab-modal"
            acceptButtonText="Create Tab"
            acceptDisabled={text.length < 1}
        >
            <label htmlFor="new-tab-name">
                <b>New Tab Label</b>
            </label>
            <input
                autoFocus
                type="text"
                id="new-tab-name"
                name="new-tab-name"
                value={text}
                onChange={(e) => setText(e.target.value)}
                placeholder="label for the new tab"
            />
        </PopupModal>
    );
};
