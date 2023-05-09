import React from "react";
import { VelocityControl, DEFAULT_SPEED } from "operator/tsx/velocitycontrol"
import { LayoutArea } from "./layoutarea";
import { ActionMode, ActionModeButton } from "operator/tsx/actionmodebutton"
import "operator/css/operator.css"
import { mockFunctionProvider } from "./functionprovider";
import { UserInteractionFunction } from "./buttonpads";
import { CustomizeButton } from "./customizebutton";
import { Sidebar } from "./sidebar";
import { SharedState } from "./customizablecomponent";
import { ComponentDef, ComponentType, TabsDef } from "./componentdefinitions";
import { DEFAULT_LAYOUT } from "./defaultlayout";


// TODO: finish this
const getParent = (splitPath: string[], layout: ComponentDef[]) => {
    let currentComponents: ComponentDef[] = layout;
    let pathIdx = 0;
    let parent: ComponentDef;
    while (pathIdx < splitPath.length -1) {
        const compIdx = +splitPath[pathIdx];
        const comp = currentComponents[compIdx];
        switch (comp.type) {
            case (ComponentType.Tabs): 
                pathIdx++;
                const tabIdx = +splitPath[pathIdx];
                currentComponents = (comp as TabsDef).tabs[tabIdx].contents;
                parent = comp;
                break;
            case (ComponentType.VideoStream):
                // Should be at end of path
                if (pathIdx != splitPath.length - 2) {
                    throw new Error(`reached video stream before end of path ${splitPath}`)
                }
                parent = comp;
                break;
            default:
                throw new Error(`reached getParent loop with type ${comp.type}`)
        }
    }
    return parent!;
}

// TODO: finish this
const moveInLayout = (oldPath: string, newPath: string, layout: ComponentDef[]) => {
    const oldPathSplit = oldPath.split('-');
    let compDef: ComponentDef;
    let oldParent: ComponentDef | undefined;
    if (oldPathSplit.length < 2) {
        compDef = layout[+oldPathSplit[0]];
    } else {
        oldParent = getParent(oldPathSplit, layout);
        
    }
}

/** Operator interface webpage */
export const Operator = () => {
    /** Speed of the robot. */
    let speed = DEFAULT_SPEED;
    let actionMode = ActionMode.StepActions;
    const [layout, setLayout] = React.useState(DEFAULT_LAYOUT);
    const [customizing, setCustomizing] = React.useState(false);
    const [activePath, setActivePath] = React.useState<string | undefined>();
    const [activeDef, setActiveDef] = React.useState<ComponentDef | undefined>();

    /**
     * Callback when the user clicks on a drop zone, moves the active component
     * into the drop zone
     * @param path path to the clicked drop zone
     */
    const handleDrop = (path: string) => {
        console.log("handleDrop", path);
    }

    /**
     * Callback when a component is selected during customization
     * @param path path to the selected component
     * @param def definition of the selected component
     */
    const handleSelect = (path: string, def: ComponentDef) => {
        if (!customizing) return;
        if (activePath == path) {
            setActiveDef(undefined);
            setActivePath(undefined);
            return;
        }
        setActiveDef(def);
        setActivePath(path);
    }

    /**
     * Callback when the customization button is clicked.
     */
    const handleCustomize = () => {
        setCustomizing(!customizing);
        setActiveDef(undefined);
        setActivePath(undefined);
    }

    /** State passed from the operator and shared by all components */
    const sharedState: SharedState = {
        customizing: customizing,
        onSelect: handleSelect,
        functionProvider: (bf: UserInteractionFunction) => mockFunctionProvider(actionMode, bf),
        activePath: activePath,
        dropZoneState: {
            onDrop: handleDrop,
            activeDef: activeDef
        }
    }

    return (
        <div id="operator">
            <div id="operator-header">
                <ActionModeButton
                    default={actionMode}
                    onChange={(newAm) => (actionMode = newAm)}
                />
                <VelocityControl
                    initialSpeed={speed}
                    onChange={(newSpeed) => (speed = newSpeed)}
                />
                <CustomizeButton
                    customizing={customizing}
                    onClick={handleCustomize}
                />
            </div>
            <div id="operator-body">
                <LayoutArea
                    layout={layout}
                    sharedState={sharedState}
                />
                <Sidebar hidden={!customizing} />
            </div>
        </div>
    )
}