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
import { ComponentDefinition, ParentComponentDefinition } from "./componentdefinitions";
import { DEFAULT_LAYOUT } from "./defaultlayout";
import { RemoteRobot } from "robot/tsx/remoterobot";
import { RemoteStream } from "utils/util";

const getParent = (splitPath: string[], layout: ParentComponentDefinition): ParentComponentDefinition => {
    let pathIdx = 0;
    let parent: ParentComponentDefinition = layout;
    while (pathIdx < splitPath.length - 1) {
        const childIdx = +splitPath[pathIdx];
        parent = parent.children[childIdx] as ParentComponentDefinition;
        pathIdx++;
    }
    return parent!;
}

const getChildFromParent = (parent: ParentComponentDefinition, childIdx: number): ComponentDefinition => {
    return parent.children[childIdx];
}

const putChildInParent = (parent: ParentComponentDefinition, child: ComponentDefinition, childIdx: number) => {
    parent.children.splice(childIdx, 0, child);
}

const removeChildFromParent = (parent: ParentComponentDefinition, childIdx: number) => {
    parent.children.splice(childIdx, 1);
}

const moveInLayout = (oldPath: string, newPath: string, layout: ParentComponentDefinition): string => {
    // Get the child and its old parent
    console.log('old path', oldPath);
    console.log('newpath', newPath);
    const oldPathSplit = oldPath.split('-');
    const oldParent = getParent(oldPathSplit, layout);
    console.log('old parent', oldParent);
    let oldChildIdx = +oldPathSplit.slice(-1);
    console.log('old child index', oldChildIdx);
    const temp = getChildFromParent(oldParent, oldChildIdx);
    console.log('temp', temp)

    // Get the new parent
    let newPathSplit = newPath.split('-');
    const newChildIdx = +newPathSplit.slice(-1);
    console.log('newChildIdx', newChildIdx)
    const newParent = getParent(newPathSplit, layout);
    console.log('newparent', newParent)

    // Put the child into the new parent
    putChildInParent(newParent, temp, newChildIdx);
    console.log('after adding child', newParent.children);

    if (oldParent === newParent && oldChildIdx > newChildIdx)
        oldChildIdx++;

    // Remove the child from the old parent
    removeChildFromParent(oldParent, oldChildIdx);

    // Check if removing the old path changes the new path
    // note: this happens when the old path was a sibling with a lower index to
    //       any node in the 
    if (newPathSplit.length < oldPathSplit.length)
        return newPath;

    const oldPathLastIdx = oldPathSplit.length - 1;
    const oldPrefix = oldPathSplit.slice(0, oldPathLastIdx);
    const newPrefix = newPathSplit.slice(0, oldPathLastIdx)
    const sameParent = oldPrefix.every((val, index) => val === newPrefix[index])

    if (!sameParent)
        return newPath;

    // index of new sibling node
    const newCorrespondingIdx = +newPathSplit[oldPathLastIdx];
    if (oldChildIdx < newCorrespondingIdx) {
        // decrease new path index since the old path is deleted
        newPathSplit[oldPathLastIdx] = "" + (+newPathSplit[oldPathLastIdx] - 1);
        console.log('updated path', newPathSplit.join('-'))
    }
    return newPathSplit.join('-');
}

/** Operator interface webpage */
export const Operator = (props: {
        remoteRobot: RemoteRobot,
        remoteStreams: Map<string, RemoteStream> 
    }) => {
    /** Speed of the robot. */
    let speed = DEFAULT_SPEED;
    let actionMode = ActionMode.StepActions;
    const [layout, setLayout] = React.useState(DEFAULT_LAYOUT);
    const [customizing, setCustomizing] = React.useState(false);
    const [activePath, setActivePath] = React.useState<string | undefined>();
    const [activeDef, setActiveDef] = React.useState<ComponentDefinition | undefined>();

    let remoteRobot = props.remoteRobot
    let remoteStreams = props.remoteStreams
    
    /**
     * Callback when the user clicks on a drop zone, moves the active component
     * into the drop zone
     * @param path path to the clicked drop zone
     */
    const handleDrop = (path: string) => {
        console.log("handleDrop", path);
        const newPath = moveInLayout(activePath!, path, layout);
        setActivePath(newPath);
        console.log('new active path', newPath)
        setLayout(layout);
    }

    /**
     * Callback when a component is selected during customization
     * @param path path to the selected component
     * @param def definition of the selected component
     */
    const handleSelect = (path: string, def: ComponentDefinition) => {
        console.log('selected', path)
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
        remoteStreams: remoteStreams,
        functionProvider: (bf: UserInteractionFunction) => mockFunctionProvider(actionMode, bf, remoteRobot),
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