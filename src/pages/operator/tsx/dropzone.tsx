import React from "react";
import "operator/css/dropzone.css"
import { ComponentDef, ComponentType } from "./componentdefinitions";
import { SharedState } from "./customizablecomponent";

/** State required for drop zones */
export type DropZoneState = {
    onDrop: DropHandler,
    activeDef?: ComponentDef
}

export type DropZoneProps = {
    path: string,
    parentDef?: ComponentDef,
    sharedState: SharedState
}

export type DropHandler = (path: string) => void

/** Area where the user can click to move the selected component to a new place. */
export const DropZone = (props: DropZoneProps) => {
    const canDrop = (): boolean => {
        const { path, parentDef } = props;
        const { activePath } = props.sharedState;
        const { activeDef } = props.sharedState.dropZoneState;
        // If no active object selected
        if (!activePath || !activeDef) {
            return false;
        }

        // Can place anything at the top level of the heiarchy
        if (!parentDef) {
            return true;
        }

        // Tab -> any other object is not allowed
        if (activeDef.type === ComponentType.Tabs) {
            return false;
        }

        // Paths cannot be adjacent
        const splitItemPath = activePath.split('-');
        const splitSelfPath = path.split('-');
        const sameLength = splitItemPath.length == splitSelfPath.length;

        const itemPrefix = splitItemPath.slice(0, -1);
        const selfPrefix = splitSelfPath.slice(0, -1);
        const matchingPrefix = sameLength && itemPrefix.every((val, index) => val === selfPrefix[index]);

        const itemLast = +splitItemPath.slice(-1)[0];
        const selfLast = +splitSelfPath.slice(-1)[0];
        const adjacent = matchingPrefix && (itemLast == selfLast || itemLast + 1 == selfLast)
        if (adjacent) {
            return false;
        }
        return true;
    }

    const handleClick = () => {
        if (!props.sharedState.customizing) return;
        props.sharedState.dropZoneState.onDrop(props.path);
    }

    const isActive = props.sharedState.customizing && canDrop();
    return (
        <div 
            className="drop-zone"
            hidden={!isActive}
            onClick={handleClick}
        >

        </div>
    )
}