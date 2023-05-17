import React from "react";
import "operator/css/dropzone.css"
import { ComponentDefinition, ComponentType } from "../utils/componentdefinitions";
import { SharedState } from "./customizablecomponent";
import { className } from "shared/util";

/** State required for drop zones */
export type DropZoneState = {
    onDrop: DropHandler,
    activeDef?: ComponentDefinition
}

export type DropZoneProps = {
    path: string,
    parentDef: ComponentDefinition,
    sharedState: SharedState
}

export type DropHandler = (path: string) => void

/** Area where the user can click to move the selected component to a new place. */
export const DropZone = (props: DropZoneProps) => {
    function canDrop(): boolean {
        const { path, parentDef } = props;
        const { activePath } = props.sharedState;
        const { activeDef } = props.sharedState.dropZoneState;
        // If no active object selected
        if (!activePath || !activeDef) {
            return false;
        }

        // Tabs can only go into layout
        if (activeDef.type === ComponentType.Tabs && parentDef.type !== ComponentType.Layout) {
            return false;
        }

        // Single tab can only go into tabs
        if (activeDef.type === ComponentType.SingleTab && parentDef.type !== ComponentType.Tabs) {
            return false;
        }

        // Only single tab can go into tabs
        if (activeDef.type !== ComponentType.SingleTab && parentDef.type === ComponentType.Tabs) {
            return false;
        }

        // Only button pad can go into video stream
        if (activeDef.type !== ComponentType.ButtonPad && parentDef.type === ComponentType.VideoStream) {
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

    /** Calls onDrop function from Operator with the path of this dropzone */
    function handleClick (e: React.MouseEvent<HTMLSpanElement>) {
        if (!props.sharedState.customizing) return;
        props.sharedState.dropZoneState.onDrop(props.path);
        e.stopPropagation();
    }

    const isActive = props.sharedState.customizing && canDrop();
    const inTab = props.parentDef.type === ComponentType.Tabs;
    const overlay = props.parentDef.type === ComponentType.VideoStream;
    const standard = !(inTab || overlay);
    return (
        <span
            className={className("drop-zone material-icons", {tab: inTab, overlay, standard})}
            hidden={!isActive}
            onClick={handleClick}
        >
            vertical_align_bottom
        </span>
    )
}