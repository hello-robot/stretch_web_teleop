import React from "react";
import "operator/css/dropzone.css"
import { ComponentDefinition, ComponentType } from "../utils/componentdefinitions";
import { SharedState } from "./customizablecomponent";
import { className } from "shared/util";

/** State required for drop zones */
export type DropZoneState = {
    /**
     * Callback when the DropZone is clicked to handle dropping the active 
     * selected component into the position of the drop zone
     * 
     * @param path path to the drop zone
     */
    onDrop: (path: string) => void,
    /** Definition of the active selected component */
    activeDef?: ComponentDefinition
}

/** Properties for {@link DropZone} */
export type DropZoneProps = {
    /** Path to the drop zone, same as CustomizableComponent path */
    path: string,
    /** Definition of the CustomizableComponent containing this drop zone */
    parentDef: ComponentDefinition,
    /** State shared between all components */
    sharedState: SharedState
}

/**
 * Area where the user can click to move the selected component to a new place.
 * 
 * @param props {@link DropZoneProps}
 */
export const DropZone = (props: DropZoneProps) => {
    function canDrop(): boolean {
        const { path, parentDef } = props;
        const { activePath } = props.sharedState;
        const { activeDef } = props.sharedState.dropZoneState;

        // If no active object selected
        if (!activeDef) {
            return false;
        }

        // Must pass drop zone rules about which components can go where
        if (!dropzoneRules(activeDef.type, parentDef.type)) return false;

        // Don't need to check if dropzone is adjacent if the active component
        // is coming from the sidebar component provider
        if (!activePath) return true;

        // Can't drop if dropzone is right next to the active element
        // (that wouldn't move the active element at all)
        return !pathsAdjacent(activePath, path);
    }

    /** Calls onDrop function from Operator with the path of this dropzone */
    function handleClick(e: React.MouseEvent<HTMLSpanElement>) {
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
            className={className("drop-zone material-icons", { tab: inTab, overlay, standard })}
            hidden={!isActive}
            onClick={handleClick}
        >
            vertical_align_bottom
        </span>
    )
}

/**
 * Checks if the active component is allowed to be placed in this drop zone
 * 
 * @param active type of the active component the user selected
 * @param parent type of the parent component containing this drop zone
 * @returns true if the active component is allowed to be dropped, false otherwise
 */
function dropzoneRules(active: ComponentType, parent: ComponentType) {
    // Tabs can only go into layout
    if (active === ComponentType.Tabs && parent !== ComponentType.Layout)
        return false;

    // Single tab can only go into tabs
    if (active === ComponentType.SingleTab && parent !== ComponentType.Tabs)
        return false;

    // Only tabs can go into layout 
    if (active !== ComponentType.Tabs && parent === ComponentType.Layout)
        return false;

    // Only single tab can go into tabs
    if (active !== ComponentType.SingleTab && parent === ComponentType.Tabs)
        return false;

    // Only button pad can go into video stream
    if (active !== ComponentType.ButtonPad && parent === ComponentType.VideoStream)
        return false;

    return true;
}

/**
 * Checks if two paths are adjacent to one another
 * 
 * @param activePath path to the active element
 * @param path path to this drop zone
 * @returns true if the paths are directly adjacent, false otherwise
 */
function pathsAdjacent(activePath: string, path: string) {
    // Check paths same length
    const splitActivePath = activePath.split('-');
    const splitSelfPath = path.split('-');
    const sameLength = splitActivePath.length == splitSelfPath.length;
    if (!sameLength) return false;

    // Should have same parent
    const activePrefix = splitActivePath.slice(0, -1);
    const selfPrefix = splitSelfPath.slice(0, -1);
    const matchingPrefix = activePrefix.every((val, index) => val === selfPrefix[index]);
    if (!matchingPrefix) return false;

    // Check if last indicies are adjacent
    const activeLast = +splitActivePath.slice(-1)[0];
    const selfLast = +splitSelfPath.slice(-1)[0];
    const adjacent = activeLast == selfLast || activeLast + 1 == selfLast;

    return adjacent;
}