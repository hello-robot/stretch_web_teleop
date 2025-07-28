import React from "react";
import {
    ComponentDefinition,
    ComponentType,
    TabDefinition,
    PanelDefinition,
    ParentComponentDefinition,
    LayoutDefinition,
} from "../utils/component_definitions";
import { SharedState } from "./CustomizableComponent";
import { className } from "shared/util";
import { PopupModal } from "../basic_components/PopupModal";
import "operator/css/DropZone.css";
import PushPinIcon from "@mui/icons-material/PushPin";

/** State required for drop zones */
export type DropZoneState = {
    /**
     * Callback when the DropZone is clicked to handle dropping the active
     * selected component into the position of the drop zone
     *
     * @param path path to the drop zone
     */
    onDrop: (path: string) => void;
    /** Definition of the active selected component */
    selectedDefinition?: ComponentDefinition;
};

/** Properties for {@link DropZone} */
export type DropZoneProps = {
    /** Path to the drop zone, same as CustomizableComponent path */
    path: string;
    /** Definition of the CustomizableComponent containing this drop zone */
    parentDef: ParentComponentDefinition;
    /** State shared between all components */
    sharedState: SharedState;
};

/**
 * Area where the user can click to move the selected component to a new place.
 *
 * @param props {@link DropZoneProps}
 */
export const DropZone = (props: DropZoneProps) => {
    const [showNewPanelModal, setShowNewPanelModal] =
        React.useState<boolean>(false);

    function canDrop(): boolean {
        const { path, parentDef } = props;
        const { selectedPath } = props.sharedState;
        const { selectedDefinition } = props.sharedState.dropZoneState;

        // If no active object selected
        if (!selectedDefinition) {
            return false;
        }

        // Must pass drop zone rules about which components can go where
        if (!dropzoneRules(selectedDefinition.type, parentDef.type))
            return false;

        // Don't need to check if dropzone is adjacent if the active component
        // is coming from the sidebar component provider
        if (!selectedPath) return true;

        // If Layout Grid only has one child panel and the panel is selected, hide dropzones to the
        // left and right of the panel as these dropzones would not move the active panel
        if (
            parentDef.type == ComponentType.Layout &&
            selectedDefinition.type == ComponentType.Panel
        ) {
            let selectedLayoutGridIdx = Number(selectedPath?.split("-")[0]);
            let pathLayoutGridIdx = Number(path.split("-")[0]);

            if (
                pathLayoutGridIdx === selectedLayoutGridIdx ||
                pathLayoutGridIdx === selectedLayoutGridIdx + 1
            ) {
                let parentIdx = Number(selectedPath?.split("-")[0]);
                let def = parentDef as LayoutDefinition;
                if (def.children[parentIdx]?.children.length < 2) return false;
            }
        }

        // Can't drop if dropzone is right next to the active element
        // (that wouldn't move the active element at all)
        return !pathsAdjacent(selectedPath, path);
    }

    /** Calls onDrop function from Operator with the path of this dropzone */
    function handleClick(e: React.MouseEvent<HTMLSpanElement>) {
        if (!props.sharedState.customizing) return;
        e.stopPropagation();
        // If adding a new tabs component from the sidebar
        if (
            props.sharedState.dropZoneState.selectedDefinition?.type ===
                ComponentType.Panel &&
            props.sharedState.selectedPath === undefined
        ) {
            setShowNewPanelModal(true);
            return;
        }
        props.sharedState.dropZoneState.onDrop(props.path);
    }

    /**
     * When adding a panel from the sidebar (so it's not already in the interface),
     * this adds a new tab child and drops the new panel into the drop zone.
     *
     * @param newTabName the name of the tab child within the new panel
     */
    function createNewPanel(newTabName: string) {
        if (
            props.sharedState.dropZoneState.selectedDefinition?.type !==
            ComponentType.Panel
        )
            throw Error(
                `Should only call createNewPanel() when the active selected component is of type Panel`,
            );

        const def = props.sharedState.dropZoneState
            .selectedDefinition as PanelDefinition;

        if (def.children.length > 1)
            throw Error(
                `createNewPanel() called with active panel definition that already has children: ${def.children}`,
            );

        if (props.sharedState.selectedPath !== undefined)
            throw Error(
                `Called createNewPanel() when active selected path was not undefined ${props.sharedState.selectedPath}`,
            );

        // Create a child tab and add it to the Panel's children
        def.children.push({
            type: ComponentType.SingleTab,
            label: newTabName,
            children: [],
        } as TabDefinition);

        // Drop the new panel into the drop zone
        props.sharedState.dropZoneState.onDrop(props.path);
    }

    const isActive = props.sharedState.customizing && canDrop();
    const inTab = props.parentDef.type === ComponentType.Panel;
    const overlay = props.parentDef.type === ComponentType.CameraView;
    const standard = !(inTab || overlay);

    return (
        <React.Fragment>
            <span
                className={className("drop-zone", {
                    tab: inTab,
                    overlay,
                    standard,
                })}
                hidden={!isActive}
                onClick={handleClick}
            >
                <PushPinIcon />
            </span>
            <NewPanelModal
                show={showNewPanelModal}
                setShow={setShowNewPanelModal}
                addPanel={createNewPanel}
            />
        </React.Fragment>
    );
};

/** Popup to name the first tab when a new panel component is added to the interface. */
const NewPanelModal = (props: {
    /** If the modal should be shown. */
    show: boolean;
    /** Callback to change the state of `show` */
    setShow: (show: boolean) => void;
    /**
     * Callback to add the panel (drop into the dropzone).
     * @param tabName the name of the tab child in the new panel
     */
    addPanel: (tabName: string) => void;
}) => {
    const [text, setText] = React.useState<string>("");
    /** Call `addPanel` with the text in the text entry. */
    function handleAccept() {
        if (text.length > 0) props.addPanel(text);
    }
    /** Update the text entry when the user types in it. */
    function handleChange(e: React.ChangeEvent<HTMLInputElement>) {
        setText(e.target.value);
    }
    return (
        <PopupModal
            setShow={props.setShow}
            show={props.show}
            onAccept={handleAccept}
            id="new-panel-modal"
            acceptButtonText="Create Panel"
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
                onChange={handleChange}
                placeholder="label for the new tab"
            />
        </PopupModal>
    );
};

/**
 * Checks if the active component is allowed to be placed in this drop zone.
 *
 * @param active type of the active component the user selected
 * @param parent type of the parent component containing this drop zone
 * @returns true if the active component is allowed to be dropped, false otherwise
 */
function dropzoneRules(active: ComponentType, parent: ComponentType) {
    // Tabs can only go into layout
    if (
        active === ComponentType.Panel &&
        parent !== ComponentType.LayoutGrid &&
        parent !== ComponentType.Layout
    )
        return false;

    // Single tab can only go into tabs
    if (active === ComponentType.SingleTab && parent !== ComponentType.Panel)
        return false;

    // Only tabs can go into panel
    if (
        active !== ComponentType.Panel &&
        (parent === ComponentType.LayoutGrid || parent == ComponentType.Layout)
    )
        return false;

    // Only single tab can go into tabs
    if (active !== ComponentType.SingleTab && parent === ComponentType.Panel)
        return false;

    // Only button pad can go into video stream
    if (
        active !== ComponentType.ButtonPad &&
        parent === ComponentType.CameraView
    )
        return false;

    return true;
}

/**
 * Checks if two paths are adjacent to one another.
 *
 * @note this prevents displaying drop zones directly adjacent to the selected
 * component, which would have no effect on the components position.
 *
 * @param selectedPath path to the active element
 * @param path path to this drop zone
 * @returns true if the paths are directly adjacent, false otherwise
 */
function pathsAdjacent(selectedPath: string, path: string) {
    // Check paths same length
    const splitActivePath = selectedPath.split("-");
    const splitSelfPath = path.split("-");
    const sameLength = splitActivePath.length == splitSelfPath.length;
    if (!sameLength) return false;

    // Should have same parent
    const activePrefix = splitActivePath.slice(0, -1);
    const selfPrefix = splitSelfPath.slice(0, -1);
    const matchingPrefix = activePrefix.every(
        (val, index) => val === selfPrefix[index],
    );
    if (!matchingPrefix) return false;

    // Check if last indices are adjacent
    const activeLast = +splitActivePath.slice(-1)[0];
    const selfLast = +splitSelfPath.slice(-1)[0];
    const adjacent = activeLast == selfLast || activeLast + 1 == selfLast;

    return adjacent;
}
