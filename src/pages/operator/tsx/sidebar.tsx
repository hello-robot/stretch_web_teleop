import "operator/css/sidebar.css"
import { className } from "utils/util";
import { ComponentDefinition, ComponentType, SingleTabDef } from "./componentdefinitions";

type SidebarProps = {
    hidden: boolean;
    onDelete: () => void;
    activeDef?: ComponentDefinition;
}

/** Popup on the right side of the screen while in customization mode. */
export const Sidebar = (props: SidebarProps) => {
    const deleteActive = props.activeDef != undefined;
    const deleteTooltip = deleteActive ? "Delete element" : "You must select an element before you can delete it"
    const selectedDescription = props.activeDef ? componentDescription(props.activeDef) : "none";
    return (
        <div id="sidebar" hidden={props.hidden}>
            <div id="sidebar-header">
                <b>Selected: {selectedDescription}</b>
            </div>
            <div id="sidebar-body">

            </div>
            <div id="sidebar-footer">
                <button id="delete-button"
                    title={deleteTooltip}
                    className={className("material-icons", { active: deleteActive })}
                    onClick={props.onDelete}
                >
                    delete_forever
                </button>
            </div>
        </div>
    )
}

/**
 * Creates a text description based on a component definition
 * @param definition component definition to describe
 * @returns string description of the component
 */
const componentDescription = (definition: ComponentDefinition): string => {
    switch (definition.type) {
        case(ComponentType.ButtonPad):
        case(ComponentType.VideoStream):
            return `${definition.id} ${definition.type}`
        case(ComponentType.Tabs):
            return "Tabs";
        case(ComponentType.SingleTab):
            return `Tab ${(definition as SingleTabDef).label}`;
        default:
            throw Error(`Cannot get description for component type ${definition.type}`)
    }
}
