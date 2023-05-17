import React from "react"
import "operator/css/sidebar.css"
import { className } from "shared/util";
import { ButtonPadId, ComponentDefinition, ComponentType, SingleTabDef, VideoStreamDef, VideoStreamId } from "../utils/componentdefinitions";

type SidebarProps = {
    hidden: boolean;
    onDelete: () => void;
    updateLayout: () => void;
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
                {props.activeDef ?
                    <SidebarOptions
                        activeDef={props.activeDef}
                        updateLayout={props.updateLayout}
                    /> : undefined}
            </div>
            <div id="sidebar-footer">
                <button id="delete-button"
                    title={deleteTooltip}
                    className={className("material-icons", { active: deleteActive })}
                    onClick={deleteActive ? () => props.onDelete() : undefined}
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
function componentDescription(definition: ComponentDefinition): string {
    switch (definition.type) {
        case (ComponentType.ButtonPad):
        case (ComponentType.VideoStream):
            return `${definition.id} ${definition.type}`
        case (ComponentType.Tabs):
            return "Tabs";
        case (ComponentType.SingleTab):
            return `${(definition as SingleTabDef).label} Tab`;
        default:
            throw Error(`Cannot get description for component type ${definition.type}`)
    }
}

type OptionsProps = {
    activeDef: ComponentDefinition;
    updateLayout: () => void;
}

const SidebarOptions = (props: OptionsProps) => {
    switch (props.activeDef.type) {
        case (ComponentType.VideoStream):
            switch (props.activeDef.id!) {
                case (VideoStreamId.overhead):
                    return <OverheadVideoStreamOptions {...props} />;
            }
        default:
            return <></>;
    }
}

const OverheadVideoStreamOptions = (props: OptionsProps) => {
    const definition = props.activeDef as VideoStreamDef;
    const pd = definition.children.length > 0 && definition.children[0].id == ButtonPadId.PredictiveDisplay;
    const [predictiveDisplayOn, setPredictiveDisplayOn] = React.useState(pd);
    function togglePredictiveDisplay() {
        const newPdOn = !predictiveDisplayOn;
        setPredictiveDisplayOn(newPdOn);
        if (newPdOn) {
            // Add predictive display to the stream
            definition.children = [
                {type: ComponentType.ButtonPad, id: ButtonPadId.PredictiveDisplay}
            ];
        } else {
            definition.children = [];
        }
        props.updateLayout();
    }
    return (
        <div>
            <ToggleButton
                on={predictiveDisplayOn}
                onClick={togglePredictiveDisplay}
                label="Predictive Display"
            />
        </div>
    )
}

type ToggleButtonProps = {
    on: boolean;
    onClick: () => void;
    label: string;
}

const ToggleButton = (props: ToggleButtonProps) => {
    const text = props.on ? "on" : "off";
    return (
        <div className="options-element">
            <button
                className={className("toggle-button", { on: props.on })}
                onClick={props.onClick}
            >
                {text}
            </button>
            <span>{props.label}</span>
        </div>
    );
}
