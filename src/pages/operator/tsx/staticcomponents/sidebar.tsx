import React from "react"
import "operator/css/sidebar.css"
import { className } from "shared/util";
import { ButtonPadDef, ButtonPadId, ComponentDefinition, ComponentId, ComponentType, LayoutDefinition, SingleTabDef, TabsDef, VideoStreamDef, VideoStreamId } from "../utils/componentdefinitions";
import { PopupModal } from "../basic_components/popup_modal";
import { Dropdown } from "../basic_components/dropdown";

type SidebarProps = {
    hidden: boolean;
    onDelete: () => void;
    updateLayout: () => void;
    onSelect: (def: ComponentDefinition, path?: string) => void;
    activeDef?: ComponentDefinition;
    activePath?: string;
    globalOptionsProps: GlobalOptionsProps
}

/** Popup on the right side of the screen while in customization mode. */
export const Sidebar = (props: SidebarProps) => {
    const deleteDisabled = props.activePath === undefined;
    const deleteTooltip = deleteDisabled ? "You must select an element before you can delete it" : "";
    const selectedDescription = props.activeDef ? componentDescription(props.activeDef) : "none";
    return (
        <div id="sidebar" hidden={props.hidden}>
            <div id="sidebar-header">
                <b>Selected: {selectedDescription}</b>
            </div>
            <div id="sidebar-body">
                {props.activePath ?
                    <SidebarOptions
                        activeDef={props.activeDef!}
                        updateLayout={props.updateLayout}
                    /> :
                    <React.Fragment>
                        <SidebarComponentProvider
                            activeDef={props.activeDef}
                            onSelect={props.onSelect}
                        />
                        <SidebarGlobalOptions
                            {...props.globalOptionsProps}
                        />

                    </React.Fragment>
                }
            </div>
            <div id="sidebar-footer">
                <button id="delete-button"
                    disabled={deleteDisabled}
                    title={deleteTooltip}
                    className={className("material-icons btn-red", {})}
                    onClick={deleteDisabled ? undefined : () => props.onDelete()}
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
            return `${(definition as VideoStreamDef | ButtonPadDef).id} ${definition.type}`
        case (ComponentType.Tabs):
            return "Tabs";
        case (ComponentType.SingleTab):
            return `\"${(definition as SingleTabDef).label}\" Tab`;
        default:
            throw Error(`Cannot get description for component type ${definition.type}`)
    }
}

/*******************************************************************************
 * Global options
 */

export type GlobalOptionsProps = {
    displayVoiceControl: boolean;
    setDisplayVoiceControl: (displayVoiceControl: boolean) => void;
    defaultLayouts: string[],
    customLayouts: string[],
    loadLayout: (layoutName: string, dflt: boolean) => void,
    saveLayout: (layoutName: string) => void,
}

const SidebarGlobalOptions = (props: GlobalOptionsProps) => {
    const [showLoadLayoutModal, setShowLoadLayoutModal] = React.useState<boolean>(false);
    const [showSaveLayoutModal, setShowSaveLayoutModal] = React.useState<boolean>(false);


    return (
        <React.Fragment>
            <div id="global-settings">
                <p>Global settings:</p>
                <ToggleButton
                    on={props.displayVoiceControl}
                    onClick={() => props.setDisplayVoiceControl(!props.displayVoiceControl)}
                    label="Display voice control"
                />
                <button
                    onClick={() => setShowLoadLayoutModal(true)}
                >
                    Load layout
                </button>
                <button
                    onClick={() => setShowSaveLayoutModal(true)}
                >
                    Save layout
                </button>
            </div>
            <LoadLayoutModal
                defaultLayouts={props.defaultLayouts}
                customLayouts={props.customLayouts}
                loadLayout={props.loadLayout}
                setShow={setShowLoadLayoutModal}
                show={showLoadLayoutModal}
            />
            <SaveLayoutModal
                saveLayout={props.saveLayout}
                setShow={setShowSaveLayoutModal}
                show={showSaveLayoutModal}
            />
        </React.Fragment>
    )
}

const LoadLayoutModal = (props: {
    defaultLayouts: string[],
    customLayouts: string[],
    loadLayout: (layoutName: string, dflt: boolean) => void,
    setShow: (show: boolean) => void,
    show: boolean,
}) => {
    const [selectedIdx, setSelectedIdx] = React.useState<number>();

    function handleAccept() {
        if (selectedIdx === undefined) return;
        let dflt: boolean, layoutName: string;
        if (selectedIdx < props.defaultLayouts.length) {
            layoutName = props.defaultLayouts[selectedIdx];
            dflt = true;
        } else {
            layoutName = props.customLayouts[selectedIdx - props.defaultLayouts.length];
            dflt = false;
        }
        console.log('loading layout', layoutName, dflt);
        props.loadLayout(layoutName, dflt);

    }

    function mapFunct(layoutName: string, dflt: boolean) {
        const prefix = dflt ? "DEFAULT" : "CUSTOM";
        return <p><em>{prefix}</em> {layoutName}</p>
    }

    const defaultOptions = props.defaultLayouts.map(layoutName => mapFunct(layoutName, true));
    const customOptions = props.customLayouts.map(layoutName => mapFunct(layoutName, false));

    return (
        <PopupModal
            setShow={props.setShow}
            show={props.show}
            onAccept={handleAccept}
            id="load-layout-modal"
            acceptButtonText="Load Layout"
        >
            <p><b>Select layout to load</b></p>
            <Dropdown
                onChange={setSelectedIdx}
                selectedIndex={selectedIdx}
                possibleOptions={defaultOptions.concat(customOptions)}
                placeholderText="Select a layout..."
            />
        </PopupModal>
    )
}

const SaveLayoutModal = (props: {
    saveLayout: (layoutName: string) => void,
    setShow: (show: boolean) => void,
    show: boolean,
}) => {
    const [name, setName] = React.useState<string>("");
    function handleAccept() {
        if (name.length > 0) props.saveLayout(name);
        setName("");
    }
    return (
        <PopupModal
            setShow={props.setShow}
            show={props.show}
            onAccept={handleAccept}
            id="save-layout-modal"
            acceptButtonText="Save"
        >
            <label htmlFor="new-layout-name"><b>New Tab Label</b></label>
            <input type="text" id="new-layout-name" name="new-tab-name"
                value={name} onChange={(e) => setName(e.target.value)}
                placeholder="name for this layout"
            />
        </PopupModal>
    )
}

/*******************************************************************************
 * Component specific options
 */

type OptionsProps = {
    activeDef: ComponentDefinition;
    updateLayout: () => void;
}

const SidebarOptions = (props: OptionsProps) => {
    switch (props.activeDef.type) {
        case (ComponentType.VideoStream):
            switch ((props.activeDef as VideoStreamDef).id!) {
                case (VideoStreamId.overhead):
                    return <OverheadVideoStreamOptions {...props} />;
            }
            break;
    }
    return <></>
}

const OverheadVideoStreamOptions = (props: OptionsProps) => {
    const definition = props.activeDef as VideoStreamDef;
    const pd = definition.children.length > 0 && definition.children[0].type == ComponentType.PredictiveDisplay;
    const [predictiveDisplayOn, setPredictiveDisplayOn] = React.useState(pd);
    function togglePredictiveDisplay() {
        const newPdOn = !predictiveDisplayOn;
        setPredictiveDisplayOn(newPdOn);
        if (newPdOn) {
            // Add predictive display to the stream
            definition.children = [{ type: ComponentType.PredictiveDisplay }];
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
    const colorClass = props.on ? "btn-green" : "btn-red";
    return (
        <div className="toggle-button-div">
            <button
                className={"toggle-button " + colorClass}
                onClick={props.onClick}
            >
                {text}
            </button>
            <span>{props.label}</span>
        </div>
    );
}

/*******************************************************************************
 * Component provider
 */

type SidebarComponentProviderProps = {
    activeDef?: ComponentDefinition;
    onSelect: (def: ComponentDefinition, path?: string) => void;
}

const SidebarComponentProvider = (props: SidebarComponentProviderProps) => {
    const [expandedType, setExpandedType] = React.useState<ComponentType>();

    const outlines: ComponentProviderTabOutline[] = [
        { type: ComponentType.Tabs },
        { type: ComponentType.VideoStream, ids: Object.values(VideoStreamId) },
        { type: ComponentType.ButtonPad, ids: Object.values(ButtonPadId) }
    ];

    function handleSelect(type: ComponentType, id?: ComponentId) {
        const definition: ComponentDefinition = { type, id };

        // Add children based on the component type
        switch (type) {
            case (ComponentType.Tabs):
                (definition as TabsDef).children = [
                    {
                        type: ComponentType.SingleTab,
                        label: "new tab",
                        children: []
                    } as SingleTabDef
                ]
                break;
            case (ComponentType.VideoStream):
                (definition as VideoStreamDef).children = []
                break;
        }

        props.onSelect(definition);
    }

    function mapTabs(outline: ComponentProviderTabOutline) {
        const expanded = outline.type === expandedType;
        const tabProps: ComponentProviderTabProps = {
            ...outline,
            expanded,
            activeDef: props.activeDef,
            onExpand: () => setExpandedType(expanded ? undefined : outline.type),
            onSelect: (id?: ComponentId) => handleSelect(outline.type, id)
        }
        return <ComponentProviderTab {...tabProps} key={outline.type} />
    }

    return (
        <div id="sidebar-component-provider">
            <p>Select a component to add:</p>
            <div id="components-set">
                {
                    outlines.map(mapTabs)
                }
            </div>
        </div>
    )
}

type ComponentProviderTabOutline = { type: ComponentType, ids?: ComponentId[] }

type ComponentProviderTabProps = ComponentProviderTabOutline & {
    expanded: boolean;
    activeDef?: ComponentDefinition;
    onSelect: (id?: ComponentId) => void;
    onExpand: () => void;
}

const ComponentProviderTab = (props: ComponentProviderTabProps) => {
    const tabActive = props.type === props.activeDef?.type;
    function mapIds(id: ComponentId) {
        const active = tabActive && id === props.activeDef?.id;
        return (
            <button
                key={id}
                onClick={() => props.onSelect(id)}
                className={className("id-button", { active })}
            >
                {id}
            </ button>
        );
    }

    function clickExpand() {
        console.log('click', props.type)
        props.ids ? props.onExpand() : props.onSelect();
    }

    return (
        <div className="provider-tab" key={props.type}>
            <button onClick={clickExpand} className={tabActive && !props.ids ? "active" : props.expanded ? "expanded" : ""}>
                <span className="material-icons">{props.ids ? "expand_more" : ""}</span>
                {props.type}
            </button>
            <div className="provider-tab-dropdown" hidden={!props.expanded}>
                {
                    props.ids ?
                        props.ids.map(mapIds) : undefined
                }
            </div>
        </div>
    )
}