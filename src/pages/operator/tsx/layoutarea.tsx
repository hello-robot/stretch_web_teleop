import React from "react";
import "../css/layoutarea.css"
import { ComponentDef } from 'operator/tsx/componentdefinitions'
import { renderComponentList } from "./render";
import { DEFAULT_LAYOUT } from "./defaultlayout"

const DropZone = ({ }) => {
    return (
        <div className="drop-zone" >
        </div>
    )
}

interface LayoutAreaProps {
    customizing: boolean,
    layout?: ComponentDef[],
    buttonPadFunctions: any
}

export class LayoutArea extends React.Component {

    layout: ComponentDef[];

    constructor(props: LayoutAreaProps) {
        super(props);
        this.layout = props.layout || DEFAULT_LAYOUT;
    }

    render(): React.ReactNode {
        return (
            <div id="layout-area" >
                {renderComponentList(this.layout)}
            </div>
        )
    }
}


