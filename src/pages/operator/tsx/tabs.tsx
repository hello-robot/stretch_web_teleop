import React from "react";
import { SingleTabDef, TabsDef } from "./componentdefinitions"
import { renderComponentList } from "./render";
import "../css/tabs.css"

type TabsProps = {
    tabsDef: TabsDef
}

type TabsState = {
    /** Index of the active tab */
    active: number;
}

export class Tabs extends React.Component<TabsProps, TabsState> {
    constructor(props: TabsProps) {
        super(props);
        this.state = {
            active: 0
        }
    }

    render(): React.ReactNode {
        const contents = renderComponentList(this.props.tabsDef.tabs[this.state.active].contents)
        return (
            <div className="tabs-component" >
                <div className="tabs-header">
                    {this.props.tabsDef.tabs.map((tabDef: SingleTabDef, idx: number) => {
                        const isActive = this.state.active === idx;
                        return (
                            <button
                                key={`${tabDef.label}-${idx}`}
                                className={"tab-button" + (isActive ? " active" : "")}
                                onClick={() => this.setState({ active: idx })}
                            >
                                {tabDef.label}
                            </button>
                        );
                    })
                    }
                </div>
                <div className="tabs-content">
                    {contents}
                </div>
            </div>
        )
    }
}