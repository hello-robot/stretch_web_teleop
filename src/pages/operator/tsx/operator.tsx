import React from "react";
import { VelocityControl, DEFAULT_SPEED } from "operator/tsx/velocitycontrol"
import { LayoutArea } from "./layoutarea";
import { ActionMode, ActionModeButton } from "operator/tsx/actionmodebutton"
import "operator/css/operator.css"

/** Internal state of Operator */
interface OperatorState {
    /** Current speed of the robot */
    // speed: number;
    customizing: boolean;
}

type CustomizeButtonProps = {
    customizing: boolean;
    onClick: () => void;
}

// Uses icons from https://fonts.google.com/icons
const CustomizeButton = (props: CustomizeButtonProps) => {
    const icon = props.customizing ? "check_circle" : "build_circle";
    return (
        <span
            className="material-icons"
            style={{ fontSize: "36px", cursor: "pointer" }}
            onClick={props.onClick}
        >
            {icon}
        </span>
    )
}

/** Operator internface webpage */
export class Operator extends React.Component<any, OperatorState> {
    speed: number;
    actionMode: ActionMode;

    constructor(props: {}) {
        super(props);
        this.speed = DEFAULT_SPEED;
        this.actionMode = ActionMode.StepActions;
        this.state = {
            customizing: false
        }

    }

    render() {
        return (
            <div id="operator-body">
                <div id="operator-header">
                    <ActionModeButton
                        default={this.actionMode}
                        onChange={(newAm) => this.actionMode = newAm}
                    />
                    <VelocityControl
                        initialSpeed={this.speed}
                        onChange={(newSpeed) => this.speed = newSpeed}
                    />
                    <CustomizeButton
                        customizing={this.state.customizing}
                        onClick={() => this.setState({ customizing: !this.state.customizing })}
                    />
                </div>
                < LayoutArea />
            </div>
        )
    }
}