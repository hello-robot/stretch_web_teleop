import React from "react";
import { useState } from "react";
import { className } from "../../../../shared/util";

export const SwitchButton = (props: {
    option1: string,
    option2: string,
    handleSwitchClick: (option: string) => void,
    activeOption: string
}) => {
    const [activeOption, setActiveOption] = useState<string>(props.activeOption);

    function onClick(option: string) {
        setActiveOption(option)
        console.log(option)
        props.handleSwitchClick(option)
    }

    return (
        <div className="switch-container">
            <div
                className={className("toggle-item", {active: activeOption === props.option1})}
                onClick={() => onClick(props.option1)}
            >
                <div className={"switch-text"}>{props.option1}</div>
            </div>
            <div
                className={className("toggle-item", {active: activeOption === props.option2})}
                onClick={() => onClick(props.option2)}
            >
                <div className={"switch-text"}>{props.option2}</div>
            </div>
        </div>
    );
}
