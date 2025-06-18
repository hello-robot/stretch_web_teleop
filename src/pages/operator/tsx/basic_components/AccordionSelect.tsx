import React, { useRef, useState } from "react";
import "operator/css/basic_components.css";
import { className } from "shared/util";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import ExpandLessIcon from "@mui/icons-material/ExpandLess";

export const AccordionSelect = <T extends string | JSX.Element>(props: {
    title: string;
    possibleOptions: T[];
    backgroundColor?: string;
    onChange: (selectedIndex: number) => void;
    toggleAccordianCallback?: () => void;
}) => {
    const [active, setActiveState] = useState<boolean>(false);
    const [height, setHeightState] = useState("0px");
    const [rotate, setRotateState] = useState("accordion_icon");
    const content = useRef(null);

    function toggleAccordion() {
        if (props.toggleAccordianCallback) {
            props.toggleAccordianCallback();
        }
        setActiveState(active ? false : true);
        setHeightState(active ? "0px" : `${content.current.scrollHeight}px`);
        setRotateState(active ? "accordion_icon" : "accordion_icon rotate");
    }

    function mapFunc(option: T, idx: number) {
        return (
            <div
                className="accordion-item"
                key={idx}
                onPointerDown={() => {
                    props.onChange(idx);
                    toggleAccordion();
                }}
            >
                {option}
            </div>
        );
    }

    return (
        <div className="accordion_section">
            <button
                className={className("accordion", { active })}
                onPointerDown={toggleAccordion}
                style={{ backgroundColor: props.backgroundColor }}
            >
                {props.title}
                {active ? <ExpandLessIcon /> : <ExpandMoreIcon />}
            </button>
            <div
                ref={content}
                style={{
                    maxHeight: `${height}`,
                    backgroundColor: props.backgroundColor,
                }}
                className="accordion_content"
            >
                <div>{props.possibleOptions.map(mapFunc)}</div>
            </div>
        </div>
    );
};
