import React, { useRef, useState } from "react";
import "operator/css/basic_components.css";
import { className } from "shared/util";

export const AccordionSelect = <T extends string | JSX.Element>(props: {
  title: string;
  possibleOptions: T[];
  onChange: (selectedIndex: number) => void;
}) => {
  const [active, setActiveState] = useState<boolean>(false);
  const [height, setHeightState] = useState("0px");
  const [rotate, setRotateState] = useState("accordion_icon");
  const content = useRef(null);

  function toggleAccordion() {
    setActiveState(active ? false : true);
    setHeightState(active ? "0px" : `${content.current.scrollHeight}px`);
    setRotateState(active ? "accordion_icon" : "accordion_icon rotate");
  }

  function mapFunc(option: T, idx: number) {
    return (
      <div
        className="accordion-item"
        key={idx}
        onClick={() => {
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
        onClick={toggleAccordion}
      >
        {props.title}
        <span className="material-icons">expand_more</span>
      </button>
      <div
        ref={content}
        style={{ maxHeight: `${height}` }}
        className="accordion_content"
      >
        <div>{props.possibleOptions.map(mapFunc)}</div>
      </div>
    </div>
  );
};
