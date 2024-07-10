import React from "react";
import { className } from "shared/util";
import "operator/css/basic_components.css";
import e from "express";

export const DropdownInput = <T extends string | JSX.Element>(props: {
  text: string;
  setText: (text: string) => void;
  selectedIndex?: number;
  setSelectedIndex: (index?: number) => void;
  possibleOptions: T[];
  placeholderText: string;
  placement: string;
  rows: number;
}) => {
  const [showDropdown, setShowDropdown] = React.useState(false);
  const componentRef = React.useRef<HTMLDivElement>(null);

  // Handler to close dropdown when click outside
  React.useEffect(() => {
    const handler = (e: any) => {
      if (componentRef.current && !componentRef.current.contains(e.target)) {
        setShowDropdown(false);
      }
    };
    if (showDropdown) {
      window.addEventListener("click", handler);
      return () => {
        window.removeEventListener("click", handler);
      };
    }
  });

  function mapFunc(option: T, idx: number) {
    const active = idx === props.selectedIndex;
    return (
      <button
        key={idx}
        onClick={(e) => {
          e.stopPropagation();
          setShowDropdown(false);
          props.setText(option as string);
          if (!active) props.setSelectedIndex(idx);
        }}
        className={className("dropdown-input-option", { active })}
      >
        {option}
      </button>
    );
  }

  return (
    <div ref={componentRef} className="dropdown-input">
      <textarea
        className="dropdown-input-textarea"
        rows={props.rows}
        value={props.text}
        onClick={(e) => e.stopPropagation()}
        onChange={(e) => {
          e.stopPropagation();
          props.setText(e.target.value);
          props.setSelectedIndex(undefined);
        }}
        placeholder={props.placeholderText}
      />
      <button
        className={className("dropdown-input-button", {
          expanded: showDropdown,
          top: props.placement == "top",
          bottom: props.placement == "bottom",
        })}
        onClick={(e) => {
          console.log("clicked", e);
          e.stopPropagation();
          setShowDropdown(!showDropdown);
        }}
      >
        <span className="material-icons">expand_more</span>
      </button>
      <div
        hidden={!showDropdown}
        className={className("dropdown-input-popup", {
          top: props.placement == "top",
          bottom: props.placement == "bottom",
        })}
      >
        {props.possibleOptions.map(mapFunc)}
      </div>
    </div>
  );
};
