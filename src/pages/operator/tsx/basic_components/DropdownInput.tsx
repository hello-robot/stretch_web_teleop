import React from "react";
import { className } from "shared/util";
import "operator/css/basic_components.css";
import e from "express";
import { text } from "stream/consumers";

export const DropdownInput = <T extends string>(props: {
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
  const dropdownPopupRef = React.useRef<HTMLDivElement>(null);

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

  // Handler to update the selected index if the possible options or text changes
  React.useEffect(() => {
    let text = props.text.trim();
    if (props.possibleOptions.includes(text as T)) {
      props.setSelectedIndex(props.possibleOptions.indexOf(text as T));
    } else {
      props.setSelectedIndex(undefined);
    }
  }, [props.possibleOptions, props.text]);

  // Function to convert each possible option into a button
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

  // Set the max-height of the popup to the screen height minus the top of the popup
  function resizeDropdownPopup() {
    if (dropdownPopupRef.current) {
      const top = dropdownPopupRef.current.getBoundingClientRect().top;
      dropdownPopupRef.current.style.maxHeight = `calc(100vh - ${top}px)`;
    }
  }
  React.useEffect(resizeDropdownPopup, [showDropdown]);
  React.useEffect(() => {
    window.addEventListener("resize", resizeDropdownPopup);
    return () => {
      window.removeEventListener("resize", resizeDropdownPopup);
    };
  });

  return (
    <div ref={componentRef} className="dropdown-input">
      <textarea
        className="dropdown-input-textarea"
        rows={props.rows}
        value={props.text}
        onClick={(e) => {
          e.stopPropagation();
          setShowDropdown(false);
        }}
        onFocus={(e) => {
          e.stopPropagation();
          e.target.select();
        }}
        onBlur={(e) => {
          document.getSelection()?.empty();
        }}
        onChange={(e) => {
          e.stopPropagation();
          props.setText(e.target.value);
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
        ref={dropdownPopupRef}
      >
        {props.possibleOptions.map(mapFunc)}
      </div>
    </div>
  );
};
