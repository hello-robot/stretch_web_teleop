import React, { JSXElementConstructor, ReactElement } from "react";
import { className } from "shared/util";
import "operator/css/TabGroup.css";

export const Tab = (props: {
  label: string;
  active: boolean;
  onClick: () => void;
  pill: boolean;
}) => {
  const active = props.active;

  return props.pill ? (
    <li
      className={className("pill-tab-btn", { active })}
      onClick={props.onClick}
      key={props.label}
    >
      {props.label}
    </li>
  ) : (
    <button
      className={className("tab-btn", { active })}
      onClick={props.onClick}
      key={props.label}
    >
      {props.label}
    </button>
  );
};

export const TabGroup = (props: {
  tabLabels: string[];
  tabContent: ((active: boolean) => React.JSX.Element)[];
  startIdx: number;
  onChange: (index: number) => void;
  pill: boolean;
}) => {
  const tabLabels = props.tabLabels;
  const tabContent = props.tabContent;
  const [activeIndex, setActiveIndex] = React.useState<number>(props.startIdx);

  return (
    <div className="tab-group" onContextMenu={(e) => e.preventDefault()}>
      <div className={props.pill ? "pill-tab" : "tab"}>
        {tabLabels.map((label, index) => (
          <Tab
            key={label}
            label={label}
            active={activeIndex === index}
            onClick={() => {
              props.onChange(index);
              setActiveIndex(index);
            }}
            pill={props.pill}
          />
        ))}
      </div>
      <div className="tab-content">
        {tabContent.map((renderFn, index) => renderFn(activeIndex === index))}
      </div>
    </div>
  );
};
