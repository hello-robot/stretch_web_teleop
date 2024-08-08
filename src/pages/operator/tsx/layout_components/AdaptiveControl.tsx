import React from "react";
import {
  CustomizableComponentProps,
  SharedState,
  isSelected,
} from "./CustomizableComponent";
import { className } from "shared/util";
import "operator/css/AdaptiveControl.css";

export const AdaptiveControl = (props: CustomizableComponentProps) => {
  const [toggleAdaptive, setToggleAdaptive] = React.useState(false);
  const { customizing } = props.sharedState;
  const selected = isSelected(props);

  const toggleButton = () => {
    setToggleAdaptive((prevToggleSVG) => !prevToggleSVG);
  };

  function handleSelect(event: React.MouseEvent<HTMLDivElement>) {
    event.stopPropagation();
    props.sharedState.onSelect(props.definition, props.path);
  }

  return (
    <div
      className={className("adaptive-container", { customizing, selected })}
      onClick={handleSelect}
    >
      <div></div>
      <button
        className={`toggle-adaptive ${toggleAdaptive ? "on" : "off"}`}
        onClick={toggleButton}
        disabled={customizing}
      >
        {toggleAdaptive ? "ON" : "OFF"}
      </button>
      <img
        src="https://docs.hello-robot.com/0.3/getting_started/images/controller_labeled.png"
        height="50%"
      />
      <div></div>
    </div>
  );
};
