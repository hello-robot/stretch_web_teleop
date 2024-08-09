import React from "react";
import {
  CustomizableComponentProps,
  SharedState,
  isSelected,
} from "./CustomizableComponent";
import { className } from "shared/util";
import "operator/css/AdaptiveControl.css";
import { adaptiveFunctionProvider } from "operator/tsx/index";

export const AdaptiveControl = (props: CustomizableComponentProps) => {
  const [toggleAdaptive, setToggleAdaptive] = React.useState(false);
  const { customizing } = props.sharedState;
  const selected = isSelected(props);

  let controllerIndex = null;
  let aPressed = false;
  let upPad = false;
  let downPad = false;
  let leftPad = false;
  let rightPad = false;

  const toggleButton = () => {
    setToggleAdaptive((prevToggleSVG) => !prevToggleSVG);
    if (!toggleAdaptive || customizing) {
      console.log("toggled on");
    }
  };

  function handleSelect(event: React.MouseEvent<HTMLDivElement>) {
    event.stopPropagation();
    props.sharedState.onSelect(props.definition, props.path);
  }

  function controllerInput() {
    if (controllerIndex !== null) {
      const pollingInterval = setInterval(() => {
        const gamepad = navigator.getGamepads()[controllerIndex];
        const buttons = gamepad.buttons;
        aPressed = buttons[0].pressed;
        upPad = buttons[12].pressed;
        downPad = buttons[13].pressed;
        leftPad = buttons[14].pressed;
        rightPad = buttons[15].pressed;

        if (aPressed) {
          console.log("A button pressed");
        }

        // let functs = adaptiveFunctionProvider.provideFunctions();
        // functs.onClick();
      }, 100);
    }
  }

  React.useEffect(() => {
    window.addEventListener("gamepadconnected", (event) => {
      controllerIndex = event.gamepad.index;
      console.log("connected");
      controllerInput();
    });
    window.addEventListener("gamepaddisconnected", (event) => {
      console.log("disconnected");
      controllerIndex = null;
    });
  }, [controllerIndex]);

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
