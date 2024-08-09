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
  const [toggleAdaptive, setToggleAdaptive] = React.useState(true);
  const [controllerIndex, setControllerIndex] = React.useState(null);
  const buttonStateRef = React.useRef([false]);
  const { customizing } = props.sharedState;
  const selected = isSelected(props);

  const toggleButton = () => {
    setToggleAdaptive((prevToggleSVG) => !prevToggleSVG);
    if (!toggleAdaptive || customizing) {
      console.log("toggled off");
    }
  };

  function handleSelect(event: React.MouseEvent<HTMLDivElement>) {
    event.stopPropagation();
    props.sharedState.onSelect(props.definition, props.path);
  }

  /**
   ******** Get controller input ********
   */
  React.useEffect(() => {
    const handleControllerInput = () => {
      if (controllerIndex !== null) {
        const gamepad = navigator.getGamepads()[controllerIndex];
        if (gamepad) {
          const newButtonState = gamepad.buttons.map(
            (button) => button.pressed,
          );
          gamepad.buttons.forEach((button, index) => {
            const isPressed = newButtonState[index];
            const wasPressed = buttonStateRef.current[index];

            if (isPressed && !wasPressed) {
              handleButtonPress(index);
            } else if (!isPressed && wasPressed) {
              handleButtonRelease(index);
            }
          });
          buttonStateRef.current = newButtonState;
        }
      }
      requestAnimationFrame(handleControllerInput);
    };

    requestAnimationFrame(handleControllerInput);
  }, [controllerIndex]);

  /**
   ******** Handle button press and release ********
   */
  const handleButtonPress = (buttonIndex) => {
    let functs = adaptiveFunctionProvider.provideFunctions(buttonIndex);
    if (functs !== null) {
      functs.onClick();
    } else {
      console.log(`Button ${buttonIndex} is not mapped.`);
    }
  };

  const handleButtonRelease = (buttonIndex) => {
    let functs = adaptiveFunctionProvider.provideFunctions(buttonIndex);
    if (functs !== null) {
      functs.onRelease();
    }
  };

  /**
   ******** Look for gamepad connection ********
   */
  React.useEffect(() => {
    const connectHandler = (event) => {
      setControllerIndex(event.gamepad.index);
      buttonStateRef.current = event.gamepad.buttons.map(
        (button) => button.pressed,
      );
      console.log("Gamepad connected: ", event.gamepad);
    };

    const disconnectHandler = (event) => {
      if (event.gamepad.index === controllerIndex) {
        setControllerIndex(null);
        buttonStateRef.current = [];
      }
      console.log("Gamepad disconnected: ", event.gamepad);
    };

    window.addEventListener("gamepadconnected", connectHandler);
    window.addEventListener("gamepaddisconnected", disconnectHandler);

    return () => {
      window.removeEventListener("gamepadconnected", connectHandler);
      window.removeEventListener("gamepaddisconnected", disconnectHandler);
    };
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
