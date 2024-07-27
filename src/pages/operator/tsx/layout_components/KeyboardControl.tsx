import React from "react";
import {
  CustomizableComponentProps,
  SharedState,
  isSelected,
} from "./CustomizableComponent";
import { keyboardFunctionProvider } from "operator/tsx/index";
import { Mode } from "../function_providers/KeyboardFunctionProvider";
import "operator/css/ButtonGrid.css";
import "operator/css/KeyboardControl.css";
import { className } from "shared/util";
import {
  ButtonPadShape,
  getIcon,
  getPathsFromShape,
  SVG_RESOLUTION,
} from "../utils/svg";

export const KeyboardControl = (props: CustomizableComponentProps) => {
  const [mode, setMode] = React.useState<Mode>(Mode.Base);
  const [keyPressed, setKeyPressed] = React.useState(false);
  const [activeMode, setActiveMode] = React.useState(1);
  const [toggleState, setToggleState] = React.useState(true);

  const { customizing } = props.sharedState;
  const selected = isSelected(props);

  function handleSelect(event: React.MouseEvent<HTMLDivElement>) {
    event.stopPropagation();
    props.sharedState.onSelect(props.definition, props.path);
  }

  const selectProp = customizing
    ? {
        onClick: handleSelect,
      }
    : {};

  const [keyState, setKeyState] = React.useState({
    w: false,
    a: false,
    s: false,
    d: false,
    q: false,
    e: false,
    j: false,
    k: false,
    ArrowUp: false,
    ArrowLeft: false,
    ArrowDown: false,
    ArrowRight: false,
  });

  const toggleButton = () => {
    setToggleState((prevToggleState) => !prevToggleState);
    console.log("keys on?", toggleState);
  };

  const handleKeyPress = React.useCallback(
    (event) => {
      if (keyPressed === true) {
        return;
      }
      setKeyPressed(true);

      switch (event.key) {
        case "1":
          setMode(Mode.Base);
          setActiveMode(1);
          console.log("base mode enabled");
          break;
        case "2":
          setMode(Mode.Wrist);
          setActiveMode(2);
          console.log("wrist mode enabled");
          break;
        case "3":
          setMode(Mode.Arm);
          setActiveMode(3);
          console.log("arm mode enabled");
          break;
      }

      let functs = keyboardFunctionProvider.provideFunctions(mode, event.key);
      functs.onClick();
      setKeyState((prevState) => ({ ...prevState, [event.key]: true }));
    },
    [mode, keyPressed],
  );

  const handleKeyRelease = React.useCallback(
    (event) => {
      setKeyPressed(false);
      setKeyState((prevState) => ({ ...prevState, [event.key]: false }));

      let functs = keyboardFunctionProvider.provideFunctions(mode, event.key);
      functs.onRelease();
    },
    [mode, keyPressed],
  );

  React.useEffect(() => {
    if (toggleState && !customizing) {
      window.addEventListener("keydown", handleKeyPress);
      window.addEventListener("keyup", handleKeyRelease);
    } else {
      setKeyPressed(false);
      window.removeEventListener("keydown", handleKeyPress);
      window.removeEventListener("keyup", handleKeyRelease);
    }
    return () => {
      window.removeEventListener("keydown", handleKeyPress);
      window.removeEventListener("keyup", handleKeyRelease);
    };
  }, [toggleState, handleKeyPress, handleKeyRelease, customizing]);

  return (
    <div
      className={className("keyboard-control", { customizing, selected })}
      onClick={handleSelect}
    >
      <div className="keyboard-row">
        <div className="keyboard-column">
          <button
            className={`${toggleState ? "toggle-key active" : "toggle-key off"}`}
            onClick={toggleButton}
          >
            {" "}
            {toggleState ? "KEYS ON" : "KEYS OFF"}{" "}
          </button>
          <div className="mode-button">
            <button
              className={
                activeMode === 1 ? "mode-button active" : "mode-button"
              }
              disabled
            >
              {" "}
              1{" "}
            </button>
            <button
              className={
                activeMode === 2 ? "mode-button active" : "mode-button"
              }
              disabled
            >
              {" "}
              2{" "}
            </button>
            <button
              className={
                activeMode === 3 ? "mode-button active" : "mode-button"
              }
              disabled
            >
              {" "}
              3{" "}
            </button>
          </div>
        </div>
      </div>
      <div className="keyboard-row">
        <div className="controls-container">
          {mode} Mode Controls
          <br />
          <div className="controls-column">
            {activeMode === 2 && (
              <button
                className={
                  keyState.q ? "keyboard-button active" : "keyboard-button"
                }
                disabled
              >
                {" "}
                Q{" "}
              </button>
            )}
            <button
              className={
                keyState.w ? "keyboard-button active" : "keyboard-button"
              }
              disabled
            >
              {" "}
              W{" "}
            </button>
            {activeMode === 2 && (
              <button
                className={
                  keyState.e ? "keyboard-button active" : "keyboard-button"
                }
                disabled
              >
                {" "}
                E{" "}
              </button>
            )}
          </div>
          <div className="controls-column">
            <button
              className={
                keyState.a ? "keyboard-button active" : "keyboard-button"
              }
              disabled
            >
              {" "}
              A{" "}
            </button>
            <button
              className={
                keyState.s ? "keyboard-button active" : "keyboard-button"
              }
              disabled
            >
              {" "}
              S{" "}
            </button>
            <button
              className={
                keyState.d ? "keyboard-button active" : "keyboard-button"
              }
              disabled
            >
              {" "}
              D{" "}
            </button>
          </div>
        </div>
      </div>
      <div className="keyboard-row">
        <div className="controls-container">
          Camera Controls <br />
          <div className="controls-column">
            <button
              className={
                keyState.ArrowUp ? "keyboard-button active" : "keyboard-button"
              }
              disabled
            >
              {" "}
              ^{" "}
            </button>
          </div>
          <div className="controls-column">
            <button
              className={
                keyState.ArrowLeft
                  ? "keyboard-button active"
                  : "keyboard-button"
              }
              disabled
            >
              {" "}
              &lt;{" "}
            </button>
            <button
              className={
                keyState.ArrowDown
                  ? "keyboard-button active"
                  : "keyboard-button"
              }
              disabled
            >
              {" "}
              v{" "}
            </button>
            <button
              className={
                keyState.ArrowRight
                  ? "keyboard-button active"
                  : "keyboard-button"
              }
              disabled
            >
              {" "}
              &gt;{" "}
            </button>
          </div>
        </div>
        <div className="gripper-container">
          Gripper Controls <br />
          <div className="gripper-column">
            <button
              className={
                keyState.j ? "keyboard-button active" : "keyboard-button"
              }
              disabled
            >
              J
            </button>
            <button
              className={
                keyState.k ? "keyboard-button active" : "keyboard-button"
              }
              disabled
            >
              {" "}
              K{" "}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};
