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
import { buttonFunctionProvider } from "../index";
import {
  ButtonState,
  ButtonPadButton,
} from "../function_providers/ButtonFunctionProvider";
import { getIcon } from "../utils/svg";
import {
  buttonStateMap,
  ButtonStateMap,
} from "../function_providers/ButtonFunctionProvider";

/**
 * Each of the possible keys that could be pressed
 */
export enum PossibleKeys {
  w = "w",
  a = "a",
  s = "s",
  d = "d",
  q = "q",
  e = "e",
  z = "z",
  x = "x",
  ArrowUp = "ArrowUp",
  ArrowLeft = "ArrowLeft",
  ArrowDown = "ArrowDown",
  ArrowRight = "ArrowRight",
}

const possibleKeysArray = Object.values(PossibleKeys);

export const KeyboardControl = (props: CustomizableComponentProps) => {
  const [mode, setMode] = React.useState<Mode>(Mode.Base);
  const [keyPressed, setKeyPressed] = React.useState(false);
  const [activeMode, setActiveMode] = React.useState(1);
  const [toggleState, setToggleState] = React.useState(true);
  const [toggleSVG, setToggleSVG] = React.useState(false);

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

  /**
   * The state of each keyboard button, allowing the input to only run
   */
  const [keyState, setKeyState] = React.useState({
    w: false,
    a: false,
    s: false,
    d: false,
    q: false,
    e: false,
    z: false,
    x: false,
    ArrowUp: false,
    ArrowLeft: false,
    ArrowDown: false,
    ArrowRight: false,
  });

  /**
   * Button pad buttons for each input for rendering the correct icon for the button
   */
  const buttonImages = {
    [Mode.Base]: {
      w: ButtonPadButton.BaseForward,
      a: ButtonPadButton.BaseRotateLeft,
      s: ButtonPadButton.BaseReverse,
      d: ButtonPadButton.BaseRotateRight,
    },
    [Mode.Arm]: {
      w: ButtonPadButton.ArmLift,
      a: ButtonPadButton.ArmRetract,
      s: ButtonPadButton.ArmLower,
      d: ButtonPadButton.ArmExtend,
    },
    [Mode.Wrist]: {
      w: ButtonPadButton.WristPitchUp,
      a: ButtonPadButton.WristRotateIn,
      s: ButtonPadButton.WristPitchDown,
      d: ButtonPadButton.WristRotateOut,
      q: ButtonPadButton.WristRollLeft,
      e: ButtonPadButton.WristRollRight,
    },
  };

  /**
   * Toggle logic for keyboard inputs and showing SVGs
   */
  const toggleButton = () => {
    setToggleState((prevToggleState) => !prevToggleState);
    console.log("keys on?", toggleState);
    if (!toggleState || customizing) {
      releaseAllKeys();
    }
  };

  const toggleIcons = () => {
    setToggleSVG((prevToggleSVG) => !prevToggleSVG);
  };

  const releaseAllKeys = () => {
    Object.keys(keyState).forEach((key) => {
      if (keyState[key]) {
        handleKeyRelease({ key });
      }
    });
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
          setMode(Mode.Arm);
          setActiveMode(2);
          console.log("arm mode enabled");
          break;
        case "3":
          setMode(Mode.Wrist);
          setActiveMode(3);
          console.log("wrist mode enabled");
          break;
      }

      let functs = keyboardFunctionProvider.provideFunctions(mode, event.key);

      if (possibleKeysArray.includes(event.key as PossibleKeys)) {
        functs.onClick();
      } else {
        console.log("Unexpected key pressed", event.key);
      }
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
      window.removeEventListener("keydown", handleKeyPress);
      window.removeEventListener("keyup", handleKeyRelease);
      releaseAllKeys();
    }
    return () => {
      window.removeEventListener("keydown", handleKeyPress);
      window.removeEventListener("keyup", handleKeyRelease);
    };
  }, [toggleState, handleKeyPress, handleKeyRelease, customizing]);

  return (
    <div
      className={className("keyboard-container", { customizing, selected })}
      onClick={handleSelect}
    >
      <div className="keyboard-row"></div>
      <div className="keyboard-row">
        <div className="keyboard-column">
          <div className="toggle-row">
            <button
              className={`${toggleState ? "toggle-key active" : "toggle-key off"}`}
              onClick={toggleButton}
              disabled={customizing}
            >
              {" "}
              {toggleState ? "KEYS ON" : "KEYS OFF"}{" "}
            </button>
            <button
              onClick={toggleIcons}
              className={`${toggleSVG ? "toggle-icons" : ""}`}
              disabled={customizing}
            >
              ICONS
            </button>
          </div>
          <div className="mode-button">
            <button
              className={`${activeMode === 1 ? "mode-button active" : "mode-button"}`}
              disabled
            >
              {" "}
              1{" "}
            </button>
            <button
              className={`${activeMode === 2 ? "mode-button active" : "mode-button"}`}
              disabled
            >
              {" "}
              2{" "}
            </button>
            <button
              className={`${activeMode === 3 ? "mode-button active" : "mode-button"}`}
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
            {activeMode === 3 && (
              <button
                className={`keyboard-button ${buttonStateMap.get(buttonImages[Mode.Wrist].q)} ${toggleSVG ? "images" : ""}`}
                disabled
              >
                {toggleSVG ? (
                  <img
                    src={getIcon(ButtonPadButton.WristRollLeft)}
                    height="15"
                    width="15"
                  />
                ) : (
                  "Q"
                )}
              </button>
            )}
            <button
              className={`keyboard-button ${buttonStateMap.get(buttonImages[mode].w)} ${toggleSVG ? "images" : ""}`}
              disabled
            >
              {toggleSVG ? (
                <img
                  src={getIcon(buttonImages[mode].w)}
                  className={`${buttonStateMap.get(buttonImages[mode].w)}`}
                />
              ) : (
                "W"
              )}
            </button>
            {activeMode === 3 && (
              <button
                className={`keyboard-button ${buttonStateMap.get(buttonImages[Mode.Wrist].e)} ${toggleSVG ? "images" : ""}`}
                disabled
              >
                {toggleSVG ? (
                  <img
                    src={getIcon(ButtonPadButton.WristRollRight)}
                    height="15"
                    width="15"
                  />
                ) : (
                  "E"
                )}
              </button>
            )}
          </div>
          <div className="controls-column">
            <button
              className={`keyboard-button ${buttonStateMap.get(buttonImages[mode].a)} ${toggleSVG ? "images" : ""}`}
              disabled
            >
              {toggleSVG ? <img src={getIcon(buttonImages[mode].a)} /> : "A"}
            </button>
            <button
              className={`keyboard-button ${buttonStateMap.get(buttonImages[mode].s)} ${toggleSVG ? "images" : ""}`}
              disabled
            >
              {toggleSVG ? <img src={getIcon(buttonImages[mode].s)} /> : "S"}
            </button>
            <button
              className={`keyboard-button ${buttonStateMap.get(buttonImages[mode].d)} ${toggleSVG ? "images" : ""}`}
              disabled
            >
              {toggleSVG ? <img src={getIcon(buttonImages[mode].d)} /> : "D"}
            </button>
          </div>
        </div>
      </div>
      <div className="keyboard-row">
        <div className="controls-container">
          Camera Controls <br />
          <div className="controls-column">
            <button
              className={`keyboard-button ${buttonStateMap.get(ButtonPadButton.CameraTiltUp)}`}
              disabled
            >
              {" "}
              ^{" "}
            </button>
          </div>
          <div className="controls-column">
            <button
              className={`keyboard-button ${buttonStateMap.get(ButtonPadButton.CameraPanLeft)}`}
              disabled
            >
              {" "}
              &lt;{" "}
            </button>
            <button
              className={`keyboard-button ${buttonStateMap.get(ButtonPadButton.CameraTiltDown)}`}
              disabled
            >
              {" "}
              v{" "}
            </button>
            <button
              className={`keyboard-button ${buttonStateMap.get(ButtonPadButton.CameraPanRight)}`}
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
              className={`keyboard-button ${buttonStateMap.get(ButtonPadButton.GripperClose)} ${toggleSVG ? "images" : ""}`}
              disabled
            >
              {toggleSVG ? (
                <img src={getIcon(ButtonPadButton.GripperClose)} />
              ) : (
                "Z"
              )}
            </button>
            <button
              className={`keyboard-button ${buttonStateMap.get(ButtonPadButton.GripperOpen)} ${toggleSVG ? "images" : ""}`}
              disabled
            >
              {toggleSVG ? (
                <img src={getIcon(ButtonPadButton.GripperOpen)} />
              ) : (
                "X"
              )}
            </button>
          </div>
        </div>
      </div>
      <div className="keyboard-row"></div>
    </div>
  );
};
