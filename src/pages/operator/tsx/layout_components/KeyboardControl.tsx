import React from "react";
import {
  CustomizableComponentProps,
  SharedState,
  isSelected,
} from "./CustomizableComponent";
import { keyboardFunctionProvider } from "operator/tsx/index";
import { Mode } from "../function_providers/KeyboardFunctionProvider";

export const KeyboardControl = (props: CustomizableComponentProps) => {
  const [mode, setMode] = React.useState<Mode>(Mode.Base);
  const [keyPressed, setKeyPressed] = React.useState<boolean>(false);

  const { customizing } = props.sharedState;
  const selected = isSelected(props);
  const functions = [];

  function handleSelect(event: React.MouseEvent<HTMLDivElement>) {
    event.stopPropagation();
    props.sharedState.onSelect(props.definition, props.path);
  }

  const handleKeyPress = React.useCallback(
    (event) => {
      if (keyPressed === true) {
        return;
      }
      console.log(keyPressed);

      setKeyPressed(true);

      switch (event.key) {
        case "1":
          setMode(Mode.Base);
          console.log("base mode enabled");
          break;
        case "2":
          setMode(Mode.Wrist);
          console.log("wrist mode enabled");
          break;
        case "3":
          setMode(Mode.Arm);
          console.log("arm mode enabled");
          break;
      }

      let functs = keyboardFunctionProvider.provideFunctions(mode, event.key);
      functs.onClick();
    },
    [mode, keyPressed],
  );

  const handleKeyRelease = React.useCallback(
    (event) => {
      console.log("Key Released");
      setKeyPressed(false);
      let functs = keyboardFunctionProvider.provideFunctions(mode, event.key);
      functs.onRelease();
    },
    [mode, keyPressed],
  );

  React.useEffect(() => {
    //window.onkeydown  = handleKeyPress;
    // window.onkeyup = function(){
    //   this.onkeydown = handleKeyPress;
    // }
    window.addEventListener("keydown", handleKeyPress);
    window.addEventListener("keyup", handleKeyRelease);
    return () => {
      window.removeEventListener("keydown", handleKeyPress);
      window.removeEventListener("keyup", handleKeyRelease);
    };
  }, [handleKeyPress, handleKeyRelease]);

  return (
    <>
      <div>
        <button>{mode}</button>
      </div>
    </>
  );
};
