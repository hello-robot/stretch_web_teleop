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

  const { customizing } = props.sharedState;
  const selected = isSelected(props);
  const functions = [];

  function handleSelect(event: React.MouseEvent<HTMLDivElement>) {
    event.stopPropagation();
    props.sharedState.onSelect(props.definition, props.path);
  }

  const handleKeyPress = React.useCallback(
    (event) => {
      switch (event.key) {
        case "1":
          setMode(Mode.Base);
          console.log("base mode enabled");
          break;
        case "2":
          setMode(Mode.Arm);
          console.log("arm mode enabled");
          break;
        case "3":
          setMode(Mode.Wrist);
          console.log("wrist mode enabled");
          break;
      }

      let functs = keyboardFunctionProvider.provideFunctions(mode, event.key);
      functs.onClick();
    },
    [mode],
  );

  React.useEffect(() => {
    window.addEventListener("keydown", handleKeyPress);
    return () => {
      window.removeEventListener("keydown", handleKeyPress);
    };
  }, [handleKeyPress]);

  return (
    <>
      <div>
        <button>{mode}</button>
      </div>
    </>
  );
};
