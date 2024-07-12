import React from "react";

enum Mode {
  Base = "base",
  Arm = "arm",
  Wrist = "wrist",
}

export default function KeyboardFunctionProvider() {
  const [mode, setMode] = React.useState();

  const handleKeyPress = React.useCallback(
    (event) => {
      switch (event.key) {
        case "w":
          HandleW();
          break;
        case "a":
          HandleA();
          break;
        case "s":
          HandleS();
          break;
        case "d":
          HandleD();
          break;
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
        default:
          break;
      }

      switch (event.keyCode) {
        case 38:
          console.log("camera moved up");
          break;
        case 37:
          console.log("camera moved left");
          break;
        case 40:
          console.log("camera moved down");
          break;
        case 39:
          console.log("camera moved right");
          break;
      }
    },
    [mode],
  );

  const HandleW = () => {
    switch (mode) {
      case Mode.Base:
        console.log("w pressed; base mode");
        break;
      case Mode.Arm:
        console.log("w pressed; arm mode");
        break;
      case Mode.Wrist:
        console.log("w pressed; wrist mode");
        break;
      default:
        break;
    }
  };

  const HandleA = () => {
    switch (mode) {
      case Mode.Base:
        console.log("a pressed; base mode");
        break;
      case Mode.Arm:
        console.log("a pressed; arm mode");
        break;
      case Mode.Wrist:
        console.log("a pressed; wrist mode");
        break;
      default:
        break;
    }
  };

  const HandleS = () => {
    switch (mode) {
      case Mode.Base:
        console.log("s pressed; base mode");
        break;
      case Mode.Arm:
        console.log("s pressed; arm mode");
        break;
      case Mode.Wrist:
        console.log("s pressed; wrist mode");
        break;
      default:
        break;
    }
  };

  const HandleD = () => {
    switch (mode) {
      case Mode.Base:
        console.log("d pressed; base mode");
        break;
      case Mode.Arm:
        console.log("d pressed; arm mode");
        break;
      case Mode.Wrist:
        console.log("d pressed; wrist mode");
        break;
      default:
        break;
    }
  };

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
}
