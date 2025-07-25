import React, { useState } from "react";
import { Tooltip } from "../static_components/Tooltip";
import RadioButtonCheckedIcon from "@mui/icons-material/RadioButtonChecked";
import SaveIcon from "@mui/icons-material/Save";
import "operator/css/MovementRecorder.css";
import "operator/css/basic_components.css";
import { CustomizableComponentProps } from "./CustomizableComponent";

export const RosbagRecorder = (props: CustomizableComponentProps) => {
    const hideLabels = props.hideLabels ?? false;
    const [isRecording, setIsRecording] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const handleClick = async () => {
        setError(null);
        if (!isRecording) {
            // Start recording
            try {
                const res = await fetch("/start_rosbag", { method: "POST" });
                if (res.ok) {
                    setIsRecording(true);
                } else {
                    const data = await res.json();
                    setError(data.error || "Failed to start recording");
                }
            } catch (e) {
                setError("Failed to start recording");
            }
        } else {
            // Stop recording
            try {
                const res = await fetch("/stop_rosbag", { method: "POST" });
                if (res.ok) {
                    setIsRecording(false);
                } else {
                    const data = await res.json();
                    setError(data.error || "Failed to stop recording");
                }
            } catch (e) {
                setError("Failed to stop recording");
            }
        }
    };

    return (
        <div id="movement-recorder-container">
            <span>Demonstration Recorder</span>
            <Tooltip text={!isRecording ? "Record rosbag" : "Save rosbag"} position="top">
                <button
                    className="save-btn btn-label"
                    onClick={handleClick}
                    style={{ minWidth: 120, minHeight: 40, fontWeight: "bold" }}
                >
                    {!isRecording ? (
                        <>
                            <i hidden={hideLabels}>Record</i>
                            <RadioButtonCheckedIcon />
                        </>
                    ) : (
                        <>
                            <i hidden={hideLabels}>Save</i>
                            <SaveIcon />
                        </>
                    )}
                </button>
            </Tooltip>
            {isRecording && <span style={{ color: "red", marginLeft: 10 }}>● Recording</span>}
            {error && <div style={{ color: "red" }}>{error}</div>}
        </div>
    );
}; 