import React, { useState } from "react";
import { Tooltip } from "../static_components/Tooltip";
import RadioButtonCheckedIcon from "@mui/icons-material/RadioButtonChecked";
import SaveIcon from "@mui/icons-material/Save";
import { CustomizableComponentProps, isSelected } from "./CustomizableComponent";
import { className } from "shared/util";
import "operator/css/basic_components.css";

export const RosbagRecorder = (props: CustomizableComponentProps) => {
    const customizing = props.sharedState?.customizing ?? false;
    const selected = props.sharedState ? isSelected(props) : false;
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

    /** Callback when component is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<HTMLDivElement>) => {
        event.stopPropagation();
        props.sharedState?.onSelect(props.definition, props.path);
    };

    // In customizing state add onClick callback
    const selectProp = customizing ? { onClick: onSelect } : {};

    return (
        <div
            className={className("operator-rosbag-recorder", {
                customizing,
                selected,
            })}
            {...selectProp}
        >
            <Tooltip text={!isRecording ? "Record demo" : "Stop recording"} position="top">
                <button
                    className="save-btn btn-label"
                    onClick={handleClick}
                    style={{
                        minWidth: 120,
                        minHeight: 40,
                        fontWeight: "bold",
                        display: "flex",
                        alignItems: "center",
                        justifyContent: "center"
                    }}
                >
                    {!isRecording ? "Record Demo" : "Stop Recording"}
                </button>
            </Tooltip>
            {error && <div style={{ color: "red", marginTop: 8 }}>{error}</div>}
        </div>
    );
}; 