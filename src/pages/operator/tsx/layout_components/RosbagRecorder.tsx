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
    const [successMessage, setSuccessMessage] = useState<string | null>(null);

    const handleClick = async () => {
        setError(null);
        setSuccessMessage(null);
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
                    setSuccessMessage("Recording saved. Open demo in Foxglove and create your program!");
                    setTimeout(() => setSuccessMessage(null), 5000);
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

    const selectProp = customizing ? { onClick: onSelect } : {};

    return (
        <div {...selectProp}>
            <Tooltip text={!isRecording ? "Record demo" : "Stop recording"} position="top">
                <button
                    className="save-btn btn-label"
                    onClick={handleClick}
                    style={{
                        width: 140,
                        height: 40,
                        fontWeight: "bold",
                        display: "flex",
                        alignItems: "center",
                        justifyContent: "center",
                        gap: "8px"
                    }}
                >
                    {!isRecording ? (
                        <>
                            <RadioButtonCheckedIcon />
                            Record Demo
                        </>
                    ) : (
                        <>
                            <SaveIcon />
                            Save Demo
                        </>
                    )}
                </button>
            </Tooltip>
            {(error || successMessage || isRecording) && (
                <div style={{ 
                    height: 32, 
                    marginTop: 8,
                    display: "flex",
                    alignItems: "center"
                }}>
                    {error && <div style={{ color: "red" }}>{error}</div>}
                    {successMessage && <div style={{ color: "green", fontWeight: "bold" }}>{successMessage}</div>}
                    {isRecording && <div style={{ color: "orange", fontWeight: "bold" }}>Recording in progress...</div>}
                </div>
            )}
        </div>
    );
}; 