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
        <div {...selectProp} style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: "2px", marginBottom: "0", paddingBottom: "0" }}>
            <div style={{ 
                position: "relative", 
                width: "100%", 
                height: 24,
                display: "flex",
                justifyContent: "center",
                alignItems: "center"
            }}>
                <Tooltip text={!isRecording ? "Record demo" : "Stop recording"} position="top">
                    <button
                        className="save-btn btn-label"
                        onClick={handleClick}
                        style={{
                            width: 180,
                            height: 40,
                            fontWeight: "bold",
                            display: "flex",
                            alignItems: "center",
                            justifyContent: "center",
                            gap: "8px",
                            whiteSpace: "nowrap",
                            textAlign: "center",
                            position: "absolute",
                            top: -8,
                            left: "50%",
                            transform: "translateX(-50%)"
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
            </div>
            <div style={{ 
                height: 32,
                width: "100%",
                display: "flex",
                alignItems: "center",
                justifyContent: "center",
                marginTop: "12px"
            }}>
                {error && <div style={{ color: "red", textAlign: "center", width: "100%" }}>{error}</div>}
                {successMessage && <div style={{ color: "green", fontWeight: "bold", textAlign: "center", width: "100%" }}>{successMessage}</div>}
                {isRecording && <div style={{ color: "orange", fontWeight: "bold", textAlign: "center", width: "100%" }}>Recording in progress...</div>}
            </div>
        </div>
    );
}; 