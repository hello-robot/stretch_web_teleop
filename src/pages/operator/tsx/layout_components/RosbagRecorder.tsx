import React, { useState } from "react";
import { CustomizableComponentProps } from "./CustomizableComponent";
import "operator/css/basic_components.css";

export const RosbagRecorder = (props: CustomizableComponentProps) => {
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
        <div className="rosbag-recorder-component">
            <button
                className={isRecording ? "recording" : ""}
                onClick={handleClick}
                style={{ minWidth: 120, minHeight: 40, fontWeight: "bold" }}
            >
                {isRecording ? "Save" : "Record"}
            </button>
            {isRecording && <span style={{ color: "red", marginLeft: 10 }}>● Recording</span>}
            {error && <div style={{ color: "red" }}>{error}</div>}
        </div>
    );
}; 