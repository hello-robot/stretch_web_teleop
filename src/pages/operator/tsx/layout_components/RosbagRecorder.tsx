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
    const [bagLabel, setBagLabel] = useState("recording");
    const [recordingInfo, setRecordingInfo] = useState<any>(null);

    const handleClick = async () => {
        setError(null);
        if (!isRecording) {
            // Start recording
            try {
                const res = await fetch("/start_rosbag", { 
                    method: "POST",
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({ label: bagLabel })
                });
                if (res.ok) {
                    const data = await res.json();
                    setIsRecording(true);
                    setRecordingInfo(data);
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
                    setRecordingInfo(null);
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
            
            {!isRecording && (
                <div style={{ marginBottom: 10 }}>
                    <input
                        type="text"
                        value={bagLabel}
                        onChange={(e) => setBagLabel(e.target.value)}
                        placeholder="Enter rosbag label"
                        style={{
                            padding: "5px 10px",
                            border: "1px solid #ccc",
                            borderRadius: "4px",
                            fontSize: "14px",
                            width: "150px"
                        }}
                    />
                </div>
            )}
            
            <Tooltip text={!isRecording ? "Record rosbag" : "Save rosbag"} position="top">
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
            
            {isRecording && (
                <div style={{ marginTop: 10, fontSize: "12px" }}>
                    <span style={{ color: "red" }}>● Recording</span>
                    {recordingInfo && (
                        <div style={{ marginTop: 5 }}>
                            <div><strong>File:</strong> {recordingInfo.bagName}</div>
                            <div><strong>Location:</strong> {recordingInfo.flashDrive ? "Flash Drive" : "Local Storage"}</div>
                            <div><strong>Path:</strong> {recordingInfo.dir}</div>
                        </div>
                    )}
                </div>
            )}
            
            {error && <div style={{ color: "red", marginTop: 10 }}>{error}</div>}
        </div>
    );
}; 