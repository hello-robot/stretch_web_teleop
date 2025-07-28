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
    const [showRosbags, setShowRosbags] = useState(false);
    const [rosbags, setRosbags] = useState<any[]>([]);

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

    const handleListRosbags = async () => {
        try {
            const res = await fetch("/list_rosbags");
            if (res.ok) {
                const data = await res.json();
                setRosbags(data.rosbags);
                setShowRosbags(!showRosbags);
            } else {
                setError("Failed to list rosbags");
            }
        } catch (e) {
            setError("Failed to list rosbags");
        }
    };

    const formatFileSize = (bytes: number) => {
        if (bytes === 0) return '0 Bytes';
        const k = 1024;
        const sizes = ['Bytes', 'KB', 'MB', 'GB'];
        const i = Math.floor(Math.log(bytes) / Math.log(k));
        return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
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
            
            <div style={{ marginTop: 10 }}>
                <button
                    onClick={handleListRosbags}
                    style={{
                        padding: "5px 10px",
                        border: "1px solid #ccc",
                        borderRadius: "4px",
                        fontSize: "12px",
                        cursor: "pointer"
                    }}
                >
                    {showRosbags ? "Hide" : "Show"} Recent Rosbags
                </button>
                
                {showRosbags && rosbags.length > 0 && (
                    <div style={{ 
                        marginTop: 10, 
                        maxHeight: "200px", 
                        overflowY: "auto",
                        border: "1px solid #ccc",
                        borderRadius: "4px",
                        padding: "10px",
                        fontSize: "12px"
                    }}>
                        {rosbags.map((bag, index) => (
                            <div key={index} style={{ 
                                borderBottom: index < rosbags.length - 1 ? "1px solid #eee" : "none",
                                padding: "5px 0"
                            }}>
                                <div><strong>{bag.name}</strong></div>
                                <div>Location: {bag.location}</div>
                                <div>Size: {formatFileSize(bag.size)}</div>
                                <div>Modified: {new Date(bag.modified).toLocaleString()}</div>
                            </div>
                        ))}
                    </div>
                )}
                
                {showRosbags && rosbags.length === 0 && (
                    <div style={{ marginTop: 10, fontSize: "12px", color: "#666" }}>
                        No rosbags found
                    </div>
                )}
            </div>
        </div>
    );
}; 