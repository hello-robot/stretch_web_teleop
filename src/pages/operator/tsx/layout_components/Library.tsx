import React, { useState } from "react";
import {
    CustomizableComponentProps,
    isSelected,
} from "./CustomizableComponent";
import { className, RobotPose } from "shared/util";
import "operator/css/Library.css";

// Mapping from user's joint order to ValidJoints (filtered to only valid joints)
const JOINT_MAPPING = [
    null,                   
    null,                    
    "joint_lift",            
    "wrist_extension",       // Sum of joint_arm_l0 + joint_arm_l1 + joint_arm_l2 + joint_arm_l3
    null,                   
    null,                    
    null,                    
    "joint_wrist_yaw",      
    "joint_head_pan",        
    "joint_head_tilt",                        
    "joint_wrist_pitch",    
    "joint_wrist_roll",
    null,      
    "joint_gripper_finger_left" 
];

interface SavedPosition {
    name: string;
    jointStates: string;
    timestamp: Date;
}

/**
 * A library component that displays functions and saved positions
 * 
 * @param props {@link CustomizableComponentProps}
 */
export const Library = (props: CustomizableComponentProps) => {
    const { customizing } = props.sharedState;
    const selected = isSelected(props);
    
    // Responsive for screen dimensions 
    const isSmallScreen = window.innerWidth < 1200;
    const functionItemStyle = {
        fontSize: isSmallScreen ? "14px" : "16px",
        padding: isSmallScreen ? "8px 10px" : "10px 14px"
    };
    const descriptionStyle = {
        fontSize: isSmallScreen ? "13px" : "15px",
        lineHeight: isSmallScreen ? "1.4" : "1.5"
    };
    

    
    // Load saved positions from session storage or use defaults
    const getInitialSavedPositions = (): SavedPosition[] => {
        const sessionPositions = sessionStorage.getItem('librarySavedPositions');
        if (sessionPositions) {
            try {
                const parsed = JSON.parse(sessionPositions);
                return parsed.map((pos: any) => ({
                    ...pos,
                    timestamp: new Date(pos.timestamp)
                }));
            } catch (error) {
                console.error("Error parsing saved positions:", error);
            }
        }
        // Default positions if no session data
        return [
            { name: "stowGripper", jointStates: "[0.0, -0.497, 3.19579]", timestamp: new Date() },
            { name: "centerWrist", jointStates: "[0.0, 0.0, 0.0]", timestamp: new Date() },
        ];
    };
    
    const [savedPositions, setSavedPositions] = useState<SavedPosition[]>(getInitialSavedPositions());
    const [showModal, setShowModal] = useState(false);
    const [newPositionName, setNewPositionName] = useState("");
    const [newJointStates, setNewJointStates] = useState("");
    const [validationError, setValidationError] = useState<string>("");

    /** Callback when component is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<HTMLDivElement>) => {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    };

    const selectProp = customizing ? { onClick: onSelect } : {};

    // Function to parse joint values from user input
    const parseJointValues = (valuesString: string): RobotPose => {
        // Remove brackets and split by comma
        const cleanString = valuesString.replace(/[\[\]]/g, '');
        const values = cleanString.split(',').map(v => parseFloat(v.trim()));
        const pose: RobotPose = {};
        
        // Calculate wrist_extension as sum of arm segments (positions 3, 4, 5, 6)
        let wristExtensionSum = 0;
        if (values.length >= 7) {
            wristExtensionSum = values[3] + values[4] + values[5] + values[6]; // joint_arm_l3 + joint_arm_l2 + joint_arm_l1 + joint_arm_l0
        }
        
        values.forEach((value, index) => {
            if (index < JOINT_MAPPING.length && !isNaN(value)) {
                const jointName = JOINT_MAPPING[index];
                // Only add to pose if jointName is not null (valid joint)
                if (jointName !== null) {
                    if (jointName === "wrist_extension") {
                        pose[jointName as keyof RobotPose] = wristExtensionSum;
                    } else {
                        pose[jointName as keyof RobotPose] = value;
                    }
                }
            }
        });
        
        return pose;
    };

    // Validate joint states input
    const validateJointStates = (input: string): boolean => {
        try {
            // Check if it's a valid array format
            // TODO: add checking to see if its from rosbag recording
            const trimmed = input.trim();
            if (!trimmed.startsWith('[') || !trimmed.endsWith(']')) {
                return false;
            }
        
            const array = JSON.parse(trimmed);
            if (!Array.isArray(array)) {
                return false;
            }
    
            return array.every(item => typeof item === 'number');
        } catch {
            return false;
        }
    };

    // Handle adding new position
    const handleAddPosition = () => {
        if (newPositionName.trim() && newJointStates.trim()) {
            // Validate joint states format
            if (!validateJointStates(newJointStates)) {
                setValidationError("Invalid joint states, copy and paste position from demo recording.");
                return;
            }
            
            try {
                // Parse the joint values into a proper RobotPose
                const pose = parseJointValues(newJointStates);
                
                const newPosition: SavedPosition = {
                    name: newPositionName.trim(),
                    jointStates: newJointStates.trim(),
                    timestamp: new Date()
                };
                const updatedPositions = [...savedPositions, newPosition];
                setSavedPositions(updatedPositions);
                
                // Save to session storage
                sessionStorage.setItem('librarySavedPositions', JSON.stringify(updatedPositions));
                
                // Add to program editor's autocomplete and syntax highlighting
                props.sharedState.addSavedPosition?.(newPositionName.trim());
                
                // Store the pose in the shared state for the program editor to use
                if ((props.sharedState as any).addCustomPose) {
                    (props.sharedState as any).addCustomPose(newPositionName.trim(), pose);
                }
                
                setNewPositionName("");
                setNewJointStates("");
                setValidationError("");
                setShowModal(false);
            } catch (error) {
                console.error("Error parsing joint values:", error);
                setValidationError("Invalid joint states, copy and paste position from demo recording.");
            }
        }
    };

    // Handle cancel button
    const handleCancel = () => {
        setNewPositionName("");
        setNewJointStates("");
        setValidationError("");
        setShowModal(false);
    };

    return (
        <div
            className={className("library-root", {
                customizing,
                selected,
            })}
            style={{
                fontSize: window.innerWidth < 1200 ? "14px" : "16px",
                padding: window.innerWidth < 1200 ? "8px" : "12px"
            }}
            {...selectProp}
        >
            <div className="library-content" style={{
                gap: window.innerWidth < 1200 ? "12px" : "16px"
            }}>
                <div className="library-sections-container" style={{
                    gap: window.innerWidth < 1200 ? "12px" : "16px"
                }}>
                    {/* Functions Section */}
                    <div className="library-section" style={{
                        gap: window.innerWidth < 1200 ? "8px" : "12px"
                    }}>
                        <h3 className="library-section-title" style={{ fontWeight: "600", color: "#ff8c00" }}>Robot Functions</h3>
                        
                        <div className="library-subsection">
                            <div className="library-text">
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.insertTextAtCursor?.("Move_Arm_to_Config()\n")}
                                        style={{ marginBottom: "12px" }}
                                    >
                                        <span style={{ fontWeight: "600", fontSize: "1.1em" }}>Move_Arm_to_Config</span>(<span style={{ color: '#6c757d' }}>configuration name</span>)
                                    </div>
                                    <div className="function-description" style={{ color: '#495057', fontSize: '1em', lineHeight: '1.5' }}>
                                        Adjust the lift and extension of the robot's arm.{'\n'}
                                        Input: Saved configuration from demo recording.
                                    </div>
                                </div>
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.insertTextAtCursor?.("Adjust_Gripper_Width()\n")}
                                        style={{ marginBottom: "12px" }}
                                    >
                                        <span style={{ fontWeight: "600", fontSize: "1.1em" }}>Adjust_Gripper_Width</span>(<span style={{ color: '#6c757d' }}>configuration name</span>)
                                    </div>
                                    <div className="function-description" style={{ color: '#495057', fontSize: '1em', lineHeight: '1.5' }}>
                                        Adjust the width of the robot's gripper. {'\n'}
                                        Input: Saved configuration from demo recording.
                                    </div>
                                </div>
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.insertTextAtCursor?.("Rotate_Wrist_to_Config()\n")}
                                        style={{ marginBottom: "12px" }}
                                    >
                                        <span style={{ fontWeight: "600", fontSize: "1.1em" }}>Rotate_Wrist_to_Config</span>(<span style={{ color: '#6c757d' }}>configuration name</span>)
                                    </div>
                                    <div className="function-description" style={{ color: '#495057', fontSize: '1em', lineHeight: '1.5' }}>
                                        Adjust the angle of the robot's wrist.{'\n'}
                                        Input: Saved configuration from demo recording.
                                    </div>
                                </div>
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.insertTextAtCursor?.("Reset_Robot()\n")}
                                        style={{ marginBottom: "12px" }}
                                    >
                                        <span style={{ fontWeight: "600", fontSize: "1.1em" }}>Reset_Robot</span>()
                                    </div>
                                    <div className="function-description" style={{ color: '#495057', fontSize: '1em', lineHeight: '1.5' }}>
                                        Reset the robot to its home position.{'\n'}
                                        Input: N/A
                                    </div>
                                </div>
                            </div>
                        </div>
                        
                        <h3 className="library-section-title" style={{ fontWeight: "600", marginTop: "16px", color: "#28a745" }}>Human Functions</h3>
                        <div className="library-subsection">
                            <div className="library-text">
                                <div className="function-group">
                                                                                <div 
                                                className="library-function-item"
                                                onClick={() => props.sharedState.insertTextAtCursor?.("Pause_And_Confirm()\n")}
                                                style={{ marginBottom: "12px" }}
                                            >
                                                <span style={{ fontWeight: "600", fontSize: "1.1em" }}>Pause_And_Confirm</span>()
                                            </div>
                                                                                <div className="function-description" style={{ color: '#495057', fontSize: '1em', lineHeight: '1.5' }}>
                                        Pause execution and wait for your confirmation.{'\n'}
                                        Input(Optional): Message shown while execution is paused.
                                    </div>
                                </div>
                                                                        <div className="function-group">
                                            <div 
                                                className="library-function-item"
                                                onClick={() => props.sharedState.insertTextAtCursor?.("Take_Control()\n")}
                                                style={{ marginBottom: "12px" }}
                                            >
                                                <span style={{ fontWeight: "600", fontSize: "1.1em" }}>Take_Control</span>()
                                            </div>
                                                                                <div className="function-description" style={{ color: '#495057', fontSize: '1em', lineHeight: '1.5' }}>
                                        Control the robot by tele-operating it.{'\n'}
                                        Input: N/A
                                    </div>
                                        </div>
                            </div>
                        </div>
                    </div>
                    
                    {/* Saved Configurations Section */}
                    <div className="library-section" style={{ display: "flex", flexDirection: "column", height: "100%" }}>
                        <h3 className="library-section-title">Saved Configurations</h3>
                        <div className="library-subsection" style={{ display: "flex", flexDirection: "column", flex: 1 }}>
                            <div className="library-text">
                                {savedPositions.map((position, index) => (
                                    <div 
                                        key={index}
                                        className="library-function-item"
                                        onClick={() => props.sharedState.insertTextAtCursor?.(position.name)}
                                        style={{ fontSize: "1em" }}
                                    >
                                        {position.name}
                                    </div>
                                ))}
                                <div style={{ 
                                    paddingTop: "8px",
                                    paddingBottom: "8px"
                                }}>
                                    <button 
                                        className="add-position-btn"
                                        onClick={() => setShowModal(true)}
                                        style={{
                                            width: "100%",
                                            textAlign: "center",
                                            display: "flex",
                                            justifyContent: "center",
                                            alignItems: "center",
                                            whiteSpace: "nowrap"
                                        }}
                                    >
                                        + Add Configuration
                                    </button>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
                
                {/* Clear button - fixed at bottom */}
                <div style={{ 
                    position: "sticky",
                    bottom: 0,
                    background: "var(--background-color)",
                    borderTop: "1px solid var(--border-color)",
                    padding: "8px 16px",
                    display: "flex", 
                    justifyContent: "flex-end",
                    zIndex: 10
                }}>
                    <button 
                        className="clear-positions-btn"
                        onClick={() => {
                            // Reset to empty positions
                            setSavedPositions([]);
                            sessionStorage.removeItem('librarySavedPositions');
                        }}
                        style={{
                            minWidth: "auto",
                            maxWidth: "auto",
                            flex: "none"
                        }}
                    >
                        Clear All
                    </button>
                </div>
            </div>
            
            {/* Modal */}
            {showModal && (
                <div style={{
                    position: "fixed",
                    top: 0,
                    left: 0,
                    width: "100vw",
                    height: "100vh",
                    background: "rgba(0,0,0,0.4)",
                    zIndex: 1000,
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center"
                }}>
                    <div style={{
                        background: "white",
                        borderRadius: 8,
                        padding: 32,
                        minWidth: 400,
                        maxWidth: 500,
                        boxShadow: "0 2px 16px rgba(0,0,0,0.2)",
                        textAlign: "center"
                    }}>
                        <div style={{ fontSize: "1.2em", marginBottom: 24 }}>
                            Add New Configuration
                        </div>
                        <div style={{ 
                            display: "flex", 
                            flexDirection: "column", 
                            gap: 16,
                            textAlign: "left",
                            marginBottom: 24
                        }}>
                            <div>
                                <label style={{ 
                                    display: "block", 
                                    marginBottom: 6,
                                    fontWeight: "bold",
                                    fontSize: "0.9em"
                                }}>
                                    Configuration Name
                                </label>
                                <input
                                    type="text"
                                    value={newPositionName}
                                    onChange={(e) => setNewPositionName(e.target.value)}
                                    placeholder="e.g.pickup_pose"
                                    style={{
                                        width: "100%",
                                        padding: "8px 12px",
                                        border: "1px solid #ccc",
                                        borderRadius: 4,
                                        fontSize: "0.9em"
                                    }}
                                />
                            </div>
                            <div>
                                <label style={{ 
                                    display: "block", 
                                    marginBottom: 6,
                                    fontWeight: "bold",
                                    fontSize: "0.9em"
                                }}>
                                    Joint States
                                </label>
                                <input
                                    type="text"
                                    value={newJointStates}
                                    onChange={(e) => setNewJointStates(e.target.value)}
                                    placeholder="e.g.[0.0,...,0.0]"
                                    style={{
                                        width: "100%",
                                        padding: "8px 12px",
                                        border: validationError ? "1px solid #f44336" : "1px solid #ccc",
                                        borderRadius: 4,
                                        fontSize: "0.9em"
                                    }}
                                />
                                {validationError && (
                                    <div style={{ 
                                        color: "#f44336", 
                                        fontSize: "0.75em", 
                                        marginTop: "4px"
                                    }}>
                                        Invalid joint states, copy and paste position from demo recording.
                                    </div>
                                )}
                            </div>
                        </div>
                        <div style={{ display: "flex", justifyContent: "center", gap: 16 }}>
                            <button
                                style={{
                                    background: "var(--btn-gray)",
                                    color: "var(--text-color)",
                                    border: "none",
                                    borderRadius: 4,
                                    padding: "8px 20px",
                                    fontWeight: "normal",
                                    fontSize: "1em",
                                    cursor: "pointer"
                                }}
                                onClick={handleCancel}
                            >
                                Cancel
                            </button>
                            <button
                                style={{
                                    background: "#4caf50",
                                    color: "white",
                                    border: "none",
                                    borderRadius: 4,
                                    padding: "8px 20px",
                                    fontWeight: "bold",
                                    fontSize: "1em",
                                    cursor: "pointer",
                                    opacity: (!newPositionName.trim() || !newJointStates.trim()) ? 0.5 : 1
                                }}
                                onClick={handleAddPosition}
                                disabled={!newPositionName.trim() || !newJointStates.trim()}
                            >
                                Save Configuration
                            </button>
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
}; 