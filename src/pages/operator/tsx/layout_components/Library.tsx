import React, { useState } from "react";
import {
    CustomizableComponentProps,
    isSelected,
} from "./CustomizableComponent";
import { className, RobotPose } from "shared/util";
import "operator/css/Library.css";

// Mapping from user's joint order to ValidJoints (filtered to only valid joints)
const JOINT_MAPPING = [
    null,                    // joint_right_wheel (skip)
    null,                    // joint_left_wheel (skip)
    "joint_lift",            
    null,                    // joint_arm_l3 (skip)
    null,                    // joint_arm_l2 (skip)
    null,                    // joint_arm_l1 (skip)
    "joint_arm",             // 7th position (combined arm)
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
            { name: "testPose", jointStates: "[-1.61e-08, -2.94e-07, -2.80e-07, -2.06e-07, -2.05e-06, 2.93e-07, 1.38e-06, 1.44e-07, 9.33e-07]", timestamp: new Date() }
        ];
    };
    
    const [savedPositions, setSavedPositions] = useState<SavedPosition[]>(getInitialSavedPositions());
    const [showModal, setShowModal] = useState(false);
    const [newPositionName, setNewPositionName] = useState("");
    const [newJointStates, setNewJointStates] = useState("");

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
        
        values.forEach((value, index) => {
            if (index < JOINT_MAPPING.length && !isNaN(value)) {
                const jointName = JOINT_MAPPING[index];
                // Only add to pose if jointName is not null (valid joint)
                if (jointName !== null) {
                    pose[jointName as keyof RobotPose] = value;
                }
            }
        });
        
        return pose;
    };

    // Handle adding new position
    const handleAddPosition = () => {
        if (newPositionName.trim() && newJointStates.trim()) {
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
                setShowModal(false);
            } catch (error) {
                console.error("Error parsing joint values:", error);
                alert("Invalid joint values. Please check the format.");
            }
        }
    };

    // Handle cancel button
    const handleCancel = () => {
        setNewPositionName("");
        setNewJointStates("");
        setShowModal(false);
    };

    return (
        <div
            className={className("library-root", {
                customizing,
                selected,
            })}
            {...selectProp}
        >
            <div className="library-header">
                <h2 className="library-title">Library</h2>
            </div>
            
            <div className="library-content">
                <div className="library-sections-container">
                    {/* Functions Section */}
                    <div className="library-section">
                        <h3 className="library-section-title">Functions</h3>
                        
                        <div className="library-subsection">
                            <h4 className="library-subsection-title robot-heading">Robot</h4>
                            <div className="library-text">
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.addToProgramEditor?.("MoveEEToPose()")}
                                    >
                                        MoveEEToPose()
                                    </div>
                                    <div className="function-description">
                                        Move the robot's end effector to a specific pose.{'\n'}
                                        Input: Saved position from demo recording.
                                    </div>
                                </div>
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.addToProgramEditor?.("AdjustGripperWidth()")}
                                    >
                                        AdjustGripperWidth()
                                    </div>
                                    <div className="function-description">
                                        Adjust the width of the end effector.{'\n'}
                                        Input: Value betweem -0.37 and 0.17.
                                    </div>
                                </div>
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.addToProgramEditor?.("RotateEE()")}
                                    >
                                        RotateEE()
                                    </div>
                                    <div className="function-description">
                                        Adjust the angle of the end effector.{'\n'}
                                        Input: Saved position from demo recording.
                                    </div>
                                </div>
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.addToProgramEditor?.("ResetRobot()")}
                                    >
                                        ResetRobot()
                                    </div>
                                    <div className="function-description">
                                        Reset the robot to its home position.{'\n'}
                                        Input: N/A
                                    </div>
                                </div>
                            </div>
                        </div>
                        
                        <div className="library-subsection">
                            <h4 className="library-subsection-title human-heading">Human</h4>
                            <div className="library-text">
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.addToProgramEditor?.("PauseAndConfirm()")}
                                    >
                                        PauseAndConfirm()
                                    </div>
                                    <div className="function-description">
                                        Pause execution and wait for your confirmation.{'\n'}
                                        Input: Message shown while execution is paused.
                                    </div>
                                </div>
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.addToProgramEditor?.("TakeControl()")}
                                    >
                                        TakeControl()
                                    </div>
                                    <div className="function-description">
                                        Control the robot by tele-operating it.{'\n'}
                                        Input: N/A
                                    </div>
                                </div>
                                <div className="function-group">
                                    <div 
                                        className="library-function-item"
                                        onClick={() => props.sharedState.addToProgramEditor?.("GiveControl()")}
                                    >
                                        GiveControl()
                                    </div>
                                    <div className="function-description">
                                        Allow the robot to assume operation.{'\n'}
                                        Input: N/A
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                    
                    {/* Saved Positions Section */}
                    <div className="library-section">
                        <h3 className="library-section-title">Saved Positions</h3>
                        <div className="library-subsection">
                            <div className="library-text">
                                {savedPositions.map((position, index) => (
                                    <div 
                                        key={index}
                                        className="library-function-item"
                                        onClick={() => props.sharedState.insertTextAtCursor?.(position.name)}
                                    >
                                        {position.name}
                                    </div>
                                ))}
                            </div>
                            <button 
                                className="add-position-btn"
                                onClick={() => setShowModal(true)}
                            >
                                + Add Position
                            </button>
                        </div>
                    </div>
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
                            Add New Position
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
                                    Position Name:
                                </label>
                                <input
                                    type="text"
                                    value={newPositionName}
                                    onChange={(e) => setNewPositionName(e.target.value)}
                                    placeholder="e.g., pickup_pose"
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
                                    Joint States:
                                </label>
                                <input
                                    type="text"
                                    value={newJointStates}
                                    onChange={(e) => setNewJointStates(e.target.value)}
                                    placeholder="e.g., [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
                                    style={{
                                        width: "100%",
                                        padding: "8px 12px",
                                        border: "1px solid #ccc",
                                        borderRadius: 4,
                                        fontSize: "0.9em"
                                    }}
                                />
                            </div>
                        </div>
                        <div style={{ display: "flex", justifyContent: "center", gap: 16 }}>
                            <button
                                style={{
                                    background: "#6c757d",
                                    color: "white",
                                    border: "none",
                                    borderRadius: 4,
                                    padding: "8px 20px",
                                    fontWeight: "bold",
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
                                Save Position
                            </button>
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
}; 