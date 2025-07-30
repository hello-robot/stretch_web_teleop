import React, { useState } from "react";
import {
    CustomizableComponentProps,
    isSelected,
} from "./CustomizableComponent";
import { className } from "shared/util";
import "operator/css/Library.css";

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
    
    // State for saved positions and modal- temporary placeholder
    const [savedPositions, setSavedPositions] = useState<SavedPosition[]>([
        { name: "stowGripper", jointStates: "[0.0, -0.497, 3.19579]", timestamp: new Date() },
        { name: "centerWrist", jointStates: "[0.0, 0.0, 0.0]", timestamp: new Date() },
        { name: "testPose", jointStates: "[-1.61e-08, -2.94e-07, -2.80e-07, -2.06e-07, -2.05e-06, 2.93e-07, 1.38e-06, 1.44e-07, 9.33e-07]", timestamp: new Date() }
    ]);
    const [showModal, setShowModal] = useState(false);
    const [newPositionName, setNewPositionName] = useState("");
    const [newJointStates, setNewJointStates] = useState("");

    /** Callback when component is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<HTMLDivElement>) => {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    };

    const selectProp = customizing ? { onClick: onSelect } : {};

    // Handle adding new position
    const handleAddPosition = () => {
        if (newPositionName.trim() && newJointStates.trim()) {
            const newPosition: SavedPosition = {
                name: newPositionName.trim(),
                jointStates: newJointStates.trim(),
                timestamp: new Date()
            };
            setSavedPositions(prev => [...prev, newPosition]);
            
            // Add to program editor's autocomplete and syntax highlighting
            props.sharedState.addSavedPosition?.(newPositionName.trim());
            
            setNewPositionName("");
            setNewJointStates("");
            setShowModal(false);
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
                                    Move the robot's end effector to a specific pose
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
                                    Adjust the width of the gripper
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
                                    Rotate the end effector around its axis
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
                                    Reset the robot to its home position
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
                                    Pause execution and wait for human confirmation
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
                                    Give manual control to the human operator
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
                                    Take control back from the human operator
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
                                    placeholder="e.g., [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]"
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