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
    
    // State for saved positions and modal
    const [savedPositions, setSavedPositions] = useState<SavedPosition[]>([
        { name: "approach_pose", jointStates: "[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]", timestamp: new Date() },
        { name: "red_cub_pose", jointStates: "[0.2, 0.3, 0.4, 0.5, 0.6, 0.7]", timestamp: new Date() },
        { name: "end_pose", jointStates: "[0.3, 0.4, 0.5, 0.6, 0.7, 0.8]", timestamp: new Date() }
    ]);
    const [showModal, setShowModal] = useState(false);
    const [newPositionName, setNewPositionName] = useState("");
    const [newJointStates, setNewJointStates] = useState("");

    /** Callback when component is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<HTMLDivElement>) => {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    };

    // In customizing state add onClick callback
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
            setNewPositionName("");
            setNewJointStates("");
            setShowModal(false);
        }
    };

    // Handle cancel
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
                            <div 
                                className="library-function-item"
                                onClick={() => props.sharedState.addToProgramEditor?.("MoveEEToPose(x,y,z)")}
                            >
                                MoveEEToPose(x,y,z)
                            </div>
                            <div 
                                className="library-function-item"
                                onClick={() => props.sharedState.addToProgramEditor?.("AdjustGripperWidth()")}
                            >
                                AdjustGripperWidth()
                            </div>
                            <div 
                                className="library-function-item"
                                onClick={() => props.sharedState.addToProgramEditor?.("RotateEE(theta)")}
                            >
                                RotateEE(theta)
                            </div>
                            <div 
                                className="library-function-item"
                                onClick={() => props.sharedState.addToProgramEditor?.("ResetRobot()")}
                            >
                                ResetRobot()
                            </div>
                        </div>
                    </div>
                    
                    <div className="library-subsection">
                        <h4 className="library-subsection-title human-heading">Human</h4>
                        <div className="library-text">
                            <div 
                                className="library-function-item"
                                onClick={() => props.sharedState.addToProgramEditor?.("PauseAndConfirm()")}
                            >
                                PauseAndConfirm()
                            </div>
                            <div 
                                className="library-function-item"
                                onClick={() => props.sharedState.addToProgramEditor?.("GiveControl()")}
                            >
                                GiveControl()
                            </div>
                            <div 
                                className="library-function-item"
                                onClick={() => props.sharedState.addToProgramEditor?.("TakeControl()")}
                            >
                                TakeControl()
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
                                    onClick={() => props.sharedState.addToProgramEditor?.(position.name)}
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
                <div className="modal-overlay" onClick={handleCancel}>
                    <div className="modal-content" onClick={(e) => e.stopPropagation()}>
                        <h3 className="modal-title">Add New Position</h3>
                        <div className="modal-form">
                            <div className="form-group">
                                <label htmlFor="position-name">Position Name:</label>
                                <input
                                    id="position-name"
                                    type="text"
                                    value={newPositionName}
                                    onChange={(e) => setNewPositionName(e.target.value)}
                                    placeholder="e.g., pickup_pose"
                                    className="form-input"
                                />
                            </div>
                            <div className="form-group">
                                <label htmlFor="joint-states">Joint States:</label>
                                <input
                                    id="joint-states"
                                    type="text"
                                    value={newJointStates}
                                    onChange={(e) => setNewJointStates(e.target.value)}
                                    placeholder="e.g., [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]"
                                    className="form-input"
                                />
                            </div>
                            <div className="modal-buttons">
                                <button 
                                    className="btn btn-primary"
                                    onClick={handleAddPosition}
                                    disabled={!newPositionName.trim() || !newJointStates.trim()}
                                >
                                    Save Position
                                </button>
                                <button 
                                    className="btn btn-secondary"
                                    onClick={handleCancel}
                                >
                                    Cancel
                                </button>
                            </div>
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
}; 