import React from "react";
import {
    CustomizableComponentProps,
    isSelected,
} from "./CustomizableComponent";
import { className } from "shared/util";
import "operator/css/Library.css";

/**
 * A library component that displays functions and saved positions
 * 
 * @param props {@link CustomizableComponentProps}
 */
export const Library = (props: CustomizableComponentProps) => {
    const { customizing } = props.sharedState;
    const selected = isSelected(props);

    /** Callback when component is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<HTMLDivElement>) => {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    };

    // In customizing state add onClick callback
    const selectProp = customizing ? { onClick: onSelect } : {};

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
                            <div>MoveEEToPose(x,y,z)</div>
                            <div>AdjustGripperWidth()</div>
                            <div>RotateEE(theta)</div>
                            <div>ResetRobot()</div>
                        </div>
                    </div>
                    
                    <div className="library-subsection">
                        <h4 className="library-subsection-title human-heading">Human</h4>
                        <div className="library-text">
                            <div>PauseAndConfirm()</div>
                            <div>GiveControl()</div>
                            <div>TakeControl()</div>
                        </div>
                    </div>
                </div>
                
                {/* Saved Positions Section */}
                <div className="library-section">
                    <h3 className="library-section-title">Saved Positions</h3>
                    <div className="library-text">
                            <div>approach_pose</div>
                            <div>red_cub_pose</div>
                            <div>end_pose</div>
                    </div>
                </div>
            </div>
        </div>
    );
}; 