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
                        <h4 className="library-subsection-title">Robot</h4>
                        <p className="library-text">
                            Robot functions and capabilities will be displayed here. 
                            This section will contain various robot operations and commands 
                            that can be executed during program execution.
                        </p>
                    </div>
                    
                    <div className="library-subsection">
                        <h4 className="library-subsection-title">Human</h4>
                        <p className="library-text">
                            Human interaction functions and controls will be displayed here. 
                            This section will contain user interface elements and human 
                            input handling for the robot system.
                        </p>
                    </div>
                </div>
                
                {/* Saved Positions Section */}
                <div className="library-section">
                    <h3 className="library-section-title">Saved Positions</h3>
                    <p className="library-text">
                        Saved robot positions and poses will be displayed here. 
                        This section will show a list of stored positions that 
                        can be recalled and used in programs or demonstrations.
                    </p>
                </div>
            </div>
        </div>
    );
}; 