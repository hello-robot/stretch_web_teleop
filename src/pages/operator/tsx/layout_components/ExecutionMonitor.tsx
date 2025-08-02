import React, { useState, useEffect } from "react";
import {
    CustomizableComponentProps,
    SharedState,
    isSelected,
} from "./CustomizableComponent";
import { className } from "shared/util";
import CloseIcon from "@mui/icons-material/Close";
import CheckIcon from "@mui/icons-material/Check";
import ErrorIcon from "@mui/icons-material/Error";
import "operator/css/ExecutionMonitor.css";

/** Properties for {@link ExecutionMonitor} */
type ExecutionMonitorProps = CustomizableComponentProps & {
    /* Programming language for syntax highlighting */
    language?: string;
};

// Robot functions (orange)
const ROBOT_FUNCTIONS = [
    'MoveEEToPose',
    'AdjustGripperWidth', 
    'RotateEE',
    'ResetRobot'
];

// Human functions (green)
const HUMAN_FUNCTIONS = [
    'PauseAndConfirm',
    'TakeControl'
];

// Saved positions (blue) - will be made dynamic
const DEFAULT_SAVED_POSITIONS = [
    'stowGripper',
    'centerWrist'
];

/**
 * A read-only display component that shows the program from the ProgramEditor
 * with syntax highlighting and line numbers
 *
 * @param props {@link ExecutionMonitorProps}
 */
export const ExecutionMonitor = (props: ExecutionMonitorProps) => {
    const [code, setCode] = useState<string>("");
    const [lineNumbers, setLineNumbers] = useState<string[]>([]);
    const [savedPositions, setSavedPositions] = useState<string[]>(DEFAULT_SAVED_POSITIONS);
    const { customizing, currentExecutingLine, isExecutingProgram, waitingForUserConfirmation, handleDoneTeleoperating, executionError, clearExecutionError } = props.sharedState;
    const selected = isSelected(props);

    // Create dynamic ALL_FUNCTIONS array that updates when savedPositions changes
    const allFunctions = React.useMemo(() => {
        return [...ROBOT_FUNCTIONS, ...HUMAN_FUNCTIONS, ...savedPositions];
    }, [savedPositions]);

    // Load code from session storage (same as ProgramEditor)
    useEffect(() => {
        const sessionCode = sessionStorage.getItem('programEditorCode');
        if (sessionCode) {
            setCode(sessionCode);
        }
    }, []);

    // Load saved positions from session storage
    useEffect(() => {
        const sessionPositions = sessionStorage.getItem('programEditorSavedPositions');
        if (sessionPositions) {
            try {
                const parsed = JSON.parse(sessionPositions);
                setSavedPositions(parsed);
            } catch (error) {
                console.error("Error parsing saved positions:", error);
            }
        }
    }, []);

    // Update line numbers when code changes
    useEffect(() => {
        const lines = code.split('\n');
        const numbers = lines.map((_, index) => (index + 1).toString());
        setLineNumbers(numbers);
    }, [code]);

    // Syntax highlighting function (same as ProgramEditor)
    const highlightSyntax = (text: string): string => {
        let highlightedText = text;
        
        // no highlighting in PauseAndConfirm parameters
        const pauseAndConfirmParams: string[] = [];
        let paramIndex = 0;
        highlightedText = highlightedText.replace(/PauseAndConfirm\s*\(\s*([^)]*)\s*\)/g, (match, content) => {
            const placeholder = `__PAUSE_CONFIRM_PARAM_${paramIndex}__`;
            pauseAndConfirmParams[paramIndex] = content;
            paramIndex++;
            return `PauseAndConfirm(${placeholder})`;
        });
        
        // Highlight robot functions in orange
        ROBOT_FUNCTIONS.forEach(func => {
            const regex = new RegExp(`\\b${func}\\b`, 'g');
            highlightedText = highlightedText.replace(regex, `<span class="robot-function">${func}</span>`);
        });
        
        // Highlight human functions in green
        HUMAN_FUNCTIONS.forEach(func => {
            const regex = new RegExp(`\\b${func}\\b`, 'g');
            highlightedText = highlightedText.replace(regex, `<span class="human-function">${func}</span>`);
        });
        
        // Highlight saved positions in blue
        savedPositions.forEach(position => {
            const regex = new RegExp(`\\b${position}\\b`, 'g');
            highlightedText = highlightedText.replace(regex, `<span class="saved-position">${position}</span>`);
        });
        
        // Restore PauseAndConfirm parameters without highlighting
        pauseAndConfirmParams.forEach((param, index) => {
            const placeholder = `__PAUSE_CONFIRM_PARAM_${index}__`;
            highlightedText = highlightedText.replace(placeholder, param);
        });
        
        return highlightedText;
    };

    /** Callback when component is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<HTMLDivElement>) => {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    };

    const handleStopProgram = () => {
        // Stop the program execution 
        if ((window as any).stopExecutionRef) {
            (window as any).stopExecutionRef.current = true;
        }
        
        // Set execution state to false
        const buttonFunctionProvider = (window as any).buttonFunctionProvider;
        if (buttonFunctionProvider) {
            buttonFunctionProvider.setExecutionState(false);
        }
        
        // Reset current executing line
        if (props.sharedState.updateCurrentExecutingLine) {
            props.sharedState.updateCurrentExecutingLine(undefined);
        }
    };

    // In customizing state add onClick callback
    const selectProp = customizing ? { onClick: onSelect } : {};

    const highlightedCode = highlightSyntax(code);
    
    // Split code into lines for highlighting the current executing line
    const codeLines = code.split('\n');
    const highlightedLines = codeLines.map((line, index) => {
        const lineNumber = index + 1;
        const isExecuting = currentExecutingLine === lineNumber;
        const hasError = executionError && currentExecutingLine === lineNumber;
        const highlightedLine = highlightSyntax(line);
        
        return (
            <div 
                key={index}
                className={className("code-line", {
                    executing: isExecuting && !hasError,
                    error: hasError
                })}
                dangerouslySetInnerHTML={{ __html: highlightedLine || '&nbsp;' }}
            />
        );
    });

    return (
        <div
            className={className("execution-monitor-root", {
                customizing,
                selected,
            })}
            {...selectProp}
        >
            <div className="execution-monitor-header">
                <div className="execution-monitor-header-left">
                    {props.language && (
                        <span className="execution-monitor-language">{props.language}</span>
                    )}
                </div>
                {isExecutingProgram && !waitingForUserConfirmation && (
                    <button 
                        className="execution-monitor-stop-button"
                        onClick={handleStopProgram}
                        type="button"
                    >
                        <CloseIcon style={{ marginRight: "4px" }} />
                        Stop
                    </button>
                )}
                {waitingForUserConfirmation && handleDoneTeleoperating && (
                    <button 
                        className="execution-monitor-done-button"
                        onClick={handleDoneTeleoperating}
                        type="button"
                    >
                        <CheckIcon style={{ marginRight: "4px" }} />
                        Done teleoperating
                    </button>
                )}
            </div>
            {executionError && (
                <div className="execution-monitor-error-banner">
                                    <div style={{ display: "flex", alignItems: "center", gap: "8px" }}>
                    <ErrorIcon style={{ fontSize: "16px" }} />
                    {executionError.message}
                </div>
                    <button 
                        className="execution-monitor-error-close"
                        onClick={clearExecutionError}
                        type="button"
                    >
                        ×
                    </button>
                </div>
            )}
            <div className="execution-monitor-container">
                <div className="line-numbers">
                    {lineNumbers.map((number, index) => {
                        const lineNumber = index + 1;
                        const isExecuting = currentExecutingLine === lineNumber;
                        const hasError = executionError && currentExecutingLine === lineNumber;
                        
                        return (
                            <div 
                                key={index} 
                                className={className("line-number", {
                                    executing: isExecuting && !hasError,
                                    error: hasError
                                })}
                            >
                                {number}
                            </div>
                        );
                    })}
                </div>
                <div className="monitor-wrapper">
                    <div className="code-display">
                        {highlightedLines}
                    </div>
                </div>
            </div>
        </div>
    );
}; 