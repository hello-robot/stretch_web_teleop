import React, { useState, useEffect } from "react";
import {
    CustomizableComponentProps,
    SharedState,
    isSelected,
} from "./CustomizableComponent";
import { className, RobotPose } from "shared/util";
import CloseIcon from "@mui/icons-material/Close";
import CheckIcon from "@mui/icons-material/Check";
import ErrorIcon from "@mui/icons-material/Error";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import "operator/css/ExecutionMonitor.css";

/** Properties for {@link ExecutionMonitor} */
type ExecutionMonitorProps = CustomizableComponentProps & {
    /* Programming language for syntax highlighting */
    language?: string;
};

// Robot functions 
const ROBOT_FUNCTIONS = [
    'Move_Arm_to_Config',
    'Adjust_Gripper_Width', 
    'Rotate_Wrist_to_Config',
    'Reset_Robot'
];

// Human functions 
const HUMAN_FUNCTIONS = [
    'Pause_And_Confirm',
    'Take_Control'
];

// Default saved configurations 
const DEFAULT_SAVED_POSITIONS = [
    'stowGripper',
    'centerWrist'
];

// Define default saved positions
const POSE_DEFINITIONS = {
    stowGripper: {
        joint_wrist_roll: 0.0,
        joint_wrist_pitch: -0.497,
        joint_wrist_yaw: 3.19579,
    },
    centerWrist: {
        joint_wrist_roll: 0.0,
        joint_wrist_pitch: 0.0,
        joint_wrist_yaw: 0.0,
    }
};

// Define home pose
const HOME_POSE: RobotPose = {
    joint_lift: 0.6000133947603754,
    wrist_extension: 0.10000127,
    joint_head_pan: -1.734968752451231,
    joint_wrist_roll: -0.0015339807878856412,
    joint_gripper_finger_left: 0.0003447935031709283,
    joint_wrist_pitch: -0.6258641614573416,
    joint_wrist_yaw: 0.002556634646760688,
};

// Program data structure for parsing 
interface ProgramLine {
    lineNumber: number;
    content: string;
    command?: string;  
    parameters?: any;   
    isExecutable: boolean; // handling for empty lines or if invalid 
    error?: {
        type: 'syntax' | 'invalid_input' | 'unknown_pose';
        message: string;
    };
}

interface Program {
    lines: ProgramLine[];
    totalLines: number;
}

/**
 * Parse program code into structured format for execution
 */
const parseProgram = (code: string): Program => {
    const lines = code.split('\n');
    const programLines: ProgramLine[] = [];
    
    lines.forEach((line, index) => {
        const lineNumber = index + 1;
        const trimmedLine = line.trim();
        
        if (trimmedLine === '') {
            // if there is an empty line
            programLines.push({
                lineNumber,
                content: line,
                isExecutable: false
            });
        } else {
            // Check for different command types
            const moveArmMatch = trimmedLine.match(/Move_Arm_to_Config\s*\(\s*([^)]*)\s*\)/);
            const resetRobotMatch = trimmedLine.match(/Reset_Robot\s*\(\s*\)/);
            const resetRobotWithParamsMatch = trimmedLine.match(/Reset_Robot\s*\(\s*[^)]+\s*\)/);
            const adjustGripperMatch = trimmedLine.match(/Adjust_Gripper_Width\s*\(\s*([^)]*)\s*\)/);
            const rotateWristMatch = trimmedLine.match(/Rotate_Wrist_to_Config\s*\(\s*([^)]*)\s*\)/);
            const takeControlMatch = trimmedLine.match(/Take_Control\s*\(\s*\)/);
            const takeControlWithParamsMatch = trimmedLine.match(/Take_Control\s*\(\s*[^)]+\s*\)/);
            const pauseAndConfirmMatch = trimmedLine.match(/Pause_And_Confirm\s*\(\s*([^)]*)\s*\)/);
            
            if (moveArmMatch) {
                const parameter = moveArmMatch[1] || null;
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "Move_Arm_to_Config",
                    parameters: parameter,
                    isExecutable: true
                });
            } else if (resetRobotWithParamsMatch) {
                // ResetRobot with parameters - invalid input
                programLines.push({
                    lineNumber,
                    content: line,
                    isExecutable: false,
                    error: {
                        type: 'invalid_input',
                        message: `Line ${lineNumber}: Invalid input`
                    }
                });
            } else if (resetRobotMatch) {
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "Reset_Robot",
                    parameters: null,
                    isExecutable: true
                });
            } else if (adjustGripperMatch) {
                const parameter = adjustGripperMatch[1] || null;
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "Adjust_Gripper_Width",
                    parameters: parameter,
                    isExecutable: true
                });
            } else if (rotateWristMatch) {
                const parameter = rotateWristMatch[1] || null;
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "Rotate_Wrist_to_Config",
                    parameters: parameter,
                    isExecutable: true
                });
            } else if (takeControlWithParamsMatch) {
                // TakeControl with parameters - invalid input
                programLines.push({
                    lineNumber,
                    content: line,
                    isExecutable: false,
                    error: {
                        type: 'invalid_input',
                        message: `Line ${lineNumber}: Invalid input`
                    }
                });
            } else if (takeControlMatch) {
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "Take_Control",
                    parameters: null,
                    isExecutable: true
                });
            } else if (pauseAndConfirmMatch) {
                const parameter = pauseAndConfirmMatch[1] || null;
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "Pause_And_Confirm",
                    parameters: parameter,
                    isExecutable: true
                });
            } else {
                // Unknown command or invalid line - syntax error
                programLines.push({
                    lineNumber,
                    content: line,
                    isExecutable: false,
                    error: {
                        type: 'syntax',
                        message: `Line ${lineNumber}: Syntax error`
                    }
                });
            }
        }
    });
    
    return {
        lines: programLines,
        totalLines: lines.length
    };
};



/**
 * A read-only display component that shows the program from the ProgramEditor
 * with syntax highlighting and line numbers
 *
 * @param props {@link ExecutionMonitorProps}
 */
export const ExecutionMonitor = (props: ExecutionMonitorProps) => {
    // State management
    const [code, setCode] = useState<string>("");
    const [lineNumbers, setLineNumbers] = useState<string[]>([]);
    const [savedPositions, setSavedPositions] = useState<string[]>(DEFAULT_SAVED_POSITIONS);
    const [showDoneMessage, setShowDoneMessage] = useState(false);
    const prevIsExecutingRef = React.useRef(false);
    const stopExecutionRef = React.useRef<boolean>(false);
    
    const { customizing, currentExecutingLine, isExecutingProgram, waitingForUserConfirmation, handleDoneTeleoperating, executionError, clearExecutionError, errorLineNumber } = props.sharedState;
    const selected = isSelected(props);

    // Create dynamic array that updates when savedPositions changes
    const allFunctions = React.useMemo(() => {
        return [...ROBOT_FUNCTIONS, ...HUMAN_FUNCTIONS, ...savedPositions];
    }, [savedPositions]);

    // Load code from session storage
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

    // Message when program finishes executing
    useEffect(() => {
        if (prevIsExecutingRef.current && !isExecutingProgram && !executionError) {
            setShowDoneMessage(true);
            const timer = setTimeout(() => {
                setShowDoneMessage(false);
            }, 5000); 
            
            return () => clearTimeout(timer);
        } else if (isExecutingProgram) {
            // Program is executing, hide done message
            setShowDoneMessage(false);
        }
        prevIsExecutingRef.current = isExecutingProgram;
    }, [isExecutingProgram, executionError]);

    // Syntax highlighting function 
    const highlightSyntax = (text: string): string => {
        let highlightedText = text;
        
        // no highlighting in Pause_And_Confirm parameters
        const pauseAndConfirmParams: string[] = [];
        let paramIndex = 0;
        highlightedText = highlightedText.replace(/Pause_And_Confirm\s*\(\s*([^)]*)\s*\)/g, (match, content) => {
            const placeholder = `__PAUSE_CONFIRM_PARAM_${paramIndex}__`;
            pauseAndConfirmParams[paramIndex] = content;
            paramIndex++;
            return `Pause_And_Confirm(${placeholder})`;
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

    // Function to read the program code 
    const readProgramCode = (): string => {
        return code;
    };

    // Function to handle Run/Stop Program button click
    const handleRunProgram = async () => {
        console.log("ExecutionMonitor: handleRunProgram called, current isExecutingProgram:", isExecutingProgram);
        
        if (isExecutingProgram) {
            // Stop execution
            console.log("ExecutionMonitor: Stop Program button clicked!");
            
            stopExecutionRef.current = true;
            
            // Set execution state to false
            const buttonFunctionProvider = (window as any).buttonFunctionProvider;
            if (buttonFunctionProvider) {
                buttonFunctionProvider.setExecutionState(false);
            }
            
            // Reset current executing line
            if (props.sharedState.updateCurrentExecutingLine) {
                props.sharedState.updateCurrentExecutingLine(undefined);
            }
            
        } else {
            // Start execution
            console.log("ExecutionMonitor: Run Program button clicked!");
            
            // Set execution state to true using the proper method
            const buttonFunctionProvider = (window as any).buttonFunctionProvider;
            if (buttonFunctionProvider) {
                buttonFunctionProvider.setExecutionState(true);
            }
            
            // Reset current executing line at start
            if (props.sharedState.updateCurrentExecutingLine) {
                props.sharedState.updateCurrentExecutingLine(undefined);
            }
            
            const programText = readProgramCode();
            console.log("ExecutionMonitor: Program text:", programText);
            
            // Parse the program 
            const program = parseProgram(programText);
            console.log("ExecutionMonitor: Parsed program:", program);
            
            // Log lines for debugging
            program.lines.forEach(line => {
                if (line.isExecutable) {
                    console.log(`ExecutionMonitor: Line ${line.lineNumber}: Executable command "${line.command}"`);
                } else {
                    console.log(`ExecutionMonitor: Line ${line.lineNumber}: Non-executable (${line.content})`);
                }
            });
            
            // Execute the program line by line
            executeProgram(program);
        }
    };

    // Function to handle button click (either run or stop)
    const handleButtonClick = () => {
        if (isExecutingProgram) {
            console.log("ExecutionMonitor: Stop button clicked");
            handleStopProgram();
        } else {
            console.log("ExecutionMonitor: Run button clicked");
            handleRunProgram();
        }
    };

    // Execute the parsed program line by line
    const executeProgram = async (program: Program) => {
        console.log("ExecutionMonitor: Starting program execution...");
        
        // Set execution state to true at the start
        const buttonFunctionProvider = (window as any).buttonFunctionProvider;
        if (buttonFunctionProvider) {
            buttonFunctionProvider.setExecutionState(true);
        }
        // Reset stop execution flag at the start
        stopExecutionRef.current = false;
        // Function to check if execution should continue
        const shouldContinue = () => {
            return !stopExecutionRef.current; 
        };
        
        try {
            console.log("ExecutionMonitor: Starting program loop, stopExecutionRef.current:", stopExecutionRef.current);
            for (const line of program.lines) {
                console.log("ExecutionMonitor: Checking line", line.lineNumber, "stopExecutionRef.current:", stopExecutionRef.current);
                
                // Update current executing line
                if (props.sharedState.updateCurrentExecutingLine) {
                    props.sharedState.updateCurrentExecutingLine(line.lineNumber);
                }
                
                if (!shouldContinue()) {
                    console.log("ExecutionMonitor: Program execution stopped by user");
                    break;
                }
                
                // Check for errors in the line
                if (line.error) {
                    console.error(`ExecutionMonitor: Error on line ${line.lineNumber}: ${line.error.message}`);
                    // Stop execution and display error
                    if (props.sharedState.updateCurrentExecutingLine) {
                        props.sharedState.updateCurrentExecutingLine(line.lineNumber);
                    }
                    if (props.sharedState.setExecutionError) {
                        props.sharedState.setExecutionError(line.error);
                    }
                    // Set error line number for highlighting
                    if (props.sharedState.setErrorLineNumber) {
                        props.sharedState.setErrorLineNumber(line.lineNumber);
                    }
                    break;
                }
                
                if (line.isExecutable) {
                    console.log(`ExecutionMonitor: Executing line ${line.lineNumber}: ${line.command} with parameter: ${line.parameters}`);
                    
                    // Executes the command based on what function it is
                    if (line.command === "Move_Arm_to_Config") {
                        const poseName = line.parameters;
                        const pose = POSE_DEFINITIONS[poseName as keyof typeof POSE_DEFINITIONS];
                        
                        if (pose) {
                            // Filter pose to only include joints for MoveEEToPose
                            const filteredPose: RobotPose = {};
                            if ('wrist_extension' in pose && pose.wrist_extension !== undefined) filteredPose.wrist_extension = pose.wrist_extension as number;
                            if ('joint_lift' in pose && pose.joint_lift !== undefined) filteredPose.joint_lift = pose.joint_lift as number;
                            if ('joint_head_pan' in pose && pose.joint_head_pan !== undefined) filteredPose.joint_head_pan = pose.joint_head_pan as number;
                            if ('joint_head_tilt' in pose && pose.joint_head_tilt !== undefined) filteredPose.joint_head_tilt = pose.joint_head_tilt as number;
                            
                            console.log(`ExecutionMonitor: Sending MoveEEToPose command with pose: ${poseName}`, filteredPose);
                            // Send command to robot
                            if ((window as any).remoteRobot) {
                                (window as any).remoteRobot.setRobotPose(filteredPose);
                                console.log(`ExecutionMonitor: Command sent to robot!`);
                                console.log(`ExecutionMonitor: Waiting...`);
                                await new Promise(resolve => setTimeout(resolve, 5000));
                                console.log(`ExecutionMonitor: Executing next command...`);
                            } else {
                                console.error("ExecutionMonitor: RemoteRobot not available");
                            }
                        } else {
                            console.error(`ExecutionMonitor: Unknown pose: ${poseName}. Available poses: ${Object.keys(POSE_DEFINITIONS).join(', ')}`);
                            // Stop execution and display error
                            if (props.sharedState.setExecutionError) {
                                props.sharedState.setExecutionError({
                                    type: 'unknown_pose',
                                    message: `Line ${line.lineNumber}: Unknown pose: ${poseName}`
                                });
                            }
                            break;
                        }
                    }
                    else if (line.command === "Adjust_Gripper_Width") {
                        const poseName = line.parameters;
                        const pose = POSE_DEFINITIONS[poseName as keyof typeof POSE_DEFINITIONS];
                        
                        if (pose) {
                            // Filter pose to only include joints for AdjustGripperWidth
                            const filteredPose: RobotPose = {};
                            if ('joint_gripper_finger_left' in pose && pose.joint_gripper_finger_left !== undefined) filteredPose.joint_gripper_finger_left = pose.joint_gripper_finger_left as number;
                            console.log(`ExecutionMonitor: Sending AdjustGripperWidth command with pose: ${poseName}`, filteredPose);
                            // Send command to robot
                            if ((window as any).remoteRobot) {
                                (window as any).remoteRobot.setRobotPose(filteredPose);
                                console.log(`ExecutionMonitor: Command sent to robot!`);
                                console.log(`ExecutionMonitor: Waiting...`);
                                await new Promise(resolve => setTimeout(resolve, 5000));
                                console.log(`ExecutionMonitor: Executing next command...`);
                            } else {
                                console.error("ExecutionMonitor: RemoteRobot not available");
                            }
                        } else {
                            console.error(`ExecutionMonitor: Unknown pose: ${poseName}. Available poses: ${Object.keys(POSE_DEFINITIONS).join(', ')}`);
                            // Stop execution and display error
                            if (props.sharedState.setExecutionError) {
                                props.sharedState.setExecutionError({
                                    type: 'unknown_pose',
                                    message: `Line ${line.lineNumber}: Unknown pose: ${poseName}`
                                });
                            }
                            break;
                        }
                    }
                    else if (line.command === "Rotate_Wrist_to_Config") {
                        const poseName = line.parameters;
                        const pose = POSE_DEFINITIONS[poseName as keyof typeof POSE_DEFINITIONS];
                        
                        if (pose) {
                            // Filter pose to only include joints for RotateEE
                            const filteredPose: RobotPose = {};
                            if ('joint_wrist_roll' in pose && pose.joint_wrist_roll !== undefined) filteredPose.joint_wrist_roll = pose.joint_wrist_roll as number;
                            if ('joint_wrist_pitch' in pose && pose.joint_wrist_pitch !== undefined) filteredPose.joint_wrist_pitch = pose.joint_wrist_pitch as number;
                            if ('joint_wrist_yaw' in pose && pose.joint_wrist_yaw !== undefined) filteredPose.joint_wrist_yaw = pose.joint_wrist_yaw as number;
                            console.log(`ExecutionMonitor: Sending RotateEE command with pose: ${poseName}`, filteredPose);
                            // Send command to robot
                            if ((window as any).remoteRobot) {
                                (window as any).remoteRobot.setRobotPose(filteredPose);
                                console.log(`ExecutionMonitor: Command sent to robot!`);
                                console.log(`ExecutionMonitor: Waiting...`);
                                await new Promise(resolve => setTimeout(resolve, 5000));
                                console.log(`ExecutionMonitor: Executing next command...`);
                            } else {
                                console.error("ExecutionMonitor: RemoteRobot not available");
                            }
                        } else {
                            console.error(`ExecutionMonitor: Unknown pose: ${poseName}. Available poses: ${Object.keys(POSE_DEFINITIONS).join(', ')}`);
                            // Stop execution and display error
                            if (props.sharedState.setExecutionError) {
                                props.sharedState.setExecutionError({
                                    type: 'unknown_pose',
                                    message: `Line ${line.lineNumber}: Unknown pose: ${poseName}`
                                });
                            }
                            break;
                        }
                    }
                    else if (line.command === "Reset_Robot") {
                        console.log(`Sending ResetRobot command (setting robot to home pose)`);
                        // Send setRobotPose command to robot with stow pose
                        if ((window as any).remoteRobot) {
                            const retractedPose = { wrist_extension: 0.00211174 };
                            (window as any).remoteRobot.setRobotPose(retractedPose);
                            console.log(`Arm retraction command sent to robot!`);
                            console.log(`Waiting for arm retraction...`);
                            await new Promise(resolve => setTimeout(resolve, 2000));
                            (window as any).remoteRobot.setRobotPose(HOME_POSE);
                            console.log(`Command sent to robot!`);
                            console.log(`Waiting...`);
                            await new Promise(resolve => setTimeout(resolve, 5000));
                            console.log(`Executing next command...`);
                        } else {
                            console.error("RemoteRobot not available");
                        }
                    }
                    else if (line.command === "Pause_And_Confirm") {
                        const message = line.parameters || "Ready to continue? Please confirm before the robot proceeds or reset to revise.";
                        console.log(`ExecutionMonitor: Pausing program execution for user confirmation: ${message}`);
                        await new Promise<void>((resolve) => {
                            (window as any).pauseAndConfirmResolve = resolve;
                            (window as any).pauseAndConfirmMessage = message;
                        });
                        console.log(`ExecutionMonitor: Resuming program execution after confirmation`);
                    }
                    else if (line.command === "Take_Control") {
                        console.log(`ExecutionMonitor: Taking control from robot`);
                        if (buttonFunctionProvider) {
                            buttonFunctionProvider.setExecutionState(false);
                        }
                        console.log(`ExecutionMonitor: Control returned to user`);
                        await new Promise<void>((resolve) => {
                            (window as any).resumeProgramExecution = resolve;
                        });
                        console.log(`ExecutionMonitor: Resuming program execution`);
                    }
                } else {
                    console.log(`ExecutionMonitor: Skipping line ${line.lineNumber}: ${line.content}`);
                }
            }
            
            console.log("ExecutionMonitor: Program execution complete!");
        } catch (error) {
            console.error("ExecutionMonitor: Error during program execution:", error);
        } finally {
            // Always reset execution state and button state when program completes or stops
            stopExecutionRef.current = true;
            if (buttonFunctionProvider) {
                buttonFunctionProvider.setExecutionState(false);
            }
            
            // Reset current executing line
            if (props.sharedState.updateCurrentExecutingLine) {
                props.sharedState.updateCurrentExecutingLine(undefined);
            }
        }
    };

    const handleStopProgram = () => {
        console.log("ExecutionMonitor: handleStopProgram called");
        stopExecutionRef.current = true;
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
        const hasError = errorLineNumber === lineNumber;
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
                <div className="execution-monitor-header-right">
                    {showDoneMessage && (
                        <div style={{
                            color: "#28a745",
                            fontWeight: "bold",
                            fontSize: "14px",
                            display: "flex",
                            alignItems: "center",
                            marginRight: "8px"
                        }}>
                            Done Executing!
                        </div>
                    )}
                    <button 
                        style={{
                            background: "#ff8c00",
                            color: "white",
                            border: "none",
                            borderRadius: 4,
                            padding: "6px 16px",
                            fontSize: "16px",
                            fontWeight: "700",
                            cursor: "pointer",
                            transition: "background-color 0.2s ease",
                            letterSpacing: "0.5px",
                            display: "flex",
                            alignItems: "center",
                            marginRight: "8px"
                        }}
                                                        onClick={async () => {
                                    if ((window as any).remoteRobot) {
                                        const retractedPose = { wrist_extension: 0.00211174 };
                                        (window as any).remoteRobot.setRobotPose(retractedPose);
                                        console.log(`Arm retraction command sent to robot!`);
                                        console.log(`Waiting for arm retraction...`);
                                        await new Promise(resolve => setTimeout(resolve, 2000));
                                        (window as any).remoteRobot.setRobotPose(HOME_POSE);
                                        console.log(`Home pose command sent to robot!`);
                                    }
                                }}
                        title="Reset robot to home position"
                    >
                        Reset Robot
                    </button>
                    <button 
                        className="run-program-button"
                        onClick={handleButtonClick}
                        type="button"
                        style={{
                            backgroundColor: isExecutingProgram ? "#dc3545" : undefined
                        }}
                    >
                        {isExecutingProgram ? (
                            <>
                                <CloseIcon style={{ marginRight: "4px" }} />
                                Stop
                            </>
                        ) : (
                            <>
                                <PlayArrowIcon style={{ marginRight: "4px" }} />
                                Run
                            </>
                        )}
                    </button>
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
                        const hasError = errorLineNumber === lineNumber;
                        
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