import React, { useState, useRef, useEffect } from "react";
import {
    CustomizableComponentProps,
    SharedState,
    isSelected,
} from "./CustomizableComponent";
import { className, RobotPose } from "shared/util";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import CloseIcon from "@mui/icons-material/Close";
import ErrorIcon from "@mui/icons-material/Error";
import "operator/css/ProgramEditor.css";

/** Properties for {@link ProgramEditor} */
type ProgramEditorProps = CustomizableComponentProps & {
    /* Initial code content */
    initialCode?: string;
    /* Programming language for syntax highlighting */
    language?: string;
    /* Whether the editor is read-only */
    readOnly?: boolean;
    /* Callback function when Run Program button is clicked */
    onRunProgram?: (code: string) => void;
};

// Robot functions 
const ROBOT_FUNCTIONS = [
    'MoveEEToPose',
    'AdjustGripperWidth', 
    'RotateEE',
    'ResetRobot'
];

// Human functions 
const HUMAN_FUNCTIONS = [
    'PauseAndConfirm',
    'TakeControl'
];

// Default saved positions 
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
            const moveEEMatch = trimmedLine.match(/MoveEEToPose\s*\(\s*([^)]*)\s*\)/);
            const resetRobotMatch = trimmedLine.match(/ResetRobot\s*\(\s*\)/);
            const resetRobotWithParamsMatch = trimmedLine.match(/ResetRobot\s*\(\s*[^)]+\s*\)/);
            const adjustGripperMatch = trimmedLine.match(/AdjustGripperWidth\s*\(\s*([^)]*)\s*\)/);
            const rotateEEMatch = trimmedLine.match(/RotateEE\s*\(\s*([^)]*)\s*\)/);
            const takeControlMatch = trimmedLine.match(/TakeControl\s*\(\s*\)/);
            const takeControlWithParamsMatch = trimmedLine.match(/TakeControl\s*\(\s*[^)]+\s*\)/);
            const pauseAndConfirmMatch = trimmedLine.match(/PauseAndConfirm\s*\(\s*([^)]*)\s*\)/);
            
            if (moveEEMatch) {
                const parameter = moveEEMatch[1] || null;
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "MoveEEToPose",
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
                    command: "ResetRobot",
                    parameters: null,
                    isExecutable: true
                });
            } else if (adjustGripperMatch) {
                const parameter = adjustGripperMatch[1] || null;
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "AdjustGripperWidth",
                    parameters: parameter,
                    isExecutable: true
                });
            } else if (rotateEEMatch) {
                const parameter = rotateEEMatch[1] || null;
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "RotateEE",
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
                    command: "TakeControl",
                    parameters: null,
                    isExecutable: true
                });
            } else if (pauseAndConfirmMatch) {
                const parameter = pauseAndConfirmMatch[1] || null;
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "PauseAndConfirm",
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
 * A code editor component with line numbers and syntax highlighting
 *
 * @param props {@link ProgramEditorProps}
 */
export const ProgramEditor = (props: ProgramEditorProps) => {
    // Initialize code from session storage or props
    const getInitialCode = () => {
        const sessionCode = sessionStorage.getItem('programEditorCode');
        return sessionCode || props.initialCode || "";
    };

    // Load custom poses from session storage
    const getInitialCustomPoses = (): {[key: string]: RobotPose} => {
        const sessionPoses = sessionStorage.getItem('programEditorCustomPoses');
        if (sessionPoses) {
            try {
                const parsed = JSON.parse(sessionPoses);
                return parsed;
            } catch (error) {
                console.error("Error parsing custom poses:", error);
            }
        }
        return {};
    };
    
    // Load saved positions from session storage
    const getInitialSavedPositions = (): string[] => {
        const sessionPositions = sessionStorage.getItem('programEditorSavedPositions');
        if (sessionPositions) {
            try {
                return JSON.parse(sessionPositions);
            } catch (error) {
                console.error("Error parsing saved positions:", error);
            }
        }
        return DEFAULT_SAVED_POSITIONS;
    };
    
    // State management
    const [code, setCode] = useState(getInitialCode());
    const [lineNumbers, setLineNumbers] = useState<string[]>([]);
    const [currentSuggestion, setCurrentSuggestion] = useState<string>("");
    const [showSuggestion, setShowSuggestion] = useState(false);
    const [isExecuting, setIsExecuting] = useState(false);
    
    const [savedPositions, setSavedPositions] = useState<string[]>(getInitialSavedPositions());
    const [customPoses, setCustomPoses] = useState<{[key: string]: RobotPose}>(getInitialCustomPoses());
    
    // Refs
    const textareaRef = useRef<HTMLTextAreaElement>(null);
    const lineNumbersRef = useRef<HTMLDivElement>(null);
    const highlightedRef = useRef<HTMLDivElement>(null);
    const stopExecutionRef = useRef<boolean>(false);
    
    // Make stopExecutionRef accessible globally 
    React.useEffect(() => {
        (window as any).stopExecutionRef = stopExecutionRef;
    }, []);
    
    // Combine default and custom poses
    const ALL_POSE_DEFINITIONS = { ...POSE_DEFINITIONS, ...customPoses };
    const { customizing, executionError, currentExecutingLine, clearExecutionError, errorLineNumber } = props.sharedState;
    const selected = isSelected(props);

    // Create dynamic array that updates when savedPositions changes
    const allFunctions = React.useMemo(() => {
        return [...ROBOT_FUNCTIONS, ...HUMAN_FUNCTIONS, ...savedPositions];
    }, [savedPositions]);

        // Function to wait for goal completion
        const waitForPoseCompletion = (targetPose: RobotPose): Promise<void> => {
            // TODO: implement this by storing the target pose + comparing w/ current joint states
            return new Promise((resolve) => {
                resolve();
            });
        };
    
    // Execute the parsed program line by line
    const executeProgram = async (program: Program) => {
        console.log("Starting program execution...");
        
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
            console.log("Starting program loop, stopExecutionRef.current:", stopExecutionRef.current);
            for (const line of program.lines) {
                console.log("Checking line", line.lineNumber, "stopExecutionRef.current:", stopExecutionRef.current);
                
                // Update current executing line
                if (props.sharedState.updateCurrentExecutingLine) {
                    props.sharedState.updateCurrentExecutingLine(line.lineNumber);
                }
                
                if (!shouldContinue()) {
                    console.log("Program execution stopped by user");
                    break;
                }
                
                // Check for errors in the line
                if (line.error) {
                    console.error(`Error on line ${line.lineNumber}: ${line.error.message}`);
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
                    console.log(`Executing line ${line.lineNumber}: ${line.command} with parameter: ${line.parameters}`);
                    
                    // Executes the command based on what function it is
                    if (line.command === "MoveEEToPose") {
                        const poseName = line.parameters;
                        const pose = ALL_POSE_DEFINITIONS[poseName as keyof typeof ALL_POSE_DEFINITIONS];
                        
                        if (pose) {
                            // Filter pose to only include joints for MoveEEToPose
                            const filteredPose: RobotPose = {};
                            if ('wrist_extension' in pose && pose.wrist_extension !== undefined) filteredPose.wrist_extension = pose.wrist_extension as number;
                            if ('joint_lift' in pose && pose.joint_lift !== undefined) filteredPose.joint_lift = pose.joint_lift as number;
                            if ('joint_head_pan' in pose && pose.joint_head_pan !== undefined) filteredPose.joint_head_pan = pose.joint_head_pan as number;
                            if ('joint_head_tilt' in pose && pose.joint_head_tilt !== undefined) filteredPose.joint_head_tilt = pose.joint_head_tilt as number;
                            
                            console.log(`Sending MoveEEToPose command with pose: ${poseName}`, filteredPose);
                            // Send command to robot
                            if ((window as any).remoteRobot) {
                                (window as any).remoteRobot.setRobotPose(filteredPose);
                                console.log(`Command sent to robot!`);
                                console.log(`Waiting...`);
                                await new Promise(resolve => setTimeout(resolve, 5000));
                                console.log(`Executing next command...`);
                            } else {
                                console.error("RemoteRobot not available");
                            }
                        } else {
                            console.error(`Unknown pose: ${poseName}. Available poses: ${Object.keys(ALL_POSE_DEFINITIONS).join(', ')}`);
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
                    else if (line.command === "AdjustGripperWidth") {
                        const poseName = line.parameters;
                        const pose = ALL_POSE_DEFINITIONS[poseName as keyof typeof ALL_POSE_DEFINITIONS];
                        
                        if (pose) {
                            // Filter pose to only include joints for AdjustGripperWidth
                            const filteredPose: RobotPose = {};
                            if ('joint_gripper_finger_left' in pose && pose.joint_gripper_finger_left !== undefined) filteredPose.joint_gripper_finger_left = pose.joint_gripper_finger_left as number;
                            console.log(`Sending AdjustGripperWidth command with pose: ${poseName}`, filteredPose);
                            // Send command to robot
                            if ((window as any).remoteRobot) {
                                (window as any).remoteRobot.setRobotPose(filteredPose);
                                console.log(`Command sent to robot!`);
                                console.log(`Waiting...`);
                                await new Promise(resolve => setTimeout(resolve, 5000));
                                console.log(`Executing next command...`);
                            } else {
                                console.error("RemoteRobot not available");
                            }
                        } else {
                            console.error(`Unknown pose: ${poseName}. Available poses: ${Object.keys(ALL_POSE_DEFINITIONS).join(', ')}`);
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
                    else if (line.command === "RotateEE") {
                        const poseName = line.parameters;
                        const pose = ALL_POSE_DEFINITIONS[poseName as keyof typeof ALL_POSE_DEFINITIONS];
                        
                        if (pose) {
                            // Filter pose to only include joints for RotateEE
                            const filteredPose: RobotPose = {};
                            if ('joint_wrist_roll' in pose && pose.joint_wrist_roll !== undefined) filteredPose.joint_wrist_roll = pose.joint_wrist_roll as number;
                            if ('joint_wrist_pitch' in pose && pose.joint_wrist_pitch !== undefined) filteredPose.joint_wrist_pitch = pose.joint_wrist_pitch as number;
                            if ('joint_wrist_yaw' in pose && pose.joint_wrist_yaw !== undefined) filteredPose.joint_wrist_yaw = pose.joint_wrist_yaw as number;
                            console.log(`Sending RotateEE command with pose: ${poseName}`, filteredPose);
                            // Send command to robot
                            if ((window as any).remoteRobot) {
                                (window as any).remoteRobot.setRobotPose(filteredPose);
                                console.log(`Command sent to robot!`);
                                console.log(`Waiting...`);
                                await new Promise(resolve => setTimeout(resolve, 5000));
                                console.log(`Executing next command...`);
                            } else {
                                console.error("RemoteRobot not available");
                            }
                        } else {
                            console.error(`Unknown pose: ${poseName}. Available poses: ${Object.keys(ALL_POSE_DEFINITIONS).join(', ')}`);
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
                    else if (line.command === "ResetRobot") {
                        console.log(`Sending ResetRobot command (homing the robot)`);
                        // Send homeTheRobot command to robot
                        if ((window as any).remoteRobot) {
                            (window as any).remoteRobot.homeTheRobot();
                            console.log(`Command sent to robot!`);
                            console.log(`Waiting...`);
                            await new Promise(resolve => setTimeout(resolve, 25000));
                            console.log(`Executing next command...`);
                        } else {
                            console.error("RemoteRobot not available");
                        }
                    }
                    else if (line.command === "PauseAndConfirm") {
                        const message = line.parameters || "Ready to continue? Please confirm before the robot proceeds or reset to revise.";
                        console.log(`Pausing program execution for user confirmation: ${message}`);
                        await new Promise<void>((resolve) => {
                            (window as any).pauseAndConfirmResolve = resolve;
                            (window as any).pauseAndConfirmMessage = message;
                        });
                        console.log(`Resuming program execution after confirmation`);
                    }
                    else if (line.command === "TakeControl") {
                        console.log(`Taking control from robot`);
                        if (buttonFunctionProvider) {
                            buttonFunctionProvider.setExecutionState(false);
                        }
                        console.log(`Control returned to user`);
                        await new Promise<void>((resolve) => {
                            (window as any).resumeProgramExecution = resolve;
                        });
                        console.log(`Resuming program execution`);
                    }
                } else {
                    console.log(`Skipping line ${line.lineNumber}: ${line.content}`);
                }
            }
            
            console.log("Program execution complete!");
        } catch (error) {
            console.error("Error during program execution:", error);
        } finally {
            // Always reset execution state and button state when program completes or stops
            stopExecutionRef.current = true;
            if (buttonFunctionProvider) {
                buttonFunctionProvider.setExecutionState(false);
            }
            setIsExecuting(false);
            
            // Reset current executing line
            if (props.sharedState.updateCurrentExecutingLine) {
                props.sharedState.updateCurrentExecutingLine(undefined);
            }
        }
    };

    // Syntax highlighting function
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



    // Function to insert text at cursor position
    const insertTextAtCursor = (text: string) => {
        if (!props.readOnly && textareaRef.current) {
            const textarea = textareaRef.current;
            const cursorPos = textarea.selectionStart;
            const currentValue = textarea.value; // Use current textarea value
            const textBefore = currentValue.substring(0, cursorPos);
            const textAfter = currentValue.substring(cursorPos);
            
            const newText = textBefore + text + textAfter;
            setCode(newText);
            
            // Save to session storage
            sessionStorage.setItem('programEditorCode', newText);
            
            // Set cursor position after the inserted text
            const newCursorPos = cursorPos + text.length;
            setTimeout(() => {
                if (textareaRef.current) {
                    textareaRef.current.setSelectionRange(newCursorPos, newCursorPos);
                    textareaRef.current.focus();
                }
            }, 0);
        }
    };

    // Function to read the program code 
    const readProgramCode = (): string => {
        return code;
    };
    
    // Function to clear the program
    const clearProgram = () => {
        setCode("");
        sessionStorage.removeItem('programEditorCode');
    };

    // Function to add a new saved position
    const addSavedPosition = (positionName: string) => {
        if (!savedPositions.includes(positionName)) {
            setSavedPositions(prev => {
                const updatedPositions = [...prev, positionName];
                // Save to session storage
                sessionStorage.setItem('programEditorSavedPositions', JSON.stringify(updatedPositions));
                return updatedPositions;
            });
        }
    };
    
    // Function to add a custom pose
    const addCustomPose = (poseName: string, pose: RobotPose) => {
        console.log("Adding custom pose:", poseName, pose);
        setCustomPoses(prev => {
            const updatedPoses = { ...prev, [poseName]: pose };
            console.log("Updated poses:", updatedPoses);
            // Save to session storage
            sessionStorage.setItem('programEditorCustomPoses', JSON.stringify(updatedPoses));
            console.log("Saved to session storage:", sessionStorage.getItem('programEditorCustomPoses'));
            return updatedPoses;
        });
    };

    // Expose the functions to sharedState
    React.useEffect(() => {
        if ((props.sharedState as any).insertTextAtCursor === undefined) {
            // Add the insert function to sharedState if it doesn't exist
            (props.sharedState as any).insertTextAtCursor = insertTextAtCursor;
        }
        if ((props.sharedState as any).addSavedPosition === undefined) {
            (props.sharedState as any).addSavedPosition = addSavedPosition;
        }
        if ((props.sharedState as any).addCustomPose === undefined) {
            (props.sharedState as any).addCustomPose = addCustomPose;
        }
    }, [props.sharedState]);

    // Update line numbers when code changes
    useEffect(() => {
        const lines = code.split('\n');
        const numbers = lines.map((_, index) => (index + 1).toString());
        setLineNumbers(numbers);
    }, [code]);

    // Sync scroll between textarea and line numbers
    const handleScroll = () => {
        if (textareaRef.current && lineNumbersRef.current) {
            lineNumbersRef.current.scrollTop = textareaRef.current.scrollTop;
        }
        if (textareaRef.current && highlightedRef.current) {
            highlightedRef.current.scrollTop = textareaRef.current.scrollTop;
            highlightedRef.current.scrollLeft = textareaRef.current.scrollLeft;
        }
    };

    // Get the current word being typed
    const getCurrentWord = (): { word: string; start: number; end: number } => {
        if (!textareaRef.current) return { word: '', start: 0, end: 0 };
        
        const textarea = textareaRef.current;
        const cursorPos = textarea.selectionStart;
        const text = textarea.value;
        
        // Find the start of the current word
        let start = cursorPos;
        while (start > 0 && /[a-zA-Z]/.test(text[start - 1])) {
            start--;
        }
        
        // Find the end of the current word
        let end = cursorPos;
        while (end < text.length && /[a-zA-Z]/.test(text[end])) {
            end++;
        }
        
        const word = text.substring(start, end);
        return { word, start, end };
    };

    // Update suggestion based on current word
    const updateSuggestion = (word: string) => {
        if (word.length === 0) {
            setShowSuggestion(false);
            setCurrentSuggestion("");
            return;
        }
        const filtered = allFunctions.filter(func => 
            func.toLowerCase().startsWith(word.toLowerCase()) && func.toLowerCase() !== word.toLowerCase()
        );
        if (filtered.length > 0) {
            setCurrentSuggestion(filtered[0]);
            setShowSuggestion(true);
        } else {
            setShowSuggestion(false);
            setCurrentSuggestion("");
        }
    };

    // Complete current word with suggestion
    const completeWord = () => {
        if (!textareaRef.current || !showSuggestion) return;
        
        const textarea = textareaRef.current;
        const { start, end } = getCurrentWord();
        const text = textarea.value;
        
        // Check if the suggestion is a function or a saved position
        const isFunction = [...ROBOT_FUNCTIONS, ...HUMAN_FUNCTIONS].includes(currentSuggestion);
        const suffix = isFunction ? '()' : '';
        
        const newText = text.substring(0, start) + currentSuggestion + suffix + text.substring(end);
        setCode(newText);
        
        // Save to session storage
        sessionStorage.setItem('programEditorCode', newText);
        
        // Set cursor position after the completed word
        const newCursorPos = start + currentSuggestion.length + suffix.length;
        setTimeout(() => {
            textarea.selectionStart = textarea.selectionEnd = newCursorPos;
            textarea.focus();
        }, 0);
        
        setShowSuggestion(false);
        setCurrentSuggestion("");
    };

    // Handle code changes
    const handleCodeChange = (event: React.ChangeEvent<HTMLTextAreaElement>) => {
        if (!props.readOnly) {
            const newCode = event.target.value;
            setCode(newCode);
            
            // Save to session storage for persistence across mode switches
            sessionStorage.setItem('programEditorCode', newCode);
            
            // Update suggestion based on current word
            const { word } = getCurrentWord();
            updateSuggestion(word);
        }
    };

    const handleKeyDown = (event: React.KeyboardEvent<HTMLTextAreaElement>) => {
        if (event.key === 'Tab') {
            event.preventDefault();
            
            if (showSuggestion) {
                // Complete with suggestion
                completeWord();
            } else {
                // Insert tab as usual
                const target = event.target as HTMLTextAreaElement;
                const start = target.selectionStart;
                const end = target.selectionEnd;
                
                const newCode = code.substring(0, start) + '    ' + code.substring(end);
                setCode(newCode);
                
                // Save to session storage
                sessionStorage.setItem('programEditorCode', newCode);
                
                // Set cursor position after the inserted tab
                setTimeout(() => {
                    target.selectionStart = target.selectionEnd = start + 4;
                }, 0);
            }
        } else if (event.key === 'Enter' && showSuggestion) {
            event.preventDefault();
            completeWord();
        } else if (event.key === 'Escape' && showSuggestion) {
            event.preventDefault();
            setShowSuggestion(false);
            setCurrentSuggestion("");
        }
    };

    /** Callback when component is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<HTMLDivElement>) => {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    };

    // In customizing state add onClick callback
    const selectProp = customizing ? { onClick: onSelect } : {};

    // Function to handle Run/Stop Program button click
    const handleRunProgram = async () => {
        console.log("handleRunProgram called, current isExecuting:", isExecuting);
        if (isExecuting) {
            // Stop execution
            console.log("Stop Program button clicked!");
            setIsExecuting(false);
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
            console.log("Run Program button clicked!");
            console.log("Setting isExecuting to true");
            setIsExecuting(true);
            
            // Reset current executing line at start
            if (props.sharedState.updateCurrentExecutingLine) {
                props.sharedState.updateCurrentExecutingLine(undefined);
            }
            
            const programText = readProgramCode();
            console.log("Program text:", programText);
            
            // Write program to file
            const userId = 0; // For testing, using temporary constant 
            const fileName = `user_${userId}_program.json`;
            const filePath = `/media/hello-robot/HCRLAB/data/${fileName}`;
            
            // Create JSON object with program data 
            const programData = {
                timestamp: new Date().toISOString(),
                userId: userId,
                programText: programText,
                programLines: parseProgram(programText).lines
            };
            
            const fileContent = JSON.stringify(programData, null, 2);
            
            // Save File 
            try {
                const requestBody = {
                    filePath: filePath,
                    fileName: fileName,
                    content: fileContent
                };
                
                console.log('Sending save_program request:', requestBody);
                
                const response = await fetch('/save_program', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify(requestBody)
                });
                
                if (response.ok) {
                    const result = await response.json();
                    console.log(`Program saved successfully:`, result);
                } else {
                    const errorData = await response.json();
                    console.error('Failed to save program file:', errorData);
                }
            } catch (error) {
                console.error('Error saving program file:', error);
            }
            
            // Parse the program 
            const program = parseProgram(programText);
            console.log("Parsed program:", program);
            
            // Log lines for debugging
            program.lines.forEach(line => {
                if (line.isExecutable) {
                    console.log(`Line ${line.lineNumber}: Executable command "${line.command}"`);
                } else {
                    console.log(`Line ${line.lineNumber}: Non-executable (${line.content})`);
                }
            });
            
            // Execute the program line by line
            executeProgram(program);
            
            if (props.onRunProgram) {
                props.onRunProgram(programText);
            }
        }
    };

    // Create highlighted version of the code with inline suggestion
    const createHighlightedCode = (): string => {
        let highlightedText = highlightSyntax(code);
        
        if (showSuggestion && currentSuggestion) {
            const { word, start, end } = getCurrentWord();
            if (word.length > 0) {
                // Add the suggestion as grey text after the current word
                const beforeWord = code.substring(0, start);
                const afterWord = code.substring(end);
                
                // Check if the suggestion is a function or a saved position
                const isFunction = [...ROBOT_FUNCTIONS, ...HUMAN_FUNCTIONS].includes(currentSuggestion);
                const suffix = isFunction ? '()' : '';
                
                const suggestionText = currentSuggestion.substring(word.length) + suffix;
                
                // Create the highlighted version with suggestion
                const beforeHighlighted = highlightSyntax(beforeWord);
                const afterHighlighted = highlightSyntax(afterWord);
                
                highlightedText = beforeHighlighted + word + `<span class="inline-suggestion">${suggestionText}</span>` + afterHighlighted;
            }
        }
        
        return highlightedText;
    };

    const highlightedCode = createHighlightedCode();

    return (
        <div
            className={className("program-editor-root", {
                customizing,
                selected,
            })}
            style={{
                fontSize: window.innerWidth < 1200 ? "14px" : "16px",
                padding: window.innerWidth < 1200 ? "8px" : "12px"
            }}
            {...selectProp}
        >
            <div className="program-editor-header" style={{
                padding: window.innerWidth < 1200 ? "8px" : "12px",
                gap: window.innerWidth < 1200 ? "8px" : "12px"
            }}>
                <div className="program-editor-header-left" style={{
                    fontSize: window.innerWidth < 1200 ? "12px" : "14px"
                }}>
                    {props.language && (
                        <span className="program-editor-language">{props.language}</span>
                    )}
                    {!props.readOnly && (
                        <button 
                            className="clear-program-button"
                            onClick={clearProgram}
                            type="button"
                            style={{
                                fontSize: window.innerWidth < 1200 ? "12px" : "14px",
                                padding: window.innerWidth < 1200 ? "6px 12px" : "8px 16px"
                            }}
                        >
                            Clear
                        </button>
                    )}
                    {isExecuting && (
                        <span 
                            style={{
                                color: "#4caf50",
                                fontWeight: "bold",
                                fontSize: window.innerWidth < 1200 ? "12px" : "14px",
                                marginLeft: window.innerWidth < 1200 ? "12px" : "16px"
                            }}
                        >
                            Running... View execution details in the Execution Monitor.
                        </span>
                    )}
                </div>
                <div className="program-editor-header-right" style={{
                    fontSize: window.innerWidth < 1200 ? "12px" : "14px"
                }}>
                    {!props.readOnly && (
                        <button 
                            className="run-program-button"
                            onClick={handleRunProgram}
                            type="button"
                            style={{
                                backgroundColor: isExecuting ? "#dc3545" : undefined,
                                fontSize: window.innerWidth < 1200 ? "12px" : "14px",
                                padding: window.innerWidth < 1200 ? "6px 12px" : "8px 16px"
                            }}
                        >
                            {isExecuting ? (
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
                    )}
                </div>
            </div>
            {executionError && (
                <div className="program-editor-error-banner">
                                    <div style={{ display: "flex", alignItems: "center", gap: "8px" }}>
                    <ErrorIcon style={{ fontSize: "16px" }} />
                    {executionError.message}
                </div>
                    <button 
                        className="program-editor-error-close"
                        onClick={clearExecutionError}
                        type="button"
                    >
                        ×
                    </button>
                </div>
            )}
            <div className="program-editor-container">
                <div className="line-numbers" ref={lineNumbersRef}>
                    {lineNumbers.map((number, index) => {
                        const lineNumber = index + 1;
                        const hasError = errorLineNumber === lineNumber;
                        
                        return (
                            <div 
                                key={index} 
                                className={className("line-number", {
                                    error: hasError
                                })}
                            >
                                {number}
                            </div>
                        );
                    })}
                </div>
                <div className="editor-wrapper">
                    <textarea
                        ref={textareaRef}
                        className="code-textarea"
                        value={code}
                        onChange={handleCodeChange}
                        onKeyDown={handleKeyDown}
                        onScroll={handleScroll}
                        readOnly={props.readOnly}
                        placeholder="Enter your code here..."
                        spellCheck={false}
                    />
                    <div 
                        ref={highlightedRef}
                        className="code-highlighted"
                        dangerouslySetInnerHTML={{ __html: highlightedCode }}
                    />
                </div>
            </div>
        </div>
    );
}; 