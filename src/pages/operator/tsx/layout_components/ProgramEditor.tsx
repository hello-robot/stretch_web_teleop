import React, { useState, useRef, useEffect } from "react";
import {
    CustomizableComponentProps,
    SharedState,
    isSelected,
} from "./CustomizableComponent";
import { className, RobotPose } from "shared/util";
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
    'GiveControl',
    'TakeControl'
];

// Saved positions (blue) - will be made dynamic
const DEFAULT_SAVED_POSITIONS = [
    'stowGripper',
    'testPose',
    'centerWrist'
];

// Define poses that can be referenced in the program
const POSE_DEFINITIONS = {
    stowGripper: {
        joint_wrist_roll: 0.0,
        joint_wrist_pitch: -0.497,
        joint_wrist_yaw: 3.19579,
        joint_lift: 0.4,
    },
    centerWrist: {
        joint_wrist_roll: 0.0,
        joint_wrist_pitch: 0.0,
        joint_wrist_yaw: 0.0,
        joint_lift: 0.6,
    },
    realsenseForward: {
        joint_head_pan: 0.075,
        joint_head_tilt: 0.0,
    },
    realsenseBase: {
        joint_head_pan: 0.075,
        joint_head_tilt: -1.1,
    },
    testPose: {
        joint_lift: 0.8,
    }
};

// Program data structure for parsing 
interface ProgramLine {
    lineNumber: number;
    content: string;
    command?: string;  // ex.MoveEEToPose
    parameters?: any;   
    isExecutable: boolean; // handling for empty lines or if invalid 
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
            const adjustGripperMatch = trimmedLine.match(/AdjustGripperWidth\s*\(\s*([^)]*)\s*\)/);
            const rotateEEMatch = trimmedLine.match(/RotateEE\s*\(\s*([^)]*)\s*\)/);
            
            if (moveEEMatch) {
                const parameter = moveEEMatch[1] || null;
                programLines.push({
                    lineNumber,
                    content: line,
                    command: "MoveEEToPose",
                    parameters: parameter,
                    isExecutable: true
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
            } else {
                // Unknown command or invalid line
                programLines.push({
                    lineNumber,
                    content: line,
                    isExecutable: false
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
    const [code, setCode] = useState(props.initialCode || "");
    const [lineNumbers, setLineNumbers] = useState<string[]>([]);
    const [currentSuggestion, setCurrentSuggestion] = useState<string>("");
    const [showSuggestion, setShowSuggestion] = useState(false);
    const [savedPositions, setSavedPositions] = useState<string[]>(DEFAULT_SAVED_POSITIONS);
    const textareaRef = useRef<HTMLTextAreaElement>(null);
    const lineNumbersRef = useRef<HTMLDivElement>(null);
    const highlightedRef = useRef<HTMLDivElement>(null);
    
    const { customizing } = props.sharedState;
    const selected = isSelected(props);

    // Create dynamic ALL_FUNCTIONS array
    const allFunctions = [...ROBOT_FUNCTIONS, ...HUMAN_FUNCTIONS, ...savedPositions];

        // Function to wait for goal completion
        const waitForPoseCompletion = (targetPose: RobotPose): Promise<void> => {
            //TODO: implement this by storing the target pose + comparing w/ current joint states
            return new Promise((resolve) => {
                // For now, just resolve immediately
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
        
        for (const line of program.lines) {
            if (line.isExecutable) {
                console.log(`Executing line ${line.lineNumber}: ${line.command} with parameter: ${line.parameters}`);
                
                // Executes the command based on what function it is
                if (line.command === "MoveEEToPose") {
                    const poseName = line.parameters;
                    const pose = POSE_DEFINITIONS[poseName as keyof typeof POSE_DEFINITIONS];
                    
                    if (pose) {
                        console.log(`Sending MoveEEToPose command with pose: ${poseName}`, pose);
                        // Send command to robot
                        if ((window as any).remoteRobot) {
                            (window as any).remoteRobot.setRobotPose(pose);
                            console.log(`Command sent to robot!`);
                            //testing this out 
                            console.log(`Waiting...`);
                            await new Promise(resolve => setTimeout(resolve, 5000));
                            console.log(`Executing next command...`);
                        } else {
                            console.error("RemoteRobot not available");
                        }
                    } else {
                        console.error(`Unknown pose: ${poseName}. Available poses: ${Object.keys(POSE_DEFINITIONS).join(', ')}`);
                    }
                }
                else if (line.command === "AdjustGripperWidth") {
                    const poseName = line.parameters;
                    const pose = POSE_DEFINITIONS[poseName as keyof typeof POSE_DEFINITIONS];
                    if (pose) {
                        console.log(`Sending AdjustGripperWidth command with pose: ${poseName}`, pose);
                        // Send command to robot
                        if ((window as any).remoteRobot) {
                            //TODO: implement sending robot the correct command
                            console.log(`Command sent to robot!`);
                            console.log(`Waiting...`);
                            await new Promise(resolve => setTimeout(resolve, 5000));
                            console.log(`Executing next command...`);
                        } else {
                            console.error("RemoteRobot not available");
                        }
                    } else {
                        console.error(`Unknown pose: ${poseName}. Available poses: ${Object.keys(POSE_DEFINITIONS).join(', ')}`);
                    }
                }
                else if (line.command === "RotateEE") {
                    const poseName = line.parameters;
                    const pose = POSE_DEFINITIONS[poseName as keyof typeof POSE_DEFINITIONS];
                    if (pose) {
                        console.log(`Sending RotateEE command with pose: ${poseName}`, pose);
                        // Send command to robot
                        if ((window as any).remoteRobot) {
                            //TODO: implement sending robot the correct command
                            console.log(`Command sent to robot!`);
                            console.log(`Waiting...`);
                            await new Promise(resolve => setTimeout(resolve, 5000));
                            console.log(`Executing next command...`);
                        } else {
                            console.error("RemoteRobot not available");
                        }
                    } else {
                        console.error(`Unknown pose: ${poseName}. Available poses: ${Object.keys(POSE_DEFINITIONS).join(', ')}`);
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
                    console.log(`Pausing program execution for user confirmation`);
                    // Set execution state to false to allow manual control
                    if (buttonFunctionProvider) {
                        buttonFunctionProvider.setExecutionState(false);
                    }
                    console.log(`Program paused. Waiting for user to resume...`);
                    // TODO: Implement pause and wait for user confirmation
                }
                else if (line.command === "TakeControl") {
                    console.log(`Taking control from robot`);
                    // Set execution state to false to allow manual control
                    if (buttonFunctionProvider) {
                        buttonFunctionProvider.setExecutionState(false);
                    }
                    console.log(`Control returned to user`);
                }
            } else {
                console.log(`Skipping line ${line.lineNumber}: ${line.content}`);
            }
        }
        
        console.log("Program execution complete!");
        
        // Set execution state to false at the end 
        if (buttonFunctionProvider) {
            buttonFunctionProvider.setExecutionState(false);
        }
    };

    // Syntax highlighting function
    const highlightSyntax = (text: string): string => {
        let highlightedText = text;
        
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
        
        return highlightedText;
    };

    // Function to add text to the editor (as new line)
    const addText = (text: string) => {
        if (!props.readOnly) {
            setCode(prevCode => prevCode + (prevCode.endsWith('\n') || prevCode === '' ? '' : '\n') + text);
        }
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

    // Function to add a new saved position
    const addSavedPosition = (positionName: string) => {
        if (!savedPositions.includes(positionName)) {
            setSavedPositions(prev => [...prev, positionName]);
        }
    };

    // Expose the functions to sharedState
    React.useEffect(() => {
        if (props.sharedState.addToProgramEditor === undefined) {
            // Add the function to sharedState if it doesn't exist
            (props.sharedState as any).addToProgramEditor = addText;
        }
        if ((props.sharedState as any).insertTextAtCursor === undefined) {
            // Add the insert function to sharedState if it doesn't exist
            (props.sharedState as any).insertTextAtCursor = insertTextAtCursor;
        }
        if ((props.sharedState as any).addSavedPosition === undefined) {
            // Add the function to add saved positions if it doesn't exist
            (props.sharedState as any).addSavedPosition = addSavedPosition;
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
        
        // Check if the suggestion is a function (robot or human function) or a saved position
        const isFunction = [...ROBOT_FUNCTIONS, ...HUMAN_FUNCTIONS].includes(currentSuggestion);
        const suffix = isFunction ? '()' : '';
        
        const newText = text.substring(0, start) + currentSuggestion + suffix + text.substring(end);
        setCode(newText);
        
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
            setCode(event.target.value);
            
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

    // Function to handle Run Program button click
    const handleRunProgram = () => {
        console.log("Run Program button clicked!");
        
        const programText = readProgramCode();
        console.log("Program text:", programText);
        
        // Write program to file
        const userId = 0; // For testing, temporary constant 
        const fileName = `user_${userId}_program.json`;
        const filePath = `/HCRLAB/data/${fileName}`;
        
        // Create JSON object with program data 
        const programData = {
            timestamp: new Date().toISOString(),
            userId: userId,
            programText: programText,
            programLines: parseProgram(programText).lines
        };
        
        const fileContent = JSON.stringify(programData, null, 2);
        
        const blob = new Blob([fileContent], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = fileName;
        a.style.display = 'none';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        
        console.log(`Program saved to: ${filePath}`);
        
        // Parse the program into structured format
        const program = parseProgram(programText);
        console.log("Parsed program:", program);
        
        // Log each line for debugging
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
                
                // Check if the suggestion is a function (robot or human function) or a saved position
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
            {...selectProp}
        >
            <div className="program-editor-header">
                <span className="program-editor-title">Program Editor</span>
                <div className="program-editor-header-right">
                    {props.language && (
                        <span className="program-editor-language">{props.language}</span>
                    )}
                    {!props.readOnly && (
                        <button 
                            className="run-program-button"
                            onClick={handleRunProgram}
                            type="button"
                        >
                            Run Program
                        </button>
                    )}
                </div>
            </div>
            <div className="program-editor-container">
                <div className="line-numbers" ref={lineNumbersRef}>
                    {lineNumbers.map((number, index) => (
                        <div key={index} className="line-number">
                            {number}
                        </div>
                    ))}
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