import React, { useState, useRef, useEffect } from "react";
import {
    CustomizableComponentProps,
    SharedState,
    isSelected,
} from "./CustomizableComponent";
import { className } from "shared/util";
import "operator/css/ProgramEditor.css";

/** Properties for {@link ProgramEditor} */
type ProgramEditorProps = CustomizableComponentProps & {
    /* Initial code content */
    initialCode?: string;
    /* Programming language for syntax highlighting */
    language?: string;
    /* Whether the editor is read-only */
    readOnly?: boolean;
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

// Saved positions (blue)
const SAVED_POSITIONS = [
    'approach_pose',
    'red_cube_pose',
    'end_pose'
];

// All available functions for tab completion
const ALL_FUNCTIONS = [...ROBOT_FUNCTIONS, ...HUMAN_FUNCTIONS, ...SAVED_POSITIONS];

/**
 * Syntax highlighting function for robot and human functions
 */
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
    SAVED_POSITIONS.forEach(position => {
        const regex = new RegExp(`\\b${position}\\b`, 'g');
        highlightedText = highlightedText.replace(regex, `<span class="saved-position">${position}</span>`);
    });
    
    return highlightedText;
};

/**
 * A code editor component with line numbers and syntax highlighting
 * similar to VS Code interface
 *
 * @param props {@link ProgramEditorProps}
 */
export const ProgramEditor = (props: ProgramEditorProps) => {
    const [code, setCode] = useState(props.initialCode || "");
    const [lineNumbers, setLineNumbers] = useState<string[]>([]);
    const [currentSuggestion, setCurrentSuggestion] = useState<string>("");
    const [showSuggestion, setShowSuggestion] = useState(false);
    const textareaRef = useRef<HTMLTextAreaElement>(null);
    const lineNumbersRef = useRef<HTMLDivElement>(null);
    const highlightedRef = useRef<HTMLDivElement>(null);
    
    const { customizing } = props.sharedState;
    const selected = isSelected(props);

    // Function to add text to the editor
    const addText = (text: string) => {
        if (!props.readOnly) {
            setCode(prevCode => prevCode + (prevCode.endsWith('\n') || prevCode === '' ? '' : '\n') + text);
        }
    };

    // Expose the addText function to sharedState
    React.useEffect(() => {
        if (props.sharedState.addToProgramEditor === undefined) {
            // Add the function to sharedState if it doesn't exist
            (props.sharedState as any).addToProgramEditor = addText;
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

    // Get current word being typed
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
        
        const filtered = ALL_FUNCTIONS.filter(func => 
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

    // Complete the current word with the suggestion
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

    // Handle key events
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
                {props.language && (
                    <span className="program-editor-language">{props.language}</span>
                )}
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