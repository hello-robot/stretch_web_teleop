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

/**
 * A code editor component with line numbers and syntax highlighting
 * similar to VS Code interface
 *
 * @param props {@link ProgramEditorProps}
 */
export const ProgramEditor = (props: ProgramEditorProps) => {
    const [code, setCode] = useState(props.initialCode || "");
    const [lineNumbers, setLineNumbers] = useState<string[]>([]);
    const textareaRef = useRef<HTMLTextAreaElement>(null);
    const lineNumbersRef = useRef<HTMLDivElement>(null);
    
    const { customizing } = props.sharedState;
    const selected = isSelected(props);

    // Function to highlight code with colors
    const highlightCode = (codeText: string): string => {
        const robotFunctions = [
            'MoveEEToPose(x,y,z)',
            'AdjustGripperWidth()',
            'RotateEE(theta)',
            'ResetRobot()'
        ];
        const humanFunctions = [
            'PauseAndConfirm()',
            'GiveControl()',
            'TakeControl()'
        ];
        
        let highlightedCode = codeText;
        
        // Highlight robot functions in orange
        robotFunctions.forEach(func => {
            const regex = new RegExp(`\\b${func.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}\\b`, 'g');
            highlightedCode = highlightedCode.replace(regex, `<span style="color: #ff8c00;">${func}</span>`);
        });
        
        // Highlight human functions in green
        humanFunctions.forEach(func => {
            const regex = new RegExp(`\\b${func.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}\\b`, 'g');
            highlightedCode = highlightedCode.replace(regex, `<span style="color: #28a745;">${func}</span>`);
        });
        
        return highlightedCode;
    };

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
    };

    // Handle code changes
    const handleCodeChange = (event: React.ChangeEvent<HTMLTextAreaElement>) => {
        if (!props.readOnly) {
            setCode(event.target.value);
        }
    };

    // Handle tab key
    const handleKeyDown = (event: React.KeyboardEvent<HTMLTextAreaElement>) => {
        if (event.key === 'Tab') {
            event.preventDefault();
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
    };

    /** Callback when component is clicked during customize mode */
    const onSelect = (event: React.MouseEvent<HTMLDivElement>) => {
        event.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    };

    // In customizing state add onClick callback
    const selectProp = customizing ? { onClick: onSelect } : {};

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
                <div className="code-editor-wrapper" style={{ position: 'relative', width: '100%', height: '100%' }}>
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
                        style={{
                            position: 'absolute',
                            top: 0,
                            left: 0,
                            width: '100%',
                            height: '100%',
                            backgroundColor: 'transparent',
                            color: 'transparent',
                            caretColor: 'var(--text-color)',
                            zIndex: 2
                        }}
                    />
                    <div 
                        className="code-highlight-overlay"
                        dangerouslySetInnerHTML={{ __html: highlightCode(code) }}
                        style={{
                            position: 'absolute',
                            top: 0,
                            left: 0,
                            width: '100%',
                            height: '100%',
                            pointerEvents: 'none',
                            zIndex: 1,
                            whiteSpace: 'pre-wrap',
                            fontFamily: 'monospace',
                            fontSize: '14px',
                            lineHeight: '1.5',
                            color: 'var(--text-color)',
                            backgroundColor: 'transparent',
                            overflow: 'hidden'
                        }}
                    />
                </div>
            </div>
        </div>
    );
}; 