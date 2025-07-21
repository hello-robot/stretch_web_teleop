import React, { useState, useRef, useEffect } from 'react';
import '../../css/InputFluid.css';

interface InputFluidProps {
    value: string;
    onChange: (e: React.ChangeEvent<HTMLInputElement>) => void;
    inputRef?: React.RefObject<HTMLInputElement>; // Optional ref for parent focus control
    minWidth?: number; // Minimum width in pixels
    maxWidth?: number; // Maximum width in pixels
    disabled?: boolean;
    placeholder?: string;
    onFocus?: (e: React.ChangeEvent<HTMLInputElement>) => void;
    onBlur?: (e: React.PointerEvent<HTMLButtonElement>) => void; // Optional onBlur handler
    autoComplete?: string; // Optional autocomplete attribute
    classNameInput?: string; // Optional class name for styling
    onClick?: (e: React.PointerEvent<HTMLInputElement>) => void;
    onPointerDown?: (e: React.PointerEvent<HTMLButtonElement>) => void;
}

/**
 * InputFluid is a flexible input component that
 * adjusts its width based on its content/placeholder.
 *
 * @param {InputFluidProps} props - The properties for the input component.
 * @returns {JSX.Element} The rendered input component.
 */
const InputFluid: React.FC<InputFluidProps> = ({
    inputRef,
    value,
    onChange,
    placeholder = '',
    minWidth = 0,
    maxWidth = 300,
    disabled = false,
    onBlur,
    autoComplete = 'off',
    classNameInput = '',
    onClick = () => { }, // Default to no-op
    onPointerDown = () => { }, // Default to no-op
}) => {
    const [inputWidth, setInputWidth] = useState<number>(minWidth);
    const spanRef = useRef<HTMLSpanElement>(null);

    // Adjust the input width based on its content
    useEffect(() => {
        if (spanRef.current) {
            // Measure the width of the hidden span containing the input value or placeholder
            const text = value || placeholder;
            spanRef.current.textContent = text || ' '; // Ensure span has content to measure
            let newWidth = spanRef.current.offsetWidth + 3; // Add padding for comfort
            newWidth = Math.max(minWidth, newWidth); // Enforce minimum width
            if (maxWidth) {
                newWidth = Math.min(newWidth, maxWidth); // Enforce maximum width if provided
            }
            setInputWidth(newWidth);
        }
    }, [value, placeholder, minWidth, maxWidth]);

    return (
        <div className="input-fluid-container">
            <span ref={spanRef} className="input-fluid-measure" />
            <input
                type="text"
                value={value}
                onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
                    onChange(e);
                }}
                placeholder={placeholder}
                className={`input-fluid ${classNameInput}`}
                style={{ width: `${inputWidth}px` }}
                disabled={disabled}
                ref={inputRef}
                autoComplete={autoComplete}
                onBlur={onBlur} // Call the onBlur prop if provided
                onClick={onClick}
                onPointerDown={onPointerDown}
            />
        </div>
    );
};

export default InputFluid;