import React, { useState, useRef, useEffect } from 'react';
import '../../css/InputFluid.css';

interface InputFluidProps {
    inputRef?: React.RefObject<HTMLInputElement>; // Optional ref for parent focus control
    value: string;
    onChange: (e: React.ChangeEvent<HTMLInputElement>) => void;
    placeholder?: string;
    minWidth?: number; // Minimum width in pixels
    maxWidth?: number; // Maximum width in pixels
    disabled?: boolean; // Optional prop to disable the input
    onBlur?: () => void; // Optional prop for blur event <handling></handling>
}

const InputFluid: React.FC<InputFluidProps> = ({
    inputRef,
    value,
    onChange,
    placeholder = '',
    minWidth = 10,
    maxWidth = 300,
    disabled = false,
    onBlur,
}) => {
    const [inputWidth, setInputWidth] = useState<number>(minWidth);
    const spanRef = useRef<HTMLSpanElement>(null);

    // Adjust the input width based on its content
    useEffect(() => {
        if (spanRef.current) {
            // Measure the width of the hidden span containing the input value or placeholder
            const text = value || placeholder;
            spanRef.current.textContent = text || ' '; // Ensure span has content to measure
            let newWidth = spanRef.current.offsetWidth; // Add padding for comfort
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
                className="input-fluid"
                style={{ width: `${inputWidth}px` }}
                disabled={disabled}
                ref={inputRef}
                onBlur={onBlur} // Call the onBlur prop if provided
            />
        </div>
    );
};

export default InputFluid;