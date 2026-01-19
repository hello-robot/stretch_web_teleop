import React from 'react'

interface FlexProps {
    children: React.ReactNode;
    direction?: 'row' | 'column';
    justify?: 'flex-start' | 'flex-end' | 'center' | 'space-between' | 'space-around';
    align?: 'flex-start' | 'flex-end' | 'center' | 'stretch' | 'baseline';
    gap?: number | string;
    style?: React.CSSProperties;
    className?: string;
}

function Flex({
    children,
    direction = 'row',
    justify = 'flex-start',
    align = 'flex-start',
    gap = 0,
    style,
    className,
}: FlexProps) {
    return (
        <div
            style={{
                display: 'flex',
                flexDirection: direction,
                justifyContent: justify,
                alignItems: align,
                gap,
                ...style
            }}
            className={className}
        >
            {children}
        </div>
    )
}

export default Flex