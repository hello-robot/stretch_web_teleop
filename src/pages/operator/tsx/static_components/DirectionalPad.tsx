import React, { ReactNode } from 'react'
import 'operator/css/DirectionalPad.css'

export default function DirectionalPad({ mapButtons, isCameraVeilVisible }: { mapButtons: (direction: string, i: number) => ReactNode, isCameraVeilVisible: boolean }) {
    const directions = ['north', 'south', 'west', 'east', 'rotate-left', 'rotate-right'];
    const buttons = directions.map(mapButtons);
    const cardinalDirectionButtons = buttons.slice(0, 4)
        .map((node, i) => {
            return <React.Fragment key={i}>{node}</React.Fragment>;
        })
    const rotateDirectionButtons = buttons.slice(4, 6)
        .map((node, i) => {
            return <React.Fragment key={i}>{node}</React.Fragment>;
        })


    return (
        <div className="base-movement-controls">
            <svg className="mask">
                <clipPath id="clip-path" clipPathUnits="objectBoundingBox"><path d="M1,1 H0 V0.5 C0.276,0.5,0.5,0.276,0.5,0 H1 V1"></path></clipPath>
            </svg>
            <div className="dbutton-pad">
                {cardinalDirectionButtons}
            </div>
            <div className="button-turn-wrapper">
                {rotateDirectionButtons}
                <div className='line' />
            </div>
        </div>
    )
}
