import React, { ReactNode, memo, useMemo, } from 'react'
import 'operator/css/DirectionalPad.css'

interface DirectionalPadProps {
    mapButtons: (direction: string, index: number) => ReactNode
    isCameraVeilVisible: boolean
}

const DIRECTIONS = [
    'north',
    'south',
    'west',
    'east',
    'rotate-left',
    'rotate-right',
] as const

const DirectionalPad: React.FC<DirectionalPadProps> = ({
    mapButtons,
    isCameraVeilVisible,
}) => {

    // only re‐map when mapButtons fn changes
    const buttons = useMemo(() => DIRECTIONS.map(mapButtons), [mapButtons])

    // slice out once, only when `buttons` changes
    const cardinal = useMemo(() => buttons.slice(0, 4), [buttons])
    const rotate = useMemo(() => buttons.slice(4), [buttons])

    if (isCameraVeilVisible) return null

    else return (
        <div
            className={`base-movement-controls ${isCameraVeilVisible ? 'display-none' : 'display-block'}`}
            tabIndex={0}
        >
            <svg className="mask">
                <clipPath
                    id="clip-path"
                    clipPathUnits="objectBoundingBox"
                >
                    <path d="M1,1 H0 V0.5 C0.276,0.5,0.5,0.276,0.5,0 H1 V1" />
                </clipPath>
            </svg>

            <div className="dbutton-pad">
                {cardinal.map((btn, i) => (
                    <React.Fragment key={i}>{btn}</React.Fragment>
                ))}
            </div>
            <div className="button-turn-wrapper">
                {rotate.map((btn, i) => (
                    <React.Fragment key={i}>{btn}</React.Fragment>
                ))}
                <div className="line" />
            </div>
        </div>
    )
}

export default memo(DirectionalPad)
