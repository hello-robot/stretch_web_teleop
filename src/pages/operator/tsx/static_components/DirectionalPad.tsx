import React from 'react'
import 'operator/css/DirectionalPad.css'

export default function DirectionalPad({ mapButtons }) {
  const buttons = ['north', 'south', 'west', 'east'];
  return (
    <div className="base-movement-controls">
      <svg className="mask">
        <clipPath id="clip-path" clipPathUnits="objectBoundingBox"><path d="M1,1 H0 V0.5 C0.276,0.5,0.5,0.276,0.5,0 H1 V1"></path></clipPath>
      </svg>
      <div className="dbutton-pad">
        {buttons.map(mapButtons)}
      </div>
      <div className="button-turn-wrapper">
        <button className="button-turn left" aria-label="Turn left" onClick={() => console.log('Turn left!')}>
        </button>
        <div className='line' />
        <button className="button-turn right" aria-label="Turn right" onClick={() => console.log('Turn right!')}>
        </button>
      </div>
    </div>
  )
}
