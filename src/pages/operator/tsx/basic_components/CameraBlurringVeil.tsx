import React, { ReactNode } from 'react';

interface CameraBlurringVeilProps {
  isVisible: boolean;
}

function CameraBlurringVeil({ isVisible }: CameraBlurringVeilProps) {
  return (
    <div
      style={{
        position: 'absolute',
        top: 0,
        left: 0,
        width: '100%',
        height: '100%',
        backgroundColor: 'hsla(0, 0.00%, 0.00%, 0.40)',
        zIndex: 1,
        backdropFilter: isVisible ? 'blur(10px)' : 'blur(0px)',
        opacity: isVisible ? 1 : 0,
        pointerEvents: isVisible ? 'auto' : 'none',
        transition: 'opacity 250ms ease-out, backdrop-filter 250ms ease-out',
      }}
    />
  );
}

export default CameraBlurringVeil;