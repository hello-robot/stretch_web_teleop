'use client'

import { useState, useEffect, useRef, ReactNode } from 'react'
import { motion, useMotionValue, useSpring } from 'framer-motion'

const SPRING_CONFIG = { damping: 30, stiffness: 150, mass: 0.2 }
const MAX_DISTANCE_FACTOR = 0.1;
const MAX_SCALE_VALUE = 1.05;

interface MagneticWrapperProps {
    children: ReactNode;
}

const MagneticWrapper: React.FC<MagneticWrapperProps> = ({ children }) => {
    const [isHovered, setIsHovered] = useState(false)
    const x = useMotionValue(0)
    const y = useMotionValue(0)
    const scale = useMotionValue(1)
    const ref = useRef<HTMLDivElement>(null) // Corrected type annotation
    const springX = useSpring(x, SPRING_CONFIG)
    const springY = useSpring(y, SPRING_CONFIG)
    const springScale = useSpring(scale, { damping: 20, stiffness: 300 })

    useEffect(() => {
        const calculateDistanceAndApplyEffect = (e: MouseEvent) => {
            if (ref.current) {
                const rect = ref.current.getBoundingClientRect()
                const centerX = rect.left + rect.width / 2
                const centerY = rect.top + rect.height / 2

                if (isHovered) {
                    const distanceX = (e.clientX - centerX) * MAX_DISTANCE_FACTOR
                    const distanceY = (e.clientY - centerY) * MAX_DISTANCE_FACTOR
                    x.set(distanceX)
                    y.set(distanceY)
                    scale.set(MAX_SCALE_VALUE)
                } else {
                    // This else block will only be hit if isHovered becomes false
                    // while the mousemove event is still somehow processed by an old listener instance,
                    // which shouldn't happen with the current dependency array.
                    // The primary reset happens onMouseLeave.
                    x.set(0)
                    y.set(0)
                    scale.set(1)
                }
            }
        }

        const handleMouseMove = (e: MouseEvent) => {
            // Only calculate if the element itself is hovered or if we need to track globally for some reason
            // Based on onMouseEnter/onMouseLeave, we only care when hovered.
            // However, the effect is applied to document, so it tracks globally.
            // The `isHovered` check inside calculateDistanceAndApplyEffect handles this.
            requestAnimationFrame(() => calculateDistanceAndApplyEffect(e))
        }

        // This listener is global. The effect's logic is gated by `isHovered`.
        document.addEventListener('mousemove', handleMouseMove)

        return () => {
            document.removeEventListener('mousemove', handleMouseMove)
            // Optional: Reset values if the component unmounts while hovered,
            // though onMouseLeave should handle most cases.
            // x.set(0);
            // y.set(0);
            // scale.set(1);
        }
    }, [isHovered, x, y, scale]) // Effect re-runs if isHovered changes, ensuring correct logic

    const handleMouseLeave = () => {
        setIsHovered(false)
        x.set(0)
        y.set(0)
        scale.set(1)
    }

    return (
        <motion.div
            ref={ref}
            onMouseEnter={() => setIsHovered(true)}
            onMouseLeave={handleMouseLeave}
            style={{
                position: 'relative', // Ensures children are positioned relative to this moving div
                x: springX,
                y: springY,
                scale: springScale,
                width: '100%',
            }}
        >
            {children}
        </motion.div>
    )
}

export default MagneticWrapper