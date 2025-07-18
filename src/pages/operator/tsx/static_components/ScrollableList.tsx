import React, { useCallback, useEffect, useRef, useState } from "react"
import ExpandLessIcon from '@mui/icons-material/ExpandLess';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import "../../css/ScrollableList.css"

interface ScrollableListProps {
    items: React.ReactNode[];
    height: number;
    scrollButtonHeight?: number;
    scrollDistance?: number;
    className?: string;
}

export default function ScrollableList({ items,
    height = 300,
    scrollButtonHeight = 40,
    scrollDistance = 150,
    className = "" }: ScrollableListProps) {

    const [showTopShadow, setShowTopShadow] = useState(false)
    const [showBottomShadow, setShowBottomShadow] = useState(false)
    const scrollRef = useRef<HTMLUListElement>(null)
    const shadowHeight = height * 0.1;

    const handleScroll = () => {
        if (!scrollRef.current) return

        const { scrollTop, scrollHeight, clientHeight } = scrollRef.current

        // Show top shadow if scrolled down
        setShowTopShadow(scrollTop > 0)

        // Show bottom shadow if not at bottom
        setShowBottomShadow(scrollTop + clientHeight < scrollHeight - 1)
    }

    const handleScrollUp = useCallback(() => {
        scrollRef.current
            && scrollRef.current.scrollBy({ top: -scrollDistance, behavior: "smooth" });
    }, []);

    const handleScrollDown = useCallback(() => {
        scrollRef.current
            && scrollRef.current.scrollBy({ top: scrollDistance, behavior: "smooth" });
    }, []);

    useEffect(() => {
        const scrollElement = scrollRef.current
        if (!scrollElement) return

        // Initial check
        handleScroll()

        scrollElement.addEventListener("scroll", handleScroll)

        // Also check on resize
        const resizeObserver = new ResizeObserver(handleScroll)
        resizeObserver.observe(scrollElement)

        return () => {
            scrollElement.removeEventListener("scroll", handleScroll)
            resizeObserver.disconnect()
        }
    }, [items])

    return (
        <div className={`scrollable-container ${className}`}
            style={{
                position: 'relative',
                top: `${scrollButtonHeight}px`,
                height: `${height + (scrollButtonHeight * 2)}px`,
            }}
        >

            {/* Top shadow */}
            <div
                className={`shadow-top ${showTopShadow ? "visible" : ""}`}
                style={{ height: `${shadowHeight}px` }}
            />

            {/* Scroll Up Button */}
            <button
                className="scrollable-list-scroll-btn up"
                onClick={handleScrollUp}
                disabled={!showTopShadow}
                style={{
                    top: `-${scrollButtonHeight}px`,
                    height: `${scrollButtonHeight}px`,
                }}
            >
                <ExpandLessIcon />
            </button>

            {/* Scrollable list */}
            <ul
                ref={scrollRef}
                className="scrollable-list"
                style={{ height: `${height}px` }}
            >
                {items.map((item) => item)}
            </ul>

            {/* Scroll Down Button */}
            <button
                className="scrollable-list-scroll-btn down"
                onClick={handleScrollDown}
                disabled={!showBottomShadow}
                style={{
                    height: `${scrollButtonHeight}px`,
                }}
            >
                <ExpandMoreIcon />
            </button>

            {/* Bottom shadow */}
            <div
                className={`shadow-bottom ${showBottomShadow ? "visible" : ""}`}
                style={{
                    bottom: `${scrollButtonHeight * 2}px`,
                    height: `${shadowHeight}px`
                }}
            />

        </div>
    )
}
