/**Resolution of SVG components */
export const SVG_RESOLUTION = 500;

/**
 * Takes a percentage value and returns the real pixel location on 
 * the SVG
 * @param percentage value between 0 and 100
 * @returns the pixel location
 * @example 0 -> 0, 50 -> resolution/2, 100 -> resolution
 */
export function percent2Pixel(percentage: number) {
    return SVG_RESOLUTION / 100 * percentage;
}

/**
 * Position and dimensions of the robot base from the overhead camera view
 */
export const OVERHEAD_ROBOT_BASE = {
    centerX: percent2Pixel(50),
    centerY: percent2Pixel(70),
    height: percent2Pixel(10),
    width: percent2Pixel(10)
}

/**Creates the SVG path for a rectangle
 * @param x left edge location
 * @param y top edge location
 * @param width the width
 * @param height the height
*/
export function rect(x: number, y: number, width: number, height: number) {
    return `M ${x} ${y} ${x + width} ${y} ${x + width} ${y + height} 
                ${x} ${y + height} Z`;
}