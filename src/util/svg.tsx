/**Resolution of SVG components */
export const SVG_RESOLUTION = 300;

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
    centerX: percent2Pixel(56),
    centerY: percent2Pixel(55),
    height: percent2Pixel(11),
    width: percent2Pixel(15)
}