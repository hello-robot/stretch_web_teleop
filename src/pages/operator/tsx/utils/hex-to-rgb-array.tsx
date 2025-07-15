/**
 * Converts a hex color string to an RGB array.
 * 
 * @param {string} hex - The hex color string (e.g., '#ff8800' or 'ff8800').
 * @returns {[number, number, number]} An array containing the RGB values.
 * @throws {Error} If the hex color is invalid.
 * 
 * Usage:
 * const myColor = hexToRgbArray('#ffffff'); // Returns [255, 255, 255]
 */

export default function hexToRgbArray(hex: string): [number, number, number] {
    // Remove leading '#' if present
    hex = hex.replace(/^#/, '');

    // Handle shorthand hex (e.g., #abc)
    if (hex.length === 3) {
        hex = hex.split('').map(c => c + c).join('');
    }

    if (hex.length !== 6) {
        throw new Error('Invalid hex color');
    }

    const num = parseInt(hex, 16);
    return [
        (num >> 16) & 255, // Red
        (num >> 8) & 255,  // Green
        num & 255          // Blue
    ];
}
