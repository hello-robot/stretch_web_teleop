// Example usage:
// hexToRgbArray('#ff8800') // [255, 136, 0]

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
