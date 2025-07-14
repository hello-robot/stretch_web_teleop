// Example usage:
// genUUID() => "41g2c3y"

const genUUID = (): string => {
    // Generates a random 7-character alphanumeric string.
    return Math.random().toString(36).substring(2, 9);
};

export default genUUID;