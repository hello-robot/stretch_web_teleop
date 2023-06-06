const path = require('path');

module.exports = {
    propsParser: require('react-docgen-typescript').withCustomConfig(
        './tsconfig.json'
    ).parse,
    // mountPointId: 'layoutcomponents',
    // ignore: ['./src/pages/operator/tsx/index.tsx'],
    sections: [
        {
            name: "Layout Components",
            components: 'src/pages/operator/tsx/layoutcomponents/*.tsx',
        },
        {
            name: "Static Components",
            components: 'src/pages/operator/tsx/staticcomponents/*.tsx',
        },
        {
            name: "Function Provider",
            components: 'src/pages/operator/tsx/functionprovider/*.tsx',
        }
    ],
    require: [path.resolve(__dirname, 'styleguide/setup.js')],
};
