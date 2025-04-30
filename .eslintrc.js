module.exports = {
    root: true,
      parserOptions: {
        ecmaVersion: 2020,
      sourceType: "module",
      ecmaFeatures: {
        jsx: true,
      },
    },
      settings: {
        react: {
        version: "detect",
      },
    },
      env: {
        jest: true,
      browser: true,
      amd: true,
      node: true,
    },
      extends: [
        "eslint:recommended",
      "react-app",
      "plugin:react/recommended",
      "plugin:react-hooks/recommended",
      "plugin:prettier/recommended",
    ],
      rules: {
        "prettier/prettier": ["error", {}, { usePrettierrc: true }],
    },
};
