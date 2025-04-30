import { defineConfig } from "eslint/config";
import { fixupConfigRules } from "@eslint/compat";
import globals from "globals";
import path from "node:path";
import { fileURLToPath } from "node:url";
import js from "@eslint/js";
import { FlatCompat } from "@eslint/eslintrc";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const compat = new FlatCompat({
    baseDirectory: __dirname,
    recommendedConfig: js.configs.recommended,
    allConfig: js.configs.all,
});

export default defineConfig([
    {
        extends: fixupConfigRules(
            compat.extends(
                "eslint:recommended",
                "react-app",
                "plugin:react/recommended",
                "plugin:react-hooks/recommended",
                "plugin:prettier/recommended"
            )
        ),

        languageOptions: {
            globals: {
                ...globals.jest,
                ...globals.browser,
                ...globals.amd,
                ...globals.node,
            },

            ecmaVersion: 2020,
            sourceType: "module",

            parserOptions: {
                ecmaFeatures: {
                    jsx: true,
                },
            },
        },

        settings: {
            react: {
                version: "detect",
            },
        },

        rules: {
            "prettier/prettier": [
                "error",
                {},
                {
                    usePrettierrc: true,
                },
            ],
            eqeqeq: "off",
            // allow double-quotes (and auto-fix to double)
            quotes: [
                "error",
                "double",
                {
                    avoidEscape: true, // still let ya use single if you gotta avoid escape
                    allowTemplateLiterals: true, // keep backticks workin’ for templates
                },
            ],
        },
    },
]);
