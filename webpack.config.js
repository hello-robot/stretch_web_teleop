const path = require('path');
const HtmlWebpackPlugin = require('html-webpack-plugin');
const webpack = require('webpack')
const dotenv = require('dotenv');

const pages = ['robot', 'operator'];

// call dotenv and it will return an Object with a parsed key 
const env = dotenv.config().parsed;

// reduce it to a nice object, the same as before
const envKeys = Object.keys(env).reduce((prev, next) => {
    prev[`process.env.${next}`] = JSON.stringify(env[next]);
    return prev;
}, {});

module.exports = (env) => {
  envKeys['process.env.storage'] = JSON.stringify(env.storage)
  console.log(envKeys)

  return {
  mode: 'development',
    entry: pages.reduce((config, page) => {
        config[page] = `./src/pages/${page}/tsx/index.tsx`;
        return config;
    }, {}),
    output: {
      filename: "[name]/bundle.js",
      path: path.resolve(__dirname, "dist"),
    },
    optimization: {
        splitChunks: {
            chunks: "all",
        },
    },
  // node: {
  //   __dirname: false,
  // },
  plugins: [
    // Work around for Buffer is undefined:
    // https://github.com/webpack/changelog-v5/issues/10
    new webpack.ProvidePlugin({
      Buffer: ['buffer', 'Buffer'],
    }),
    // new webpack.ProvidePlugin({
    //   process: 'process/browser',
    // }),
    new webpack.DefinePlugin(envKeys),
  ].concat(
    pages.map(
      (page) =>
        new HtmlWebpackPlugin({
          inject: true,
          template: `./src/pages/${page}/html/index.html`,
          filename: `${page}/index.html`,
          chunks: [page],
      })
    )
  ),
  module: {
    rules: [
      {
        test: /\.(ts)x?$/,
        use: [
          {
            loader: 'babel-loader',
            options: {
              presets: [
                ['@babel/preset-env', {'loose': true}], 
                ["@babel/preset-react", {"runtime": "automatic"}], 
                '@babel/preset-typescript'
              ],
              plugins: [
                '@babel/plugin-transform-runtime'
              ]
            },
          },
        ],
        exclude: /node_modules/,
      },
      {
        test: /\.css$/i,
        use: ["style-loader", "css-loader"],
      },
      {
        test: /\.(jpe?g|png|gif|svg)$/i,
        use: 'file-loader'
      }
    ],
  },
  externals: {
    'express': 'commonjs express'
  },
  resolve: {
    extensions: ['.tsx', '.js'],
    alias: {
      "shared": path.resolve(__dirname, './src/shared/'),
      "operator": path.resolve(__dirname, './src/pages/operator/'),
      "robot": path.resolve(__dirname, './src/pages/robot/'),
    }
  },
  watch: true,
  }
};
