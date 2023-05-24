const path = require('path');
const HtmlWebpackPlugin = require('html-webpack-plugin');
const { CleanWebpackPlugin } = require('clean-webpack-plugin');
const webpack = require('webpack')

const pages = ['robot', 'operator'];

module.exports = {
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
    new CleanWebpackPlugin(),
    // Work around for Buffer is undefined:
    // https://github.com/webpack/changelog-v5/issues/10
    new webpack.ProvidePlugin({
      Buffer: ['buffer', 'Buffer'],
    }),
    new webpack.ProvidePlugin({
      process: 'process/browser',
    }),
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
};
