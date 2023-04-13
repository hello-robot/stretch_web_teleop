const path = require('path');
const HtmlWebpackPlugin = require('html-webpack-plugin');

module.exports = {
  entry: { main: './src/tsx/index.tsx' },
  // externals: {
  //   "react-native": true,
  //   "react": true
  // },
  node: {
    __dirname: false,
  },
  plugins: [new HtmlWebpackPlugin({
    template: './src/html/index.html'
  })],
  module: {
    rules: [
      {
        test: /\.(ts)x?$/,
        use: [
          {
            loader: 'babel-loader',
            options: {
              presets: ['@babel/preset-env', ["@babel/preset-react", {"runtime": "automatic"}], '@babel/preset-typescript', "module:metro-react-native-babel-preset",],
            },
          },
        ],
        exclude: /node_modules/,
      },
      {
        test: /\.css$/i,
        use: ["style-loader", "css-loader"],
      },
    ],
  },
  resolve: {
    extensions: ['.tsx', '.ts', '.js'],
    alias: {
      'react-native': 'react-native-web',
    },
  },
  output: {
    filename: 'bundle.js',
    path: path.resolve(__dirname, 'dist'),
  },
  devServer: {
    static: path.join(__dirname, "dist"),
    compress: true,
    port: 3000,
  },
};