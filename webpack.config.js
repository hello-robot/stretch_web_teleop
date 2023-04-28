const path = require('path');
const HtmlWebpackPlugin = require('html-webpack-plugin');

module.exports = {
  entry: { main: './src/tsx/index.tsx' },
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
              presets: [
                ['@babel/preset-env', {'loose': true}], 
                ["@babel/preset-react", {"runtime": "automatic"}], '@babel/preset-typescript'],
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
  },
  output: {
    filename: 'bundle.js',
    path: path.resolve(__dirname, 'dist'),
  },
  devServer: {
    allowedHosts: ['slinky.hcrlab.cs.washington.edu'],
    static: path.join(__dirname, "dist"),
    compress: true,
    port: 3000,
  },
};
