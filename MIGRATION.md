## Migrating to ROS 2

The `/tests` folder contains a set of tests demoing the use of `roslib.js`. The initial goal is to replicate the same test behavior in ROS 2. 

The Jest unit test framework is used to execute each individual unit tests. However, since we are using TypeScript for Raect (*.tsx files), we need to first transpile TypeScript code to JavaScript. To do this, we use the `ts-jest` node package. The configuration indicating Jest to first transpile code before executing tests, is present within `jest.config.js`.

Now, since we need to import `roslib` within our tests, we need to transpile ES6 JavaScript code to be backwards compatible with vanilla JavaScript. For this, we use Babel. Jest automatically detects the `babel.config.js` file.

Refer to [https://www.testim.io/blog/typescript-unit-testing-101](https://www.testim.io/blog/typescript-unit-testing-101) for more details.


