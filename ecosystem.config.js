module.exports = {
  apps: [
    {
      name: "server",
      script: "./server.js",
      watch: true,
      ignore_watch: ["rosbags", "node_modules", "logs"]
    },
    {
      name: "start_robot_browser",
      script: "./start_robot_browser.js",
      watch: true,
      ignore_watch: ["rosbags", "node_modules", "logs"]
    }
  ]
}; 