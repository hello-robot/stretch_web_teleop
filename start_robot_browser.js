#!/usr/bin/env node
const { firefox } = require("playwright");
const chalk = require("chalk");
const logId = "start_robot_browser.js";

// You may want to change this to test that the
// robot is using certificates that are valid for its real hostname
let robotHostname = "localhost"; // or NGROK_URL
if (process.argv.length > 2) {
    robotHostname = process.argv[2];
}

const listenConsole = async (page) => {
    // make args accessible
    const describe = (jsHandle) => {
        return jsHandle.evaluate((obj) => {
            // serialize |obj| however you want
            return obj;
            // return `OBJ: ${typeof obj}, ${obj}`;
        }, jsHandle);
    };

    const colors = {
        LOG: chalk.grey, // (text: any) => text,
        ERR: chalk.red,
        WAR: chalk.yellow,
        INF: chalk.cyan,
    };

    // listen to browser console
    page.on("console", async (msg) => {
        const args = await Promise.all(msg.args().map((arg) => describe(arg)));

        // make ability to paint different console[types]
        const type = msg.type().substr(0, 3).toUpperCase();
        const color = colors[type] || chalk.blue;
        let text = "";
        let objs = [];
        for (let i = 0; i < args.length; ++i) {
            if (typeof args[i] !== "object") {
                text += `${args[i]} `;
            } else {
                objs.push(args[i]);
            }
        }
        console.log(color(`CONSOLE.${type}: ${text} `));
        for (let i = 0; i < objs.length; ++i) {
            console.log(objs[i]);
        }
    });
};

(async () => {
    const navigation_timeout_ms = 30000; //30 seconds (default is 30 seconds)
    const min_idle_time = 1000;
    var try_again = true;
    var num_tries = 0;
    var max_tries = 50; // -1 means try forever

    ///////////////////////////////////////////////
    // sleep code from
    // https://stackoverflow.com/questions/951021/what-is-the-javascript-version-of-sleep
    function sleep(ms) {
        return new Promise((resolve) => setTimeout(resolve, ms));
    }
    ///////////////////////////////////////////////

    const browser = await firefox.launch({
        headless: true, // default is true
        handleSIGINT: false,
        defaultViewport: null,
        // NOTE: I (Amal) believe the below args are unnecessary now that we've switched from Chromium to Firefox.
        args: [
            "--use-fake-ui-for-media-stream", //gives permission to access the robot's cameras and microphones (cleaner and simpler than changing the user directory)
            "--disable-features=WebRtcHideLocalIpsWithMdns", // Disables mDNS hostname use in local network P2P discovery. Necessary for enterprise networks that don't forward mDNS traffic
            "--ignore-certificate-errors",
        ],
        firefoxUserPrefs: {
            "permissions.default.microphone": 1, // Give permission to access the robot's microphone
        },
    });

    const context = await browser.newContext({ ignoreHTTPSErrors: true }); // avoid ERR_CERT_COMMON_NAME_INVALID

    const page = await context.newPage();
    await listenConsole(page);

    while (try_again) {
        try {
            await page.goto(`https://${robotHostname}/robot`);
            console.log(logId + ": finished loading");
            try_again = false;
        } catch (e) {
            console.log(logId + ": trying again");
            console.log(e);
            await sleep(200);
            try_again = true;
        }
    }

    console.log(logId + ": start script complete");
    async function closeRobotBrowser() {
        await new Promise((resolve) => {
            page.goto(`https://${robotHostname}/whatever`); // page.close() doesn't work to trigger window.onbeforeunload
            setTimeout(resolve, 500);
        });
    }
    process.on("SIGTERM", () => {
        closeRobotBrowser().finally(() => process.exit(0));
    });
    process.on("SIGINT", () => {
        closeRobotBrowser().finally(() => process.exit(0));
    });
})();
