#!/usr/bin/env node
const puppeteer = require('puppeteer');
// const { chromium } = require('playwright');
const logId = 'start_robot_browser.js';

// You may want to change this to test that the
// robot is using certificates that are valid for its real hostname
let robotHostname = "localhost" // or NGROK_URL
if (process.argv.length > 2) {
	robotHostname = process.argv[2]
}

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
		return new Promise(resolve => setTimeout(resolve, ms));
	}
	///////////////////////////////////////////////

	// const browser = await chromium.launch({
	// 	headless: true, // default is true
	// 	ignoreHTTPSErrors: true, // avoid ERR_CERT_COMMON_NAME_INVALID
	// 	defaultViewport: null,
	// 	args: ['--use-fake-ui-for-media-stream', //gives permission to access the robot's cameras and microphones (cleaner and simpler than changing the user directory)
	// 		'--disable-features=WebRtcHideLocalIpsWithMdns', // Disables mDNS hostname use in local network P2P discovery. Necessary for enterprise networks that don't forward mDNS traffic
    //         '--ignore-certificate-errors']
    // });
	const browser = await puppeteer.launch({
		headless: true, // default is true
		ignoreHTTPSErrors: true, // avoid ERR_CERT_COMMON_NAME_INVALID
			args: ['--use-fake-ui-for-media-stream', //gives permission to access the robot's cameras and microphones (cleaner and simpler than changing the user directory)
			'--disable-features=WebRtcHideLocalIpsWithMdns', // Disables mDNS hostname use in local network P2P discovery. Necessary for enterprise networks that don't forward mDNS traffic
            '--ignore-certificate-errors']});

	const page = await browser.newPage();
	
	while (try_again) {
		try {
			await page.goto(`https://${robotHostname}/robot`);
			console.log(logId + ': finished loading');
			try_again = false;
		} catch (e) {
			console.log(logId + ': trying again');
			console.log(e);
			await sleep(200);
			try_again = true;
		}
	}

	console.log(logId + ': start script complete');
})();
