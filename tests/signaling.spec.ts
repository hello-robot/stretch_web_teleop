import { test } from '@playwright/test';
import { exec, execSync } from 'child_process';

test.describe('local signaling', () => {
    let server_process;

    test.beforeAll(async () => {
        server_process = exec('node server.js');
        execSync('sleep 1');
    });

    test.afterAll(async () => {
        server_process.kill();
    });

    test('do browser pages connect', async ({ browser }) => {
        const robotContext = await browser.newContext();
        const operatorContext = await browser.newContext();
        try {
            const robotPage = await robotContext.newPage();
            const operatorPage = await operatorContext.newPage();

            robotPage.on('console', (msg) => {
                console.debug('Robot browser said:', msg.text());
            });
            operatorPage.on('console', (msg) => {
                console.debug('Operator browser said:', msg);
            });

            await robotPage.goto('https://localhost/robot/');
            await operatorPage.goto('https://localhost/operator/');

            execSync('sleep 2');
        } finally {
            await robotContext.close();
            await operatorContext.close();
        }
    });
});
