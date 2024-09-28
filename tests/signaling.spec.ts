import { test } from '@playwright/test';
import { exec, execSync } from 'child_process';
import { LocalSignaling } from 'shared/signaling/LocalSignaling';

test.describe('local signaling', () => {
    let server_process;

    test.beforeAll(async () => {
        server_process = exec('node server.js');
        execSync('sleep 1');
    });

    test.afterAll(async () => {
        server_process.kill();
    });

    test('whatever', async ({ browser }) => {
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

            await robotPage.goto('https://localhost/test/');
            await robotPage.evaluate(() => {
                // function importFromWindow(imports: string[]) {
                //     let ctx;
                //     for (const i in imports) {
                //         eval(`ctx['${i}'] = window.${i}`);
                //         eval(`${i} = ctx['${i}'];`);
                //     }
                // }
                // importFromWindow(['LocalSignaling']);

                let signaler = new window.LocalSignaling({
                    onSignal: (msg) => { console.log(msg) },
                    onGoodbye: () => {},
                });
                console.log('hey');
            });

            execSync('sleep 2');
        } finally {
            await robotContext.close();
            await operatorContext.close();
        }
    });
});
