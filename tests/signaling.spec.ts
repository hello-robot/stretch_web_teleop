import { test, expect } from '@playwright/test';

test('has title', async ({ browser }) => {
    const playwrightContext = await browser.newContext();
    const githubContext = await browser.newContext();
    try {
        const playwrightPage = await playwrightContext.newPage();
        const githubPage = await githubContext.newPage();
        await playwrightPage.goto('https://playwright.dev/');
        await githubPage.goto('https://github.com/');
        await expect(playwrightPage).toHaveTitle(/Playwright/);
        await expect(githubPage).toHaveTitle(/GitHub/);
    } finally {
        await playwrightContext.close()
    }
});
