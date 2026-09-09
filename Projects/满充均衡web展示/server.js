const http = require('http');
const { execSync } = require('child_process');
const url = require('url');
const fs = require('fs');
const path = require('path');

const PORT = 3000;

const server = http.createServer((req, res) => {
    // 设置CORS头
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type');

    if (req.method === 'OPTIONS') {
        res.writeHead(200);
        res.end();
        return;
    }

    const parsedUrl = url.parse(req.url, true);
    const pathname = parsedUrl.pathname;

    if (pathname === '/calculate') {
        const voltMax = parseInt(parsedUrl.query.voltMax) || 3450;
        const voltDiff = parseInt(parsedUrl.query.voltDiff) || 20;

        try {
            // 调用C程序
            const result = execSync(`balance_calc.exe ${voltMax} ${voltDiff}`, {
                cwd: __dirname,
                encoding: 'utf8'
            });

            // 解析JSON输出
            const jsonResult = JSON.parse(result);

            res.writeHead(200, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify(jsonResult));
        } catch (error) {
            res.writeHead(500, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ error: error.message }));
        }
    } else if (pathname === '/' || pathname === '/index_with_c.html') {
        // 提供HTML文件
        const filePath = path.join(__dirname, 'index_with_c.html');
        fs.readFile(filePath, 'utf8', (err, data) => {
            if (err) {
                res.writeHead(404);
                res.end('File not found');
                return;
            }
            res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
            res.end(data);
        });
    } else {
        res.writeHead(404);
        res.end('Not found');
    }
});

server.listen(PORT, () => {
    console.log(`服务器运行在 http://localhost:${PORT}`);
    console.log(`请在浏览器中打开: http://localhost:${PORT}`);
    console.log('按 Ctrl+C 停止服务器');
});
