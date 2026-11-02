const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const crypto=require('node:crypto');
const {chromium}=require('C:/Users/ucawa/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'../..');
const out=path.join(root,'analysis/normalized_yaw_20261031');
const source='C:/Users/ucawa/.codex/visualizations/2026/11/02/01a19c81-ea32-77d2-b235-88c8c166eb65/normalized-yaw-feedback.html';
const fragment=fs.readFileSync(source,'utf8');
for(const m of fragment.matchAll(/<script>([\s\S]*?)<\/script>/g))new vm.Script(m[1]);
const expected=JSON.parse(fs.readFileSync(path.join(out,'chart_data.json'),'utf8'));
const embedded=JSON.parse(fragment.match(/id="normalized-yaw-input">([\s\S]*?)<\/script>/)[1]);
if(JSON.stringify(embedded)!==JSON.stringify(expected))throw new Error('Chart evidence mismatch');
if(Buffer.byteLength(fragment)>=1000000)throw new Error('Fragment over size limit');
const executable=[chromium.executablePath(),'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'].find(fs.existsSync);
if(!executable)throw new Error('No headless browser available');
(async()=>{
  const browser=await chromium.launch({headless:true,executablePath:executable});
  const page=await browser.newPage();
  const errors=[];
  page.on('pageerror',e=>errors.push(String(e)));
  const results=[];
  try{
    for(const width of [736,360]){
      await page.setViewportSize({width,height:900});
      // プレビューだけのテーマ。配布fragmentはホストの意味色を使う。
      await page.setContent(`<style>:root{--font-size-base:14px;--foreground:#252525;--border:#c8c8c8;--popover:#f1f1f1;--popover-foreground:#252525;--viz-series-1:#2570a6;--viz-series-2:#b05b28;--viz-series-3:#78783c}body{margin:12px;font:14px sans-serif}h3{margin:0;font-size:14px;font-weight:500}select{font:inherit;max-width:100%}</style>${fragment}`,{waitUntil:'networkidle'});
      await page.waitForFunction(()=>document.querySelectorAll('rect[data-chart-frame]').length===2);
      for(const number of ['14074','14036','14034','14037']){
        await page.locator('select').selectOption(number);
        const check=await page.evaluate(()=>({svg:document.querySelectorAll('svg').length,
          paths:document.querySelectorAll('path[data-series]').length,
          overflow:document.documentElement.scrollWidth>innerWidth,
          textOverflow:[...document.querySelectorAll('svg text')].filter(e=>{const a=e.getBoundingClientRect(),b=e.ownerSVGElement.getBoundingClientRect();return a.left<b.left-1||a.right>b.right+1||a.top<b.top-1||a.bottom>b.bottom+1;}).map(e=>e.textContent),
          allPathsInside:[...document.querySelectorAll('path[data-series]')].every(e=>{const p=e.getBBox(),r=e.ownerSVGElement.querySelector('rect[data-chart-frame]').getBBox();return p.x>=r.x-1&&p.y>=r.y-1&&p.x+p.width<=r.x+r.width+1&&p.y+p.height<=r.y+r.height+1;})}));
        if(check.svg!==2||check.paths!==5||check.overflow||check.textOverflow.length||!check.allPathsInside)throw new Error(JSON.stringify({width,number,...check}));
        results.push({width,number,...check});
      }
      await page.locator('select').selectOption('14036');
      const first=page.locator('.ny-legend[data-panel="yaw"] button').first();
      await first.click();
      if(await page.locator('path[data-series]').count()!==4)throw new Error('Legend toggle failed');
      await first.click();
      const hit=await page.locator('[data-chart-hover-overlay]').first().boundingBox();
      await page.mouse.move(hit.x+hit.width*.45,hit.y+hit.height*.5);
      if(!(await page.locator('[role="tooltip"]').isVisible()))throw new Error('Tooltip hidden');
      const content=await page.locator('[role="tooltip"]').innerText();
      if(!content.includes('14036')||!content.includes('目標角速度')||!content.includes('実角速度'))throw new Error('Cross-series tooltip incomplete');
      await page.mouse.move(0,0);
      await page.screenshot({path:path.join(out,`chart-${width}.png`),fullPage:true});
    }
    if(errors.length)throw new Error(errors.join('\n'));
    fs.writeFileSync(path.join(out,'chart_validation.json'),JSON.stringify({source,sha256:crypto.createHash('sha256').update(fragment).digest('hex'),embeddedDataMatches:true,selector:true,legendToggle:true,crossSeriesTooltip:true,browserErrors:errors,results},null,2)+'\n');
    console.log('PASS: embedded data, 4 logs x 2 widths, paths, labels, selector, legend and tooltip');
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
