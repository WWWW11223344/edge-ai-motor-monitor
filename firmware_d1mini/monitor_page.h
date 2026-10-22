/* ==========================================================================
 *  monitor_page.h  --  手机看板网页（一整个 HTML 页面，直接编译进固件里）
 *
 *  【这是什么】一个完整的单页网页：手机连上 D1 的热点 / 同一 WiFi，
 *    浏览器打开 http://192.168.157.79 就能看到电机的实时数据。
 *
 *  【三个设计约束，决定了它长这样】
 *    ① 全部自带：CSS 和 JS 都写在页面里，不引用任何外部网址。
 *       现场没有外网也能正常打开（这是嵌入式页面最重要的一条）。
 *    ② 放在 Flash 里：数组用 PROGMEM 声明，不占宝贵的 RAM。
 *       页面内容用「原生字符串字面量」(raw string) 包起来，
 *       这样里面的引号、反斜杠都不用转义，写起来和普通 HTML 一样。
 *    ③ 只读：页面只做 GET /status，没有任何会改数据的按钮。
 *
 *  【页面结构（2026-10-21 重做，v2）】
 *    ┌ 顶部        标题 + 链路状态胶囊（绿=通信正常，红=中断）
 *    ├ 缺路横幅    只有真缺路时才出现，逐路点名（电流 / 转速 / 母线电压）
 *    ├ 判定主卡    正常 / 学习中 / 异常 / 待机 / 未启用 / 推理故障，六态分色渐变
 *    │   └ 主卡内嵌两个核心量：转速、特征比值 R（AI 的真正输入）
 *    ├ 趋势卡      近 60 秒转速与 R 的双曲线（Canvas 现画，无外部依赖）
 *    ├ 电气清单    电流 / 母线电压 / 档位与占空比，紧凑三行
 *    └ 底部        数据年龄、有效帧 / 异常帧、固件版本
 *
 *  【v2 相对 v1 改了什么】
 *    1. 信息分层：原来六格卡片等权重平铺，转速与 R 淹没在中间。v2 把它们
 *       提到判定主卡内部做主视觉，电流/电压/档位降为清单行。
 *    2. 加了趋势：每秒一次 /status，页内自存最近 60 点，用 Canvas 画双曲线。
 *       曲线用「各自的极值」独立缩放，不做双 Y 轴（小屏上双轴刻度读不出来）。
 *       无效值（--）在曲线上断开，不补 0，避免把「没采到」画成「测到 0」。
 *    3. 缺路提示从底部小灰字改成顶部琥珀色横幅 —— 这条是诊断信息，不是备注，
 *       位置必须比备注显眼。
 *    4. 主卡背景改为按状态渐变（绿/琥珀/红），异常时保留呼吸动画。
 *    5. 适配 iPhone 底部安全区（env(safe-area-inset-bottom)），
 *       横条不再压住固件版本行。
 *
 *  【为什么没有加速度了】
 *    MPU6050 已于 2026-10-19 从 STM32 固件摘除（HARDWARE/_removed_mpu6050_20261019），
 *    帧里 accel_x/y/z 三路的有效位恒为 0，永远只能显示「--」。
 *    页面直接不再展示，改展示「特征比值 R」——那才是 AI 真正的输入。
 *
 *  【dR 是怎么来的（2026-10-21）】
 *    42 字节帧的 6 个槽位里没有 dR，但它不必由 STM32 发 —— D1 侧就能算：
 *    D1 有 256 帧环形缓冲（约 25 秒），取最近 32 帧（100 ms 一帧 = 3.2 s），
 *    算 q = rpm / vbat 的峰峰值，与板上 feature_r_ripple_abs() 同一口径
 *    （板上那个量化简后正好等于窗内 q 的峰峰值，因为窗内占空比恒定）。
 *    这样 STM32 固件一个字节都不用改、协议版本也不用动，
 *    代价只是 D1 每拼一次 JSON 多扫 32 帧（微秒级）。
 *    固件给的是放大 1000 倍的整数 dr_x1000，页面除回来再显示两位小数，
 *    与 OLED 上轮显的 dR 是同一个数、同一个单位。
 *  【为什么不显示相似度】相似度属板内中间量，5.2 节第 3 条约定
 *    「对外只发结论、不发中间量」；帧里只有 AI 阶段与报警标志，没有原值。
 *    所以 OLED 底条上的 S:100 在页面上是看不到的，这是有意为之，不是漏了。
 *
 *  【编码铁律】本文件必须保存为 UTF-8（无 BOM），页面里已声明 charset=utf-8。
 *    一旦被编辑器改存成 GBK，手机上打开中文会全部乱码。
 *
 *  【数据怎么来的】页内 JS 每秒 fetch('/status') 一次（带 2.2 秒超时），
 *    把返回的 JSON 填到各个元素里；拿到 null 就显示「--」。
 * ========================================================================== */
#ifndef MONITOR_PAGE_H /* 重复包含保护：首次进来时该宏尚未定义 */
#define MONITOR_PAGE_H /* 定义包含标记，之后再 #include 本文件时上面那条 #ifndef 即为假 */
/* 【这行是整个页面的容器】
 *   static const char MONITOR_PAGE[]   页面字节数组
 *   PROGMEM                            放到 Flash 而不是 RAM
 *   R"HTML( ... )HTML"                 「原生字符串字面量」，原样保留、不做转义
 * 【注意】修改页面时不要动开头和结尾的起止标记，
 * 也不要让页面正文里出现「右括号 + HTML + 双引号」这一串字符（那会提前结束字符串）。
 * 【页面正文起点】从下面这行的 R"HTML( 到文件末尾的 )HTML" 之间是页面正文
 *  （HTML + CSS + JavaScript），属于字符串数据而不是 C++ 代码，
 *  因此页面内部的说明一律用 HTML / CSS / JS 各自的注释语法书写。 */
static const char MONITOR_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#f4f6f9" media="(prefers-color-scheme:light)">
<meta name="theme-color" content="#0a0c10" media="(prefers-color-scheme:dark)">
<title>电机状态监测</title><style>
/* ---------- 配色：浅色 / 深色各一套，手机跟随系统自动切换 ---------- */
:root{
  --bg:#f4f6f9; --card:#ffffff; --card2:#f8fafc; --ink:#0f172a; --muted:#64748b;
  --line:rgba(15,23,42,.08);
  --ok:#059669;  --ok-bg:linear-gradient(155deg,#ecfdf5 0%,#d7f7e9 100%);
  --warn:#b45309; --warn-bg:linear-gradient(155deg,#fffbeb 0%,#fdf0cf 100%);
  --bad:#dc2626; --bad-bg:linear-gradient(155deg,#fef2f2 0%,#fde0dd 100%);
  --idle:#64748b; --idle-bg:#f1f5f9;
  --spd:#0284c7; --rr:#7c3aed;
}
@media (prefers-color-scheme:dark){
  :root{
    --bg:#0a0c10; --card:#14181f; --card2:#1a1f28; --ink:#eef2f7; --muted:#8b95a5;
    --line:rgba(255,255,255,.075);
    --ok:#34d399;  --ok-bg:linear-gradient(155deg,#0c2a21 0%,#103a2d 100%);
    --warn:#fbbf24; --warn-bg:linear-gradient(155deg,#2a2110 0%,#3a2c12 100%);
    --bad:#f87171; --bad-bg:linear-gradient(155deg,#2c1414 0%,#3d1a18 100%);
    --idle:#8b95a5; --idle-bg:#1a1f28;
    --spd:#38bdf8; --rr:#a78bfa;
  }
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--ink);-webkit-text-size-adjust:100%;
  font:16px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif}
main{max-width:560px;margin:0 auto;
  padding:14px 16px calc(26px + env(safe-area-inset-bottom))}

/* ---------- 顶部：品牌点 + 标题 + 链路胶囊 ---------- */
header{display:flex;align-items:center;justify-content:space-between;gap:10px;margin:2px 2px 14px}
.brand{display:flex;align-items:center;gap:9px;min-width:0}
.brand i{width:9px;height:9px;border-radius:50%;background:var(--muted);flex:none;
  box-shadow:0 0 0 3px rgba(148,163,184,.16);transition:background .25s,box-shadow .25s}
body[data-link=live] .brand i{background:var(--ok);box-shadow:0 0 0 3px rgba(52,211,153,.18)}
body[data-link=stale] .brand i,body[data-link=offline] .brand i{background:var(--bad);
  box-shadow:0 0 0 3px rgba(248,113,113,.18)}
h1{margin:0;font-size:17px;font-weight:700;letter-spacing:.01em;white-space:nowrap;
  overflow:hidden;text-overflow:ellipsis}
#pill{display:inline-flex;align-items:center;gap:6px;flex:none;padding:6px 11px;
  border-radius:999px;background:var(--idle-bg);color:var(--idle);font-size:12.5px;font-weight:650;
  transition:background .25s,color .25s;white-space:nowrap}
#pill i{width:7px;height:7px;border-radius:50%;background:currentColor;flex:none}
body[data-link=live] #pill{background:rgba(52,211,153,.14);color:var(--ok)}
body[data-link=stale] #pill,body[data-link=offline] #pill{background:rgba(248,113,113,.14);color:var(--bad)}

/* ---------- 缺路横幅：只有非空才占位 ---------- */
.banner{margin:0 0 12px;padding:11px 14px;border-radius:14px;background:var(--warn-bg);
  border:1px solid rgba(180,83,9,.22);color:var(--warn);font-size:13px;font-weight:650;line-height:1.45}
.banner:empty{display:none}
@media (prefers-color-scheme:dark){.banner{border-color:rgba(251,191,36,.22)}}

/* ---------- 判定主卡 ---------- */
.hero{position:relative;background:var(--card);border:1px solid var(--line);
  border-radius:22px;padding:19px 21px 20px;transition:background .35s,border-color .35s;
  overflow:hidden}
.hero::after{content:"";position:absolute;inset:0;border-radius:22px;pointer-events:none;
  box-shadow:inset 0 1px 0 rgba(255,255,255,.07)}
body[data-state=ok] .hero{background:var(--ok-bg);border-color:transparent}
body[data-state=learn] .hero{background:var(--warn-bg);border-color:transparent}
body[data-state=alarm] .hero{background:var(--bad-bg);border-color:transparent;
  animation:breathe 1.6s ease-in-out infinite}
@keyframes breathe{0%,100%{opacity:1}55%{opacity:.7}}
.cap{font-size:11px;font-weight:750;letter-spacing:.14em;color:var(--muted)}
#stateTitle{margin:7px 0 3px;font-size:34px;font-weight:800;letter-spacing:-.015em;
  line-height:1.15;transition:color .35s}
#stateSub{margin:0;font-size:13px;color:var(--muted);min-height:19px}
body[data-state=ok] #stateTitle{color:var(--ok)}
body[data-state=learn] #stateTitle{color:var(--warn)}
body[data-state=alarm] #stateTitle{color:var(--bad)}
/* 主卡内嵌的两个核心量 */
.hero-stats{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-top:18px;padding-top:16px;
  border-top:1px solid var(--line)}
.hs .k{display:block;font-size:11.5px;font-weight:650;color:var(--muted);letter-spacing:.05em;
  margin-bottom:2px}
.hs .v{display:block;white-space:nowrap;font-variant-numeric:tabular-nums}
.hs .v b{font-size:29px;font-weight:780;letter-spacing:-.02em}
.hs .v i{font-size:12px;font-style:normal;font-weight:650;color:var(--muted);margin-left:4px}

/* ---------- 主卡内的判定量 dR：跳变幅度 + 相对门限的进度条 ----------
 * dR 是真正下判决的那个量（阈值 1.0 rpm/V），不是备注，
 * 所以给它单独一行 + 一根条：条满了就报警，看一眼知道离报警还有多远。 */
.judge{margin-top:16px;padding-top:15px;border-top:1px solid var(--line)}
.judge-head{display:flex;align-items:baseline;justify-content:space-between;gap:10px}
.judge-head .k{font-size:11.5px;font-weight:650;color:var(--muted);letter-spacing:.05em;white-space:nowrap}
.judge-head .v{white-space:nowrap;font-variant-numeric:tabular-nums}
.judge-head .v b{font-size:24px;font-weight:780;letter-spacing:-.01em}
.judge-head .v i{font-size:12px;font-style:normal;font-weight:650;color:var(--muted);margin-left:4px}
.judge-head .v em{font-size:11.5px;font-style:normal;font-weight:650;color:var(--muted);
  margin-left:10px;opacity:.85}
.bar{margin-top:9px;height:6px;border-radius:999px;background:rgba(148,163,184,.18);overflow:hidden}
.bar span{display:block;height:100%;width:0;border-radius:999px;background:rgba(148,163,184,.35);
  transition:width .3s ease,background .3s ease}

/* ---------- 趋势卡 ---------- */
.card{background:var(--card);border:1px solid var(--line);border-radius:20px;
  padding:15px 18px 13px;margin-top:12px}
.chart-head{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:4px}
.legend{display:flex;align-items:center;gap:5px;font-size:11.5px;font-weight:650;color:var(--muted)}
.legend i{width:9px;height:3px;border-radius:2px;display:inline-block}
.lg-spd{background:var(--spd)}
.lg-r{background:var(--rr)}
/* 两条曲线分开画：R 与转速在「同档同压」下严格成正比，共用一张图会完全重合，
 * 紫色把蓝色整个盖住，看起来像只有一条线。拆成上下两格，各占一半高度。 */
.plot-row{display:flex;align-items:center;gap:10px;margin-top:9px}
.plot-lab{width:44px;flex:none;display:flex;align-items:center;gap:5px;
  font-size:11.5px;font-weight:650;color:var(--muted);white-space:nowrap}
.plot-lab i{width:9px;height:3px;border-radius:2px;flex:none;display:inline-block}
.plot-row canvas{flex:1 1 auto;min-width:0;width:100%;height:54px;display:block}
.chart-foot{margin-top:6px;font-size:11.5px;color:var(--muted);font-weight:600;
  display:flex;justify-content:space-between;font-variant-numeric:tabular-nums}
.chart-foot span{white-space:nowrap}

/* ---------- 电气清单 ---------- */
.list{margin-top:12px;background:var(--card);border:1px solid var(--line);
  border-radius:20px;padding:2px 18px}
.row{display:flex;align-items:center;justify-content:space-between;gap:12px;
  padding:13px 0;border-bottom:1px solid var(--line)}
.row:last-child{border-bottom:none}
.row .k{font-size:13.5px;font-weight:650;color:var(--muted)}
.row .v{white-space:nowrap;font-variant-numeric:tabular-nums}
.row .v b{font-size:20px;font-weight:720;letter-spacing:-.01em}
.row .v i{font-size:12px;font-style:normal;font-weight:650;color:var(--muted);margin:0 4px}

/* ---------- 底部 ---------- */
footer{margin-top:14px;padding:0 3px;font-size:12.5px;color:var(--muted);line-height:1.9}
footer div{display:flex;justify-content:space-between;gap:12px}
footer b{font-weight:650;font-variant-numeric:tabular-nums;color:var(--ink)}
#ver{margin-top:4px;opacity:.7;font-size:11.5px}
</style></head><body data-link="waiting" data-state="idle"><main>

<header>
  <div class="brand"><i></i><h1>电机状态监测</h1></div>
  <div id="pill"><i></i><span id="linkText">等待数据</span></div>
</header>

<!-- 缺路横幅：逐路点名，只在真缺路时由 JS 写入文字，空字符串时靠 CSS 隐藏 -->
<div id="cond" class="banner" role="status"></div>

<!-- 判定主卡：配色由 body[data-state] 驱动；转速与 R 提到这里做主视觉 -->
<section class="hero">
  <div class="cap">运行判定</div>
  <h2 id="stateTitle">--</h2>
  <p id="stateSub"></p>
  <div class="hero-stats">
    <div class="hs"><span class="k">转速</span>
      <span class="v"><b id="rpm">--</b><i>rpm</i></span></div>
    <div class="hs"><span class="k">特征比值 R</span>
      <span class="v"><b id="ratio">--</b><i>rpm/V</i></span></div>
  </div>
  <!-- dR：与板上 OLED 同一口径的跳变幅度（窗内 rpm/V 峰峰值），阈值 1.0 -->
  <div class="judge">
    <div class="judge-head">
      <span class="k">跳变幅度 dR</span>
      <span class="v"><b id="dr">--</b><i>rpm/V</i><em>门限 1.0</em></span>
    </div>
    <div class="bar"><span id="drBar"></span></div>
  </div>
</section>

<!-- 趋势卡：转速与 R 分两格画，各自按自己这 60 点的极值缩放；无效值断开不补零 -->
<section class="card">
  <div class="chart-head">
    <span class="cap">近 60 秒趋势</span>
  </div>
  <div class="plot-row">
    <span class="plot-lab"><i class="lg-spd"></i>转速</span>
    <canvas id="chartSpd"></canvas>
  </div>
  <div class="plot-row">
    <span class="plot-lab"><i class="lg-r"></i>R</span>
    <canvas id="chartR"></canvas>
  </div>
  <div class="chart-foot">
    <span>转速 <b id="spdMin">--</b> ~ <b id="spdMax">--</b> rpm</span>
    <span>R <b id="rMin">--</b> ~ <b id="rMax">--</b></span>
  </div>
</section>

<!-- 电气清单：三行紧凑排布 -->
<section class="list">
  <div class="row"><span class="k">电流</span>
    <span class="v"><b id="current">--</b><i>A</i></span></div>
  <div class="row"><span class="k">母线电压</span>
    <span class="v"><b id="bus">--</b><i>V</i></span></div>
  <div class="row"><span class="k">档位 / 占空比</span>
    <span class="v"><b id="level">--</b><i id="gearDot"></i><b id="duty"></b><i id="gearPct"></i></span></div>
</section>

<footer>
  <div><span>已运行</span><span><b id="uptime">--</b> s</span></div>
  <div><span>数据年龄</span><span><b id="age">--</b> s</span></div>
  <div><span>有效帧 / 异常帧</span><span><b id="frames">0</b> / <b id="rejected">0</b></span></div>
  <div id="ver"><span id="version">d1-telemetry-2.1.11</span></div>
</footer>
</main><script>
'use strict';
var el=function(id){return document.getElementById(id);};
/* 链路状态的文字：对应 /status 的 link 字段 */
var LINK={waiting:'等待数据',live:'通信正常',stale:'通信中断'};
/* 判定主卡的三段文字：[大标题, 副标题] */
var STATE={
  ok:['正常','AI 在线监测，未发现异常'],
  learn:['学习中','正在建立基准，保持正常运转约 37 秒'],
  alarm:['异常','检测到状态异常，请检查电机负载'],
  idle:['待机','等待电机启动'],
  off:['AI 未启用','固件未加载异常检测模型'],
  err:['推理故障','AI 运行异常，建议重启设备']
};
/* 当前在用的三路测量值有效位：bit0 电流 | bit4 转速 | bit5 母线电压
 * （加速度三位已随 MPU6050 摘除，恒为 0，不再参与判断） */
var CH_CURRENT=0x01,CH_SPEED=0x10,CH_BUS=0x20;

/* ---------- 趋势缓冲：每秒一个点，滚动保留最近 60 点 ---------- */
var HIST=60;
var spdHist=[],rHist=[];

function setState(k){
  document.body.dataset.state=k;
  el('stateTitle').textContent=STATE[k][0];
  el('stateSub').textContent=STATE[k][1];
}
function clearReadings(){
  /* 档位那一行特意把分隔符与百分号一起清空：无数据时只留一个「--」，
   * 否则会显示成「-- · -- %」，一屏横杠反而不知道在说什么 */
  el('rpm').textContent='--';
  el('current').textContent='--';
  el('bus').textContent='--';
  el('ratio').textContent='--';
  el('dr').textContent='--';
  el('uptime').textContent='--';
  setDrBar(null);   /* 条回到空槽，不给「0 是安全」的错觉 */
  el('level').textContent='--';
  el('duty').textContent='';
  el('gearDot').textContent='';
  el('gearPct').textContent='';
}
/* dR 进度条：长度 = dR / 门限(1.0)，颜色分三档。
 * 满格 = 已越门限。无效时给中性灰而不是绿色 —— 绿色会被读成「安全」。 */
var DR_LIMIT=1.0;
function setDrBar(dr){
  var bar=el('drBar');
  if(dr===null){bar.style.width='0%';bar.style.background='rgba(148,163,184,.35)';return;}
  var pct=Math.min(dr/DR_LIMIT,1)*100;
  bar.style.width=pct.toFixed(1)+'%';
  bar.style.background=(dr>=DR_LIMIT)?'var(--bad)':((dr>=DR_LIMIT*0.5)?'var(--warn)':'var(--ok)');
}
/* 按状态码决定主卡该显示哪一态 */
function pickState(d){
  if(d.ai_alarm===1){return 'alarm';}
  if(d.ai_phase==='learning'){return 'learn';}
  if(d.ai_phase==='off'){return 'off';}
  if(d.ai_phase==='error'){return 'err';}
  return 'ok';
}

/* ---------- 趋势绘制 ----------
 * 两条线各自按「自己这 60 点里的极值」缩放：转速 140 与 R 13 量级差十倍，
 * 共用一根 Y 轴的话 R 会被压成一条直线。也不做双 Y 轴 —— 手机小屏上
 * 两套刻度会互相干扰，看不出读数。用颜色区分 + 底部标区间就够了。
 * 无效值存 null，画线时断开，不补 0（补 0 会把「没采到」误画成「测到 0」）。 */
function pushHist(spd,r){
  spdHist.push(spd); rHist.push(r);
  if(spdHist.length>HIST){spdHist.shift();}
  if(rHist.length>HIST){rHist.shift();}
}
function extent(arr){
  var lo=null,hi=null,i,v;
  for(i=0;i<arr.length;i++){
    v=arr[i];
    if(v===null||typeof v!=='number'){continue;}
    if(lo===null||v<lo){lo=v;}
    if(hi===null||v>hi){hi=v;}
  }
  return [lo,hi];
}
/* minSpan 是这个量的「最小显示跨度」：
 * 稳态下转速只摆 1~2 rpm，若完全按极值拉满，一条几乎水平的线会被放大成
 * 剧烈起伏，看的人会以为电机在抖。给一个下限之后，小波动就老老实实是小平线，
 * 真出问题时（掉几十 rpm）照样顶到边。 */
function drawSeries(g,arr,w,h,col,minSpan){
  var ex=extent(arr),lo=ex[0],hi=ex[1];
  if(lo===null){return;}
  var span=hi-lo;
  if(span<minSpan){                       /* 量程太窄：以中点为心撑到最小跨度 */
    var mid=(hi+lo)/2;
    lo=mid-minSpan/2; hi=mid+minSpan/2;
  }else{                                  /* 正常：上下各留 20% 余量 */
    var pad=span*0.2; lo-=pad; hi+=pad;
  }
  /* 左右各留 3 px：末端那颗点有半径，贴边画会被裁掉一半 */
  var top=5,bot=h-5,step=(w-6)/(HIST-1),x0=3,off=HIST-arr.length,i,x,y,started=false;
  g.strokeStyle=col;g.lineWidth=2;g.lineJoin='round';g.lineCap='round';
  g.beginPath();
  for(i=0;i<arr.length;i++){
    if(arr[i]===null||typeof arr[i]!=='number'){started=false;continue;}
    x=x0+(off+i)*step;
    y=bot-(arr[i]-lo)/(hi-lo)*(bot-top);
    if(started){g.lineTo(x,y);}else{g.moveTo(x,y);started=true;}
  }
  g.stroke();
  /* 末端点：给「当前值在哪」一个视觉锚 */
  for(i=arr.length-1;i>=0;i--){
    if(arr[i]===null||typeof arr[i]!=='number'){continue;}
    x=x0+(off+i)*step; y=bot-(arr[i]-lo)/(hi-lo)*(bot-top);
    g.fillStyle=col;g.beginPath();g.arc(x,y,2.4,0,Math.PI*2);g.fill();
    break;
  }
}
/* 一格画一条线：画布尺寸按 devicePixelRatio 放大，否则 Retina 屏上发虚 */
function drawPlot(id,arr,col,minSpan){
  var c=el(id);
  if(!c){return;}
  var dpr=window.devicePixelRatio||1;
  var w=c.clientWidth||240,h=c.clientHeight||54;
  if(c.width!==Math.round(w*dpr)||c.height!==Math.round(h*dpr)){
    c.width=Math.round(w*dpr);c.height=Math.round(h*dpr);
  }
  var g=c.getContext('2d');
  g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);
  /* 一条中线当参照：线在上=高于本段均值，在下=低于 */
  g.strokeStyle='rgba(148,163,184,.2)';g.lineWidth=1;
  var mid=Math.round(h/2)+0.5;
  g.beginPath();g.moveTo(0,mid);g.lineTo(w,mid);g.stroke();
  drawSeries(g,arr,w,h,col,minSpan);
}
function drawChart(){
  var cs=getComputedStyle(document.documentElement);
  var spdCol=(cs.getPropertyValue('--spd')||'#38bdf8').trim();
  var rrCol=(cs.getPropertyValue('--rr')||'#a78bfa').trim();
  drawPlot('chartSpd',spdHist,spdCol,4);      /* 转速最小跨度 4 rpm */
  drawPlot('chartR',rHist,rrCol,0.4);         /* R 最小跨度 0.4 rpm/V */
}
/* 底部区间：只统计有效点，没有有效点就显示 -- */
function refreshRange(){
  var a=extent(spdHist),b=extent(rHist);
  el('spdMin').textContent=(a[0]===null?'--':a[0].toFixed(1));
  el('spdMax').textContent=(a[1]===null?'--':a[1].toFixed(1));
  el('rMin').textContent=(b[0]===null?'--':b[0].toFixed(2));
  el('rMax').textContent=(b[1]===null?'--':b[1].toFixed(2));
}

function show(d){
  if(['waiting','live','stale'].indexOf(d.link)<0){throw new Error('Bad status');}
  document.body.dataset.link=d.link;
  el('linkText').textContent=LINK[d.link];
  clearReadings();
  el('age').textContent=(d.age_ms===null?'--':(d.age_ms/1000).toFixed(1));
  el('frames').textContent=d.frames;
  el('rejected').textContent=d.rejected;
  el('version').textContent=d.version;
  el('cond').textContent='';

  if(d.link!=='live'){
    setState('idle');
    /* 链路状态和主卡文字分开说：主卡说「判定不出结果」，胶囊说「为什么」 */
    el('stateSub').textContent=(d.link==='stale'?'STM32 已停止发送数据':'等待 STM32 上电');
    pushHist(null,null);
    drawChart();refreshRange();
    return;
  }

  var rpm=d.rpm_x10,mv=d.battery_mV;
  el('rpm').textContent=(rpm===null?'--':(rpm/10).toFixed(1));
  el('current').textContent=(d.current_mA===null?'--':(d.current_mA/1000).toFixed(3));
  el('bus').textContent=(mv===null?'--':(mv/1000).toFixed(2));
  /* 特征比值 R：与板上 OLED 完全同一口径。
   * 第一步算原始的 rpm/V；第二步按膝点归一化到 58% 基准档，即乘 (58-50)/(duty-50)。
   * 归一化的意义：空载转速对占空比是「过膝点的直线」，R = 17.5 x (duty-50)，
   * 所以除以 (duty-50) 之后不同档位的 R 才可比。
   * duty<=50 落在起动死区里，这个式子没有物理意义，一律显示 --。 */
  var ratio=null;
  if(rpm>0&&mv>0&&d.duty>50){ratio=(rpm*100/mv)*(8/(d.duty-50));}
  el('ratio').textContent=(ratio===null?'--':ratio.toFixed(2));
  /* dR：D1 侧用最近 32 帧就地算出的窗内 rpm/V 峰峰值，与 OLED 轮显的那个 dR 同一个数，
   * 单位都是 rpm/V、都保留两位小数。固件给的是放大 1000 倍的整数（省掉浮点 JSON）。 */
  var dr=(typeof d.dr_x1000==='number')?(d.dr_x1000/1000):null;
  el('dr').textContent=(dr===null?'--':dr.toFixed(2));
  setDrBar(dr);
  /* 已运行秒数：对齐 OLED 底条上的 T:xxx s */
  el('uptime').textContent=(typeof d.uptime_ms==='number')?Math.floor(d.uptime_ms/1000):'--';
  el('level').textContent='L'+d.level;
  el('duty').textContent=d.duty;
  el('gearDot').textContent='·';
  el('gearPct').textContent='%';

  var st=pickState(d);
  setState(st);
  if(st==='ok'&&rpm===0&&d.duty===0){el('stateSub').textContent='电机已停转，等待启动';}

  /* 逐路点名：缺哪一路就说哪一路，比笼统的「部分测量无效」有用 —— 一眼能对上接线 */
  var miss=[],mask=d.feature_valid_mask;
  if((mask&CH_CURRENT)===0){miss.push('电流');}
  if((mask&CH_SPEED)===0){miss.push('转速');}
  if((mask&CH_BUS)===0){miss.push('母线电压');}
  if(miss.length>0){el('cond').textContent='未采到：'+miss.join(' / ')+'（检查对应接线）';}

  pushHist(rpm===null?null:rpm/10, ratio);
  drawChart();refreshRange();
}
/* 每秒拉一次 /status，2.2 秒超时；超时或失败即判定连接中断 */
function refresh(){
  var controller=new AbortController();
  var timer=setTimeout(function(){controller.abort();},2200);
  fetch('/status',{cache:'no-store',signal:controller.signal})
    .then(function(r){if(!r.ok){throw new Error('HTTP');}return r.json();})
    .then(show)
    .catch(function(){
      document.body.dataset.link='offline';
      el('linkText').textContent='连接中断';
      clearReadings();setState('idle');
      el('stateSub').textContent='手机与 D1 的连接已断开';
      el('age').textContent='--';el('cond').textContent='';
      pushHist(null,null);drawChart();refreshRange();
    })
    .then(function(){clearTimeout(timer);setTimeout(refresh,1000);});
}
/* 转屏或窗口尺寸变化时重算画布，否则曲线会被拉花 */
window.addEventListener('resize',function(){drawChart();});
refresh();
</script></body></html>
)HTML"; /* 页面字符串在此闭合：)HTML 收尾，分号结束上面那条变量定义 */
#endif /* MONITOR_PAGE_H 条件编译段结束 */
