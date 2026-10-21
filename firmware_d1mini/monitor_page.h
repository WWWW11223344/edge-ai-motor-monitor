/* ==========================================================================
 *  monitor_page.h  --  手机看板网页（一整个 HTML 页面，直接编译进固件里）
 *
 *  【这是什么】一个完整的单页网页：手机连上 D1 的热点 / 同一 WiFi，
 *    浏览器打开 http://192.168.4.1 就能看到电机的实时数据。
 *
 *  【三个设计约束，决定了它长这样】
 *    ① 全部自带：CSS 和 JS 都写在页面里，不引用任何外部网址。
 *       现场没有外网也能正常打开（这是嵌入式页面最重要的一条）。
 *    ② 放在 Flash 里：数组用 PROGMEM 声明，不占宝贵的 RAM。
 *       页面内容用「原生字符串字面量」(raw string) 包起来，
 *       这样里面的引号、反斜杠都不用转义，写起来和普通 HTML 一样。
 *    ③ 只读：页面只做 GET /status，没有任何会改数据的按钮。
 *
 *  【页面结构（2026-10-20 重做）】
 *    顶部：标题 + 链路状态胶囊（绿=通信正常，红=中断）
 *    AI 判定主卡：正常 / 学习中 / 异常 / 待机 / 未启用 / 推理故障，六种状态分色
 *    六格读数：转速、电流、母线电压、特征比值 R、档位、占空比
 *    底部：数据年龄、有效帧 / 异常帧、固件版本
 *
 *  【为什么没有加速度了】
 *    MPU6050 已于 2026-10-19 从 STM32 固件摘除（HARDWARE/_removed_mpu6050_20261019），
 *    帧里 accel_x/y/z 三路的有效位恒为 0，永远只能显示「--」。
 *    页面直接不再展示，改展示「特征比值 R」——那才是 AI 真正的输入。
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
<meta name="theme-color" content="#f2f3f5" media="(prefers-color-scheme:light)">
<meta name="theme-color" content="#0f1115" media="(prefers-color-scheme:dark)">
<title>电机状态监测</title><style>
/* 配色全部走 CSS 变量：浅色 / 深色各一套，手机跟随系统自动切换 */
:root{
  --bg:#f2f3f5;--card:#ffffff;--ink:#12161c;--muted:#6b7480;--line:#e4e7eb;
  --ok:#0b7d5a;--ok-bg:#e4f5ee;
  --warn:#9a6206;--warn-bg:#fdf2df;
  --bad:#c2341c;--bad-bg:#fdeae7;
  --idle:#6b7480;--idle-bg:#e9ecef;
}
@media (prefers-color-scheme:dark){
  :root{
    --bg:#0f1115;--card:#181c22;--ink:#f2f4f7;--muted:#98a2b0;--line:#252b34;
    --ok:#3fd39b;--ok-bg:#11322a;
    --warn:#f0b357;--warn-bg:#33280f;
    --bad:#ff6b57;--bad-bg:#3a1a16;
    --idle:#98a2b0;--idle-bg:#212730;
  }
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--ink);-webkit-text-size-adjust:100%;
  font:16px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif}
main{max-width:560px;margin:0 auto;padding:18px 16px 26px}

/* ---------- 顶部：标题 + 链路状态胶囊 ---------- */
header{display:flex;align-items:center;justify-content:space-between;gap:10px;margin:2px 2px 14px}
h1{margin:0;font-size:20px;font-weight:650;letter-spacing:.01em}
#pill{display:inline-flex;align-items:center;gap:7px;flex:none;padding:7px 12px;
  border-radius:999px;background:var(--idle-bg);color:var(--idle);font-size:13px;font-weight:600;
  transition:background .2s,color .2s;white-space:nowrap}
#pill i{width:8px;height:8px;border-radius:50%;background:currentColor;flex:none}
body[data-link=live] #pill{background:var(--ok-bg);color:var(--ok)}
body[data-link=stale] #pill,body[data-link=offline] #pill{background:var(--bad-bg);color:var(--bad)}

/* ---------- AI 判定主卡 ---------- */
#state{background:var(--card);border:1px solid var(--line);border-radius:18px;
  padding:18px 20px 19px;margin-bottom:10px;transition:background .25s,border-color .25s}
#state .cap{font-size:12px;font-weight:700;letter-spacing:.12em;color:var(--muted)}
#stateTitle{font-size:30px;font-weight:700;line-height:1.2;margin:7px 0 3px;
  font-variant-numeric:tabular-nums;transition:color .25s}
#stateSub{font-size:13px;color:var(--muted);min-height:19px}
body[data-state=ok] #state{background:var(--ok-bg);border-color:transparent}
body[data-state=ok] #stateTitle{color:var(--ok)}
body[data-state=learn] #state{background:var(--warn-bg);border-color:transparent}
body[data-state=learn] #stateTitle{color:var(--warn)}
body[data-state=alarm] #state{background:var(--bad-bg);border-color:transparent;
  animation:breathe 1.5s ease-in-out infinite}
body[data-state=alarm] #stateTitle{color:var(--bad)}
@keyframes breathe{0%,100%{opacity:1}55%{opacity:.66}}

/* ---------- 六格读数 ---------- */
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.cell{background:var(--card);border:1px solid var(--line);border-radius:14px;
  padding:13px 15px 14px;min-width:0}
.cell b{display:block;font-size:12.5px;font-weight:600;color:var(--muted);margin-bottom:6px}
.cell p{margin:0;font-size:25px;font-weight:650;line-height:1.15;white-space:nowrap;
  font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
.cell p .u{font-size:13px;font-weight:600;color:var(--muted);margin-left:4px}

/* ---------- 底部 ---------- */
#cond{margin:14px 2px 0;font-size:13px;line-height:1.5;color:var(--warn)}
#cond:empty{display:none}
footer{margin-top:14px;padding:0 2px;font-size:12.5px;color:var(--muted);line-height:1.85}
footer div{display:flex;justify-content:space-between;gap:12px}
footer span{font-variant-numeric:tabular-nums}
#ver{opacity:.72;font-size:11.5px;margin-top:5px}
</style></head><body data-link="waiting" data-state="idle"><main>

<header>
  <h1>电机状态监测</h1>
  <div id="pill"><i></i><span id="linkText">等待数据</span></div>
</header>

<!-- AI 判定主卡：标题由 ai_phase / ai_alarm 决定，配色由 body[data-state] 驱动 -->
<section id="state">
  <div class="cap">运行判定</div>
  <div id="stateTitle">--</div>
  <div id="stateSub"></div>
</section>

<!-- 六格读数：只保留 STM32 当前真正在采的三路 + 档位/占空比 + 特征比值 -->
<div class="grid">
  <div class="cell"><b>转速</b><p><span id="rpm">--</span><span class="u">rpm</span></p></div>
  <div class="cell"><b>电流</b><p><span id="current">--</span><span class="u">A</span></p></div>
  <div class="cell"><b>母线电压</b><p><span id="bus">--</span><span class="u">V</span></p></div>
  <div class="cell"><b>特征比值 R</b><p><span id="ratio">--</span><span class="u">rpm/V</span></p></div>
  <div class="cell"><b>档位</b><p><span id="level">--</span></p></div>
  <div class="cell"><b>当前占空比</b><p><span id="duty">--</span><span class="u">%</span></p></div>
</div>

<p id="cond" role="status"></p>

<footer>
  <div><span>数据年龄</span><span><span id="age">--</span> s</span></div>
  <div><span>有效帧 / 异常帧</span><span><span id="frames">0</span> / <span id="rejected">0</span></span></div>
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

function setState(k){
  document.body.dataset.state=k;
  el('stateTitle').textContent=STATE[k][0];
  el('stateSub').textContent=STATE[k][1];
}
function clearReadings(){
  var ids=['rpm','current','bus','ratio','level','duty'],i;
  for(i=0;i<ids.length;i++){el(ids[i]).textContent='--';}
}
/* 按状态码决定主卡该显示哪一态 */
function pickState(d){
  if(d.ai_alarm===1){return 'alarm';}
  if(d.ai_phase==='learning'){return 'learn';}
  if(d.ai_phase==='off'){return 'off';}
  if(d.ai_phase==='error'){return 'err';}
  return 'ok';
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
  var ratioText='--';
  if(rpm>0&&mv>0&&d.duty>50){ratioText=((rpm*100/mv)*(8/(d.duty-50))).toFixed(2);}
  el('ratio').textContent=ratioText;
  el('level').textContent='L'+d.level;
  el('duty').textContent=d.duty;

  var st=pickState(d);
  setState(st);
  if(st==='ok'&&rpm===0&&d.duty===0){el('stateSub').textContent='电机已停转，等待启动';}

  /* 逐路点名：缺哪一路就说哪一路，比笼统的「部分测量无效」有用 —— 一眼能对上接线 */
  var miss=[],mask=d.feature_valid_mask;
  if((mask&CH_CURRENT)===0){miss.push('电流');}
  if((mask&CH_SPEED)===0){miss.push('转速');}
  if((mask&CH_BUS)===0){miss.push('母线电压');}
  if(miss.length>0){el('cond').textContent='未采到：'+miss.join(' / ')+'（检查对应接线）';}
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
    })
    .then(function(){clearTimeout(timer);setTimeout(refresh,1000);});
}
refresh();
</script></body></html>
)HTML"; /* 页面字符串在此闭合：)HTML 收尾，分号结束上面那条变量定义 */
#endif /* MONITOR_PAGE_H 条件编译段结束 */
