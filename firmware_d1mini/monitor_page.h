/* ==========================================================================
 *  monitor_page.h  --  手机看板网页（一整个 HTML 页面，直接编译进固件里）
 *
 *  【这是什么】一个完整的单页网页：手机连上 D1 的热点/同一 WiFi，
 *    浏览器打开 http://192.168.4.1 就能看到电机的实时数据。
 *
 *  【三个设计约束，决定了它长这样】
 *    ① 全部自带：CSS 和 JS 都写在页面里，不引用任何外部网址。
 *       现场没有外网也能正常打开（这是嵌入式页面最重要的一条）。
 *    ② 放在 Flash 里：数组用 PROGMEM 声明，不占宝贵的 RAM。
 *       页面内容用「原生字符串字面量」(raw string，起止标记里带 HTML 分隔符)包起来，
 *       这样里面的引号、反斜杠都不用转义，写起来和普通 HTML 一样。
 *    ③ 只读：页面只做 GET /status，没有任何会改数据的按钮。
 *
 *  【页面结构】
 *    顶部标题 + 一个状态圆点（绿=通信正常，红=中断）
 *    中间 10 张读数卡片：档位、占空比、转速、电流、X/Y/Z 加速度、母线电压、AI 状态、报警
 *    中间一行提示文字：数据不全时提示「部分测量无效」，报警时提示「检测到异常」
 *    底部统计：数据年龄、有效帧数、异常帧数、版本号
 *
 *  【数据怎么来的】页内 JS 每秒 fetch('/status') 一次（带 2.2 秒超时），
 *    把返回的 JSON 填到各个 <span> 里；拿到 null 就显示「--」。
 * ========================================================================== */
/* 2026-10-05：自包含的手机端监控页，用于展示 M2 遥测帧里的六个测量字段。 */
#ifndef MONITOR_PAGE_H /* 重复包含保护：首次进来时该宏尚未定义 */
#define MONITOR_PAGE_H /* 定义包含标记，之后再 #include 本文件时上面那条 #ifndef 即为假 */
/* 【这行是整个页面的容器】
 *   static const char MONITOR_PAGE[]                    页面字节数组
 *   PROGMEM                                             放到 Flash 而不是 RAM
 *   = 「原生字符串字面量」                                 原样保留，不做转义处理
 * 页面里所有中文都写成 &#xxxxx; 这种数字实体（例如 &#30005; = 电），
 * 这样无论源文件是什么编码，浏览器看到的中文都一定正确。
 * 【注意】修改这个页面时不要动开头和结尾的起止标记，
 * 也不要让页面里出现「右括号+HTML+引号」这一串字符（那会提前结束字符串）。 */
/* 【页面正文起点】从下面这行的 R"HTML( 到文件末尾的 )HTML" 之间是页面正文
 *  （HTML + CSS + JavaScript），属于字符串数据而不是 C++ 代码，
 *  因此页面内部的说明一律用 HTML / CSS / JS 各自的注释语法书写，
 *  不要在这里插 C++ 注释（那会变成下发给浏览器的文本，还会撑大固件）。 */
static const char MONITOR_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<!-- head 区：声明 UTF-8 编码 + 手机自适应视口 + 页面样式（CSS） -->
<title>&#30005;&#26426;&#29366;&#24577;&#30417;&#27979;</title><style>
*{box-sizing:border-box}body{margin:0;background:#f4f6f6;color:#20272b;font:16px system-ui,sans-serif;letter-spacing:0}
main{max-width:760px;margin:auto;padding:24px 20px}h1{font-size:24px;margin:0 0 18px;line-height:1.4}
#link{display:flex;align-items:center;gap:10px;border-bottom:1px solid #cbd2d5;padding:0 0 20px;min-height:48px}
#dot{width:10px;height:10px;border-radius:50%;background:#777;flex:none}body[data-link=live] #dot{background:#158057}
body[data-link=stale] #dot,body[data-link=offline] #dot{background:#be442c}.readings{margin:0;display:grid;grid-template-columns:1fr 1fr;gap:0 28px}
.reading{min-width:0;padding:20px 0;border-bottom:1px solid #d3dadd}dt{font-size:14px;color:#5a646b}
dd{margin:8px 0 0;font-size:25px;font-variant-numeric:tabular-nums;overflow-wrap:anywhere;min-height:36px}
small{font-size:14px;color:#60686c}#condition{min-height:24px;margin:18px 0;color:#8f4822}
#ai_alarm.bad{color:#be442c;font-weight:700}
.meta{font-size:13px;color:#626a70;line-height:1.8;overflow-wrap:anywhere}.meta span{font-variant-numeric:tabular-nums}
@media(max-width:380px){main{padding:20px 14px}.readings{gap:0 16px}dd{font-size:21px}}
</style></head><body data-link="waiting"><main>
<h1>&#30005;&#26426;&#29366;&#24577;&#30417;&#27979;</h1>
<!-- 顶部状态条：一个圆点 + 一行文字
     圆点颜色由 <body> 上的 data-link 属性驱动（live 绿 / stale 红 / offline 红） -->
<div id="link" role="status"><span id="dot" aria-hidden="true"></span><span id="linkText">&#31561;&#24453;&#25968;&#25454;</span></div>
<!-- 十张读数卡片。每张是一个 .reading：一行标题(dt) + 一行大数字(dd)。
     这些 span 的 id 和 /status 返回的 JSON 字段名一一对应，
     下面的 JS 就是按 id 去找这些元素填数字的。 -->
<dl class="readings">
<div class="reading"><dt>&#26723;&#20301;</dt><dd id="level">--</dd></div>
<div class="reading"><dt>&#24403;&#21069;&#21344;&#31354;&#27604;</dt><dd><span id="duty">--</span> <small>%</small></dd></div>
<div class="reading"><dt>&#36716;&#36895;</dt><dd><span id="rpm_x10">--</span> <small>rpm</small></dd></div>
<div class="reading"><dt>&#30005;&#27969;</dt><dd><span id="current_mA">--</span> <small>A</small></dd></div>
<div class="reading"><dt>X &#21152;&#36895;&#24230;</dt><dd><span id="accel_x_mg">--</span> <small>g</small></dd></div>
<div class="reading"><dt>Y &#21152;&#36895;&#24230;</dt><dd><span id="accel_y_mg">--</span> <small>g</small></dd></div>
<div class="reading"><dt>Z &#21152;&#36895;&#24230;</dt><dd><span id="accel_z_mg">--</span> <small>g</small></dd></div>
<div class="reading"><dt>&#27597;&#32447;&#30005;&#21387;</dt><dd><span id="battery_mV">--</span> <small>V</small></dd></div>
<div class="reading"><dt>AI &#29366;&#24577;</dt><dd id="ai_phase">--</dd></div>
<div class="reading"><dt>&#24322;&#24120;&#25253;&#35686;</dt><dd id="ai_alarm">--</dd></div>
<!-- 中间那行提示文字：由 JS 动态填
     例如「部分流量无效」「检测到异常，请检查电机」 -->
</dl><p id="condition" role="status"></p>
<!-- 底部统计信息：数据年龄（秒）、累计有效帧、累计异常帧、固件版本 -->
<div class="meta">&#25968;&#25454;&#24180;&#40836; <span id="age">--</span> s<br>
&#26377;&#25928;&#24103; <span id="frames">0</span> &middot; &#24322;&#24120;&#24103; <span id="rejected">0</span><br>
<!-- ======================= 页面脚本（JavaScript） =======================
     逻辑很简单，三块：
       ① specs / aiPhaseText：定义每个测量值的「整除系数」和显示位数，以及 AI 阶段的中文说法
       ② show(d)：把 /status 返回的 JSON 填进页面
       ③ refresh()：每秒请求一次 /status，带 2.2 秒超时；失败就显示「连接中断」
     【为什么要有超时】WiFi 信号差的时候 fetch 可能永远不返回，
     不给超时页面就会一直停在旧数据上，看起来像「卡死」。 -->
<span id="version">d1-telemetry-2.0.0</span></div></main><script>
'use strict';
// 每个测量值的显示换算规则：[整除系数, 保留小数位]
// 因为协议里传的都是整数（省带宽），显示时要除回来：
//   电流 mA / 1000 = 安培，保留 3 位小数
//   加速度 mg / 1000 = g，保留 3 位；转速 x10 / 10 = rpm，保留 1 位
//   母线电压 mV / 1000 = 伏，保留 2 位
const specs={current_mA:[1000,3],accel_x_mg:[1000,3],accel_y_mg:[1000,3],accel_z_mg:[1000,3],rpm_x10:[10,1],battery_mV:[1000,2]};
// AI 阶段的显示文字：和 status_json.cpp 里输出的 off/learning/detecting/error 一一对应
// 用 \u 转义写中文，避免页面编码出问题
const aiPhaseText={off:'\u672a\u542f\u7528',learning:'\u5b66\u4e60\u4e2d',detecting:'\u68c0\u6d4b\u4e2d',error:'\u9519\u8bef'};
const el=id=>document.getElementById(id);
function clearReadings(){for(const k of ['level','duty','ai_phase','ai_alarm',...Object.keys(specs)])el(k).textContent='--';el('ai_alarm').classList.remove('bad');}
// show(d)：收到 /status 的 JSON 后，把页面刷新一遍
// 步骤：校验 link 字段是否合法 -> 更新 body 的 data-link（驱动圆点颜色）
//       -> 更新状态文字 -> 先清空所有读数 -> 只有 live 时才填真实数字
//       -> 置空的位置显示「--」，报警时给「⚠ 报警」加红色样式
function show(d){
 if(!['waiting','live','stale'].includes(d.link))throw Error('Bad status');
 document.body.dataset.link=d.link;
 el('linkText').textContent={waiting:'\u7b49\u5f85 STM32 \u6570\u636e',live:'\u901a\u4fe1\u6b63\u5e38',stale:'STM32 \u901a\u4fe1\u4e2d\u65ad'}[d.link];
 clearReadings();
 if(d.link==='live'){
  el('level').textContent='L'+d.level;el('duty').textContent=d.duty;
  for(const [k,[scale,digits]] of Object.entries(specs))el(k).textContent=d[k]===null?'--':(d[k]/scale).toFixed(digits);
  el('ai_phase').textContent=d.ai_phase==='null'?'--':(aiPhaseText[d.ai_phase]||d.ai_phase);
  const alarm=d.ai_alarm===1;
  el('ai_alarm').textContent=alarm?'\u26a0 \u62a5\u8b66':'\u6b63\u5e38';
  el('ai_alarm').classList.toggle('bad',alarm);
 }
 let condition='';
 if(d.link==='live'&&d.feature_valid_mask!==63)condition='\u90e8\u5206\u6d4f\u91cf\u65e0\u6548';
 if(d.link==='live'&&d.ai_alarm===1)condition=(condition?condition+' \u00b7 ':'')+'\u68c0\u6d4b\u5230\u5f02\u5e38\uff0c\u8bf7\u68c0\u67e5\u7535\u673a';
 el('condition').textContent=condition;
 el('age').textContent=d.age_ms===null?'--':(d.age_ms/1000).toFixed(1);
 el('frames').textContent=d.frames;el('rejected').textContent=d.rejected;el('version').textContent=d.version;
}
// refresh()：每秒轮询一次（自己调用自己，形成无限循环）
// r.ok 为假（比如 500）也当失败处理；失败时把页面切到 offline 并清空读数，
// 这样「数据是旧的」和「真断了」能一眼区分开。
// 无论成功失败，finally 里都会在 1 秒后再调一次 refresh()。
async function refresh(){
 const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),2200);
 try{const r=await fetch('/status',{cache:'no-store',signal:controller.signal});if(!r.ok)throw Error('HTTP');show(await r.json());}
 catch(e){document.body.dataset.link='offline';el('linkText').textContent='\u624b\u673a\u4e0e D1 \u8fde\u63a5\u4e2d\u65ad';clearReadings();el('age').textContent='--';el('condition').textContent='';}
 finally{clearTimeout(timer);setTimeout(refresh,1000);}
}
refresh();
</script></body></html>)HTML"; /* 页面字符串在此闭合：)HTML 收尾，分号结束上面那条变量定义 */
#endif /* MONITOR_PAGE_H 条件编译段结束 */
