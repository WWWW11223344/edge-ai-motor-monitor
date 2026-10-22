/* ============================================================================
 *  d1mini_telemetry.ino  --  ESP8266（Wemos D1 mini）遥测网关
 *
 *  【这个程序干什么】一句话：
 *    STM32 板子通过一根线（D6 引脚）不停地把「电机状态二进制包」发过来，
 *    这块 D1 mini 负责把包收下来、存住最近 256 条，然后提供两种看法：
 *      1) 本地 WiFi 上开一个只读网页，手机 / 电脑连上就能看实时数据（不用装 APP）；
 *      2) USB 串口输出 JSON 给电脑，还支持「60 秒原始码流抓包」，用于离线分析、
 *         给 AI 模型攒训练样本。
 *
 *  【硬件接线】只有 2 根相关线：
 *    STM32 USART3_TX(PB10) --杜邦线--> D1 mini D6(GPIO12)   57600 波特率，8N1
 *    D1 mini 的 USB 口      --USB线---> 电脑              115200，打印日志 / 抓包 / 收命令
 *    D1 mini 只收不发（TX 引脚给 -1），永远不会去驱动 STM32 的引脚，接线接错也不会烧。
 *
 *  【两种工作模式（互斥，见 loop()）】
 *    正常模式：WiFi + 网页服务器 + 每 1ms 从串口搬一次数据
 *    抓包模式：进入前先把 WiFi 整个关掉，60 秒内把收到的原始帧原样转发到 USB，
 *              结束（或敲 s）后自动把 WiFi 恢复回来
 *
 *  【三个只读接口（只能看，没有任何能改数据的接口）】
 *    GET /         网页看板（HTML，内容来自 monitor_page.h）
 *    GET /status   当前状态 JSON：最新一帧 + 各种计数器
 *    GET /raw      历史帧 JSON：?after=N 增量拉 / ?tail=1 取最新 / ?limit=N 控制条数
 *
 *  【必须先懂的 3 个概念】
 *    ① 软件串口（SoftwareSerial）：ESP8266 只有 1 个硬件串口，还被 USB 日志占了，
 *       所以收 STM32 数据只能靠「软件串口」——用 GPIO 中断 + 边沿时间戳去还原每个 bit。
 *       它的死穴是：WiFi 中断一多，bit 采样就会偏，帧被校验拒掉。
 *       所以全文到处是「关 WiFi 睡眠」「不要 yield」「抓包前先关 WiFi」这类讲究。
 *    ② 二进制帧：STM32 发的是 42 字节定长包（帧头 A5 5A + 序号 + 各种测量值 + CRC），
 *       解析规则见 m2_protocol.h；只要收到的时间对不上就整帧丢掉，保证数据可信。
 *    ③ 非阻塞：ESP8266 单核，任何一次 delay/死等都会让网页卡住或丢串口数据，
 *       所以每个函数都尽量「干一点活就赶紧返回」，靠反复调用慢慢推进。
 * ============================================================================ */
// 2026-10-05：GPIO12(D6) 只收不发地接收 STM32 的 M2 遥测，加密热点 + 只读 HTTP。
/* ---------------------------------------------------------------- 头文件
 *  ESP8266WiFi.h        : 连 WiFi（STA 模式）
 *  ESP8266WebServer.h   : 内置 HTTP 服务器（监听 80 端口）
 *  SoftwareSerial.h     : 软件串口，用来收 STM32 的 57600 数据
 *  Schedule.h           : 注册「在空闲时反复执行的小函数」（这里用来定时收串口）
 *  wifi_config.h        : 本地 WiFi 账号密码 + 连接超时（本文件不要提交 / 外传）
 *  m2_protocol.h        : 与 STM32 约定的 42 字节帧格式：编码、解码、CRC 校验
 *  raw_history.h        : 最近 256 帧的环形缓冲（网页翻历史用）
 *  status_json.h        : 把当前状态拼成 JSON 字符串
 *  monitor_page.h       : 网页看板的 HTML，作为常量字符串编进 Flash，不占 RAM
 * ---------------------------------------------------------------- */
#include <ESP8266WiFi.h> /* 连接路由器 WiFi（STA 模式） */
#include <ESP8266WebServer.h> /* 内置 HTTP 服务器，监听 80 端口 */
#include <SoftwareSerial.h> /* 软件串口：唯一硬串口被 USB 日志占了，收数据只能软件模拟 */
#include <Schedule.h> /* 注册「空闲时反复执行的小函数」，用来每 1ms 搬一次串口 */
#include <stdlib.h> /* strtoul / snprintf 等标准库函数声明 */
#include <string.h> /* memcpy 等内存与字符串函数声明 */
#include "wifi_config.h" /* 本地 WiFi 账号密码与连接超时，切勿外传 */
#include "m2_protocol.h" /* 与 STM32 约定的 42 字节二进制帧格式 */
#include "raw_history.h" /* 最近 256 帧的环形缓冲，网页翻历史用 */
#include "status_json.h" /* 把当前状态拼成 JSON */
#include "monitor_page.h" /* 网页看板 HTML，编进 Flash 不占 RAM */

/* ------------------------------------------------------------ 兼容旧 core 的宏
 * 【为什么要有这段】esp8266 core 3.x 把 F_CRYSTAL 收进了 core_esp8266_main.cpp
 * 内部（只是那个 .cpp 的局部宏），不再对外暴露，所以在 sketch 里直接用会报
 * 「F_CRYSTAL was not declared」。而开机横幅要打印晶振频率 —— 软件串口的波特率
 * 正是按它算出来的，频率不对收到的就全是错帧，所以这条诊断不能删。
 * 这里补一个默认值：ESP8266 晶振固定 26 MHz。core 2.x 自己会定义，故用 #ifndef 包住。 */
#ifndef F_CRYSTAL
#define F_CRYSTAL 26000000UL /* ESP8266 晶振 26 MHz，core 3.x 不再导出，此处兜底 */
#endif

/* ------------------------------------------------------------ 可调参数
 * 这些都写成了 U 结尾的无符号常量（嵌入式习惯，避免隐式类型转换的坑）
 * 想改行为，优先只改这一段。
 * ------------------------------------------------------------ */
#define M2_RAW_DEFAULT_LIMIT 16U /* /raw 不传 limit 时默认返回几帧 */
#define M2_RAW_MAX_LIMIT 32U /* /raw 一次最多返回几帧，硬上限 */
#define STM32_UART_BAUD 57600U /* 与 STM32 约定的串口波特率 */
#define UART_FRAME_WAIT_US 9000U /* 单次搬数据的时间上限（微秒），到点就让出 CPU */
#define UART_PARTIAL_IDLE_US 30000U /* 半截帧静默多久判定源端断开（微秒） */
#define USB_CAPTURE_MS 60000U /* USB 抓包时长上限（毫秒） */
#define NETWORK_RESUME_DELAY_MS 2000U /* 抓包结束后等多久再恢复网络（毫秒） */
#define NETWORK_LOST_GRACE_MS 6000U /* 已连上后掉线，宽限多久才动手重连（毫秒） */

/* ------------------------------------------------------------ AP 兜底热点密码
 * 【为什么改成固定值】原来 AP 密码每次上电随机生成 14 位，只能从 USB 敲 p 才看得到。
 *   现场演示时手机根本没法输这个密码，等于没有兜底。
 *   改成固定密码后，手机输一次就能一直用（只读数据，风险可控）。
 *   想换密码就在 wifi_config.h 里定义 D1_AP_PASSWORD，本文件会自动让位。 */
#ifndef D1_AP_PASSWORD
#define D1_AP_PASSWORD "motor12345" /* AP 兜底热点密码：手机连 Motor-xxxxxx 时输这个 */
#endif

/* ------------------------------------------------------------ 全局变量
 * 按用途分 6 组看，别被这一堆吓到：
 *  ① 外设对象  : g_uart 软件串口、g_server 网页服务器
 *  ② 数据本体  : g_receiver 收帧状态机、g_history 最近 256 帧、g_rawSnapshot 查询快照
 *  ③ 工作缓冲  : g_rawChunk、g_ssid、g_password、g_uartBytes
 *  ④ 网络状态  : g_staReady / g_apReady / g_networkReady / 重连截止时间
 *  ⑤ 串口诊断  : g_uartOverflows / backlog / partial / discards（判断是谁弄丢了数据）
 *  ⑥ USB 抓包  : g_usb 开头的这批计数器，记录本次抓包的收发与丢弃
 *
 *  【为什么这些缓冲要放全局？】HTTP 处理函数是同步执行的，局部大数组会吃栈，
 *  而 ESP8266 的栈只有几 KB，一旦溢出就是「莫名其妙重启」，非常难查。
 * ------------------------------------------------------------ */
static EspSoftwareSerial::UART g_uart; /* 软件串口对象：用 GPIO12(D6) 收 STM32 的 57600 数据 */
static ESP8266WebServer g_server(80); /* 网页服务器对象，监听 80 端口 */
static M2Receiver g_receiver = {}; /* 收帧状态机：找同步头、拼满 42 字节、校验，产出最新一帧 */
static M2History g_history = {}; /* 最近 256 帧的环形缓冲，/raw 靠它翻历史 */
// 同步执行的 HTTP 处理函数会用到这些缓冲，放全局以免占用 ESP8266 很小的栈。
static M2Frame g_rawSnapshot[M2_RAW_MAX_LIMIT]; /* /raw 响应期间的帧快照，避免边读缓冲边被新帧改写 */
static char g_rawChunk[1024]; /* /raw 分块发送的拼装缓冲，一批最多 1KB */
static char g_ssid[32]; /* AP 模式热点名（Motor-芯片ID，31 字符 + 结束符） */
static char g_password[17]; /* AP 模式随机密码（14 位 + 结束符），只活在 RAM 里 */
static bool g_staReady = false; /* true = 已连上路由器，STA 模式可用 */
static bool g_apReady = false; /* true = 已开自己的热点，AP 兜底模式可用 */
static bool g_networkReady = false; /* true = 网页服务器已启动，STA 或 AP 都算 */
static bool g_routesRegistered = false; /* true = 路由已注册过，避免服务器重启时重复注册 */
static bool g_networkResumePending = false; /* true = 抓包刚结束，等待延时后恢复网络 */
static uint32_t g_networkResumeAtMs; /* 恢复网络的时刻，按 millis 计时 */
static uint32_t g_staDeadline; /* 连接路由器的截止时刻，超过就转 AP 兜底 */
static uint32_t g_staLostSinceMs; /* 发现掉线的那一刻；0 表示当前没掉线 */
static uint32_t g_uartOverflows; /* 软件串口 FIFO 溢出次数，说明 CPU 被占太久 */
static uint32_t g_uartBacklogEvents; /* 一次读到超过一帧的次数，说明上游有积压 */
static uint32_t g_uartBacklogBytes; /* 上面那种情况累计多出来的字节数 */
static uint32_t g_externalDiscards; /* 因溢出或半帧超时而整批丢弃的次数 */
static uint8_t g_uartBytes[512]; /* 从软件串口一次搬出来的原始字节暂存区 */
static bool g_uartReceiving; /* 重入保护：true 表示正在搬数据，避免回调再进来打乱状态 */
static uint32_t g_uartLastByteUs; /* 最近收到一个字节的时刻（微秒），用于半帧超时判断 */
static uint32_t g_uartPartialTimeouts; /* 半截帧超过 30ms 没有后续的丢弃次数 */
static uint32_t g_uartWaitLimits; /* 9ms 窗口读满仍未收完的次数，CPU 太忙或源端发太快 */
static bool g_usbActive; /* true = 正在 USB 抓包，此期间不处理网页、不打印日志 */
static uint32_t g_usbStartedMs; /* 本次抓包开始时刻 */
static uint32_t g_usbLastFrameMs; /* 最近收到一帧的时刻，用于源端超时判断 */
static uint32_t g_usbSeen; /* 本次抓包见到的帧数 */
static uint32_t g_usbSent; /* 本次成功转发到 USB 的帧数 */
static uint32_t g_usbDropped; /* 因 USB 写缓冲不够而丢掉的帧数 */
static uint32_t g_usbEncodeErrors; /* 重新编码失败的帧数 */
static uint32_t g_usbWriteErrors; /* 实际写入字节数不符的帧数 */
static uint32_t g_usbRejectedStart; /* 起点快照：被拒总帧数，结束时相减即本次增量 */
static uint32_t g_usbRejectVersionStart; /* 起点快照：版本号不符被拒 */
static uint32_t g_usbRejectLengthStart; /* 起点快照：帧长不符被拒 */
static uint32_t g_usbRejectFormatStart; /* 起点快照：帧头格式不符被拒 */
static uint32_t g_usbRejectCrcStart; /* 起点快照：CRC 校验失败被拒 */
static uint32_t g_usbRejectRangeStart; /* 起点快照：数值超出合理范围被拒 */
static uint32_t g_usbRejectOtherStart; /* 起点快照：其它原因被拒 */
static uint32_t g_usbOverflowStart; /* 起点快照：串口 FIFO 溢出次数 */
static uint32_t g_usbDiscardStart; /* 起点快照：整批丢弃次数 */
static uint32_t g_usbPartialStart; /* 起点快照：半帧超时次数 */
static uint32_t g_usbDuplicateStart; /* 起点快照：重复帧次数 */
static uint32_t g_usbResyncStart; /* 起点快照：重新同步次数 */

/* 前向声明：这几个函数在文件后面才定义，先在这里告诉编译器「有这么个函数」，
 * 以免上面的函数调用它们时报未声明。 */
static void send_status(); /* 处理状态查询请求 GET /status */
static void send_raw(); /* 处理历史帧查询请求 GET /raw */
static void recv_uart(bool waitFrame = true); /* 从软件串口搬数据，默认在 9ms 内尽量多搬 */
static void start_ap_fallback(); /* 连不上路由器时开自己的热点兜底 */

/* print_access()：打印当前该怎么访问（本地 USB 串口敲 p 才会调用）
 *   STA 模式：打印路由器热点名、拿到的 IP、访问地址
 *   AP 模式：打印自己热点的名字和随机密码
 * 【安全设计】AP 的随机密码只活在 RAM 里，只有本地插着 USB 显式要才打印；
 *   网页和 JSON 接口里永远不会出现密码。 */
// 临时密码只留在 RAM 里，只有本地 USB 显式请求时才打印。
static void print_access()
{
    if (g_staReady) { /* STA 模式：已连上路由器 */
        const String ip = WiFi.localIP().toString(); /* 把本机 IP 转成可打印的字符串 */
        /* 【为什么要打网关】手机打不开网页时，第一个要排的就是「在不在同一网段」：
         *   手机 IP 和这里打印的网关前三段一致，才可能访问得到 STA_URL。 */
        Serial.printf("STA_SSID=%s\nSTA_IP=%s\nSTA_GW=%s\nSTA_URL=http://%s\n",
            D1_WIFI_SSID, ip.c_str(), WiFi.gatewayIP().toString().c_str(), ip.c_str()); /* 实参：热点名 / 本机 IP / 网关 / 拼好的访问地址 */
    } else if (g_apReady) { /* 否则看 AP 兜底模式是否就绪 */
        Serial.printf("AP_SSID=%s\nAP_PASSWORD=%s\n", g_ssid, g_password); /* 打印热点名与固定密码，只有本地 USB 显式请求才走这里 */
        Serial.println("AP_URL=http://192.168.4.1");
    } else { /* 两种模式都还没就绪 */
        Serial.println("AP_NOT_READY"); /* 明确回一句「还没好」，避免上位机空等 */
    }
}

/* start_http_server()：注册路由 + 开始监听 80 端口
 * 路由只注册一次（g_routesRegistered 把关），因为服务器可能会被反复 start/stop。
 *   /         看板页，加 no-store 禁止浏览器缓存，保证每次看到的都是最新数据
 *   /status   状态 JSON
 *   /raw      历史帧 JSON
 *   其它路径   统一 404
 * 全是只读 GET，没有任何写操作，所以别人连上你的热点也只能看，改不了东西。 */
static void start_http_server()
{
    if (!g_routesRegistered) { /* 路由只注册一次：服务器可能被反复 start/stop */
        g_server.on("/", HTTP_GET, []() { /* 根路径：返回网页看板 */
            g_server.sendHeader("Cache-Control", "no-store"); /* 禁止浏览器缓存，保证每次都是最新数据 */
            g_server.send_P(200, "text/html; charset=utf-8", MONITOR_PAGE); /* send_P 直接从 Flash 发送页面，不先拷到 RAM */
        }); /* 根路径处理函数结束 */
        g_server.on("/status", HTTP_GET, send_status); /* /status：当前状态 JSON */
        g_server.on("/raw", HTTP_GET, send_raw); /* /raw：历史帧 JSON */
        g_server.onNotFound([]() { g_server.send(404, "text/plain", "Not found"); }); /* 其它路径统一 404，不做任何兜底动作 */
        g_routesRegistered = true; /* 记下「已注册」，下次进来就跳过这一整段 */
    }
    g_server.begin(); /* 开始监听 80 端口 */
    g_networkReady = true; /* 网络对外可用，网页可以访问了 */
}

/* suspend_network_for_capture()：抓包前把 WiFi 彻底关掉
 * 【为什么必须关】软件串口靠 GPIO 中断按时间戳采 bit，WiFi 的中断会插进来抢时间，
 *   导致采样点偏掉 -> 帧被 CRC 拒掉。所以严格抓包期间必须让 WiFi 完全闭嘴：
 *   停网页服务器 -> 关自动重连 -> 断开连接 -> 切 WIFI_OFF -> 进强制休眠。
 * 不会动 WiFi 配置（persistent(false) 保证不写 Flash），抓包结束后照原账号重连。
 * 参数不变、密码不变，只是临时把网络静音。 */
/* 严格 USB 抓包期间，让 57600 波特率的 GPIO12 软件接收器
 * 与 WiFi 的时序抖动隔离；WiFi 账号密码保持不变。 */
static void suspend_network_for_capture()
{
    g_server.stop(); /* 停掉网页服务器，不再接受请求 */
    g_networkReady = false; /* 标记网络不可用 */
    g_staReady = false; /* 清掉 STA 就绪标记 */
    g_apReady = false; /* 清掉 AP 就绪标记 */
    g_staLostSinceMs = 0U; /* 清掉掉线计时，抓包结束后按「重新连接」从头算 */
    WiFi.setAutoReconnect(false); /* 关掉自动重连，否则它会偷偷抢中断 */
    WiFi.disconnect(true); /* 断开当前连接 */
    WiFi.mode(WIFI_OFF); /* 彻底关掉 WiFi 射频 */
    WiFi.forceSleepBegin(); /* 进强制休眠，确保没有 WiFi 中断打扰串口采样 */
    delay(1); /* 等 1ms 让上面的关闭动作真正生效 */
}

/* resume_network_after_capture()：抓包结束后把网络恢复回来
 * 顺序有讲究：唤醒 -> 设模式(STA) -> 开自动重连 -> 关 WiFi 睡眠 -> 重新 begin 连接
 * 每一步都可能失败，失败就立刻转 AP 兜底，保证「现场没有路由器时也能看数据」。
 * 连不上路由器有 20 秒超时（D1_WIFI_CONNECT_TIMEOUT_MS），超时由 service_network() 处理。 */
static void resume_network_after_capture()
{
    g_networkResumePending = false; /* 清掉「待恢复」标记 */
    WiFi.forceSleepWake(); /* 先从强制休眠里唤醒 */
    delay(1); /* 等 1ms 让唤醒生效 */
    WiFi.persistent(false); /* 不让 WiFi 参数写 Flash，延长寿命 */
    WiFi.setAutoReconnect(true); /* 重新打开自动重连 */
    if (!WiFi.mode(WIFI_STA)) { /* 切回 STA 模式；失败就转 AP 兜底 */
        Serial.println("STA_MODE_FAILED; starting AP fallback"); /* 打印失败原因，便于现场排查 */
        start_ap_fallback(); /* 兜底：自己开热点 */
        return; /* 已经转 AP，直接返回 */
    }
    WiFi.setSleepMode(WIFI_NONE_SLEEP);  // 禁止 WiFi 睡眠，否则它的唤醒中断会打乱软件串口采样
    WiFi.begin(D1_WIFI_SSID, D1_WIFI_PASSWORD); /* 用原账号密码重新连接路由器 */
    g_staDeadline = millis() + D1_WIFI_CONNECT_TIMEOUT_MS; /* 设置连接超时时刻，超时由 service_network() 处理 */
    Serial.printf("STA_RECONNECT ssid=%s timeout_ms=%lu\n", D1_WIFI_SSID, /* 打印重连信息，不含密码 */
        D1_WIFI_CONNECT_TIMEOUT_MS); /* 超时毫秒数 */
}

/* start_ap_fallback()：兜底方案——自己开热点
 * 什么时候用：连不上指定的路由器（换了场地、路由器没开），
 *   或者连上之后又掉线、重连也救不回来。
 * 热点名 Motor-XXXXXX（XXXXXX 是芯片 ID，多块板子能认出来是哪块）。
 * 密码是固定值 D1_AP_PASSWORD（原来每次上电随机 14 位、只能从 USB 敲 p 才看得到，
 *   现场演示时手机根本连不上，等于没有兜底）。最多 2 个设备同时连，
 *   固定地址 192.168.4.1。密码同样只在 USB 敲 p 时打印，不写进网页。 */
static void start_ap_fallback()
{
    snprintf(g_ssid, sizeof(g_ssid), "Motor-%06X", ESP.getChipId()); /* 热点名 = Motor-6 位芯片 ID，多块板子能区分 */
    snprintf(g_password, sizeof(g_password), "%s", D1_AP_PASSWORD); /* 固定密码，手机输一次就能一直用 */
    g_apReady = WiFi.mode(WIFI_AP) && WiFi.softAP(g_ssid, g_password, 1, false, 2); /* 开热点：信道 1、不隐藏、最多 2 个设备接入 */
    if (g_apReady) { /* 热点开成功 */
        g_staLostSinceMs = 0U; /* AP 模式下不再统计 STA 掉线 */
        start_http_server(); /* 启动网页服务器，固定地址 192.168.4.1 */
        Serial.println("AP_READY url=http://192.168.4.1; send p over USB for access");
    } else { /* 连热点都没开起来，极少见 */
        Serial.println("AP_FAILED"); /* 明确打印失败，避免静默 */
    }
}

/* service_network()：网络状态机，每轮 loop 调用一次，全程非阻塞
 *   已就绪但掉线      -> 宽限 NETWORK_LOST_GRACE_MS 后收摊重连，救不回来就转 AP 兜底
 *   已就绪且还连着    -> 什么都不做，直接返回
 *   连上路由器了      -> 标记 STA 就绪、启动网页、打印 IP 和信号强度
 *   超时还没连上      -> 判定失败，转 AP 兜底
 * 判断超时用的是 (millis() - 截止时间) 的无符号相减，这样即使 millis() 回绕也不会算错。
 * 【小白提示】millis() 大约 49 天回绕一次，直接比较大小会出错，必须用相减。
 *
 * 【2026-10-22 补掉线自愈】原来这一段是「只要就绪就 return」，从那以后再也不看
 *   WiFi.status()。结果热点中途关掉 / 换信道时，D1 会永远停在「以为自己还连着」
 *   的状态里：网页打不开、不会重连、不会开 AP 兜底、串口一句话都不报，
 *   从外面看就是「板子死了」，实际它自己毫无察觉。这就是本次「手机打不开」
 *   最难查的那一层。现在改成就绪之后照样盯着连接状态。 */
static void service_network()
{
    if (g_networkReady) { /* 已经就绪：但仍然要盯着「还连不连得着」 */
        if (g_apReady) { /* AP 兜底模式下没有「掉线」这回事 */
            return; /* 直接返回，保持非阻塞 */
        }
        if (WiFi.status() == WL_CONNECTED) { /* 还连着 */
            g_staLostSinceMs = 0U; /* 清掉掉线计时 */
            return; /* 直接返回，保持非阻塞 */
        }
        if (g_staLostSinceMs == 0U) { /* 刚发现掉线：先记下时刻 */
            g_staLostSinceMs = millis(); /* 从这一刻开始算宽限 */
            return; /* 给协议栈自己的自动重连留点时间 */
        }
        if (millis() - g_staLostSinceMs < NETWORK_LOST_GRACE_MS) { /* 还在宽限期内 */
            return; /* 继续等，先不打扰 */
        }
        Serial.printf("STA_LOST %lu ms; reconnecting\n", /* 明确报出来，便于现场判断 */
            static_cast<unsigned long>(millis() - g_staLostSinceMs)); /* 实参：已经掉了多久 */
        g_server.stop(); /* 先停服务器，避免重新 begin 时占着旧 socket */
        g_networkReady = false; /* 标记网络不可用，网页暂时访问不了 */
        g_staReady = false; /* 清掉 STA 就绪标记 */
        g_staLostSinceMs = 0U; /* 掉线计时清零 */
        WiFi.reconnect(); /* 主动让协议栈再连一次 */
        g_staDeadline = millis() + D1_WIFI_CONNECT_TIMEOUT_MS; /* 给这一轮连接 20 秒，超时转 AP */
        return; /* 本轮结束，下一轮由下面的「未就绪」分支接手 */
    }
    if (WiFi.status() == WL_CONNECTED) { /* 连上路由器了 */
        g_staReady = true; /* 标记 STA 就绪 */
        g_staLostSinceMs = 0U; /* 连上了，掉线计时清零 */
        start_http_server(); /* 启动网页服务器 */
        Serial.printf("STA_READY ip=%s rssi=%d\n", /* 打印 IP 与信号强度，便于判断现场信号好坏 */
            WiFi.localIP().toString().c_str(), WiFi.RSSI()); /* 实参：本机 IP / 信号强度 dBm */
        return; /* 就绪后返回 */
    }
    if (static_cast<int32_t>(millis() - g_staDeadline) >= 0) { /* 无符号相减再转有符号，毫秒回绕也判得对 */
        Serial.println("STA_TIMEOUT; starting AP fallback"); /* 打印超时原因 */
        start_ap_fallback(); /* 转 AP 兜底，保证现场没路由器也能看数据 */
    }
}

/* send_status()：处理 GET /status
 * 需要三样东西：收帧器状态（最新一帧 + 计数）、当前时间、4 个串口诊断计数。
 * 交给 status_json_with_diagnostics() 拼 JSON，返回 0 表示拼不下（≥768 字节）-> 回 500。
 * 【关键约定】凡是「无效 / 过期」的测量值都输出 null，绝不输出 0，
 *   因为 0 会被上位机当成真实测量值，画出假曲线。 */
// JSON 拼装函数与实际收帧器共用同一套主机侧测试用例。
static void send_status()
{
    /* 2026-10-21：JSON 新增 dr_x1000 与 uptime_ms 两个字段，缓冲从 768 提到 900。
     * 拼不下时 status_json 会返回 0 并回 500，宁可多留余量也不要半截 JSON。 */
    char body[900]; /* JSON 输出缓冲；放栈上且不到 1 KB，安全 */
    g_server.sendHeader("Cache-Control", "no-store"); /* 禁止缓存，保证拿到的是此刻状态 */
    /* 第二个参数传 &g_history：dR 要回看最近 32 帧，就地算窗内 rpm/V 的峰峰值
     * （与板上 OLED 轮显的 dR 同口径）。传 NULL 则该字段输出 null。 */
    if (status_json_with_diagnostics(&g_receiver, &g_history, millis(), g_uartOverflows, /* 拼 JSON：收帧器 + 帧历史 + 当前时间 + 4 个诊断计数 */
            g_uartBacklogEvents, g_uartBacklogBytes, g_externalDiscards, /* 诊断计数：积压次数 / 积压字节 / 丢弃次数 */
            body, sizeof(body)) == 0U) { /* 返回 0 表示缓冲不够拼不下，下面回 500 */
        g_server.send(500, "text/plain", "Status unavailable"); /* 拼装失败，明确告诉调用方 */
        return; /* 结束处理 */
    }
    g_server.send(200, "application/json", body); /* 正常返回 JSON */
}

/* parse_uint32_arg()：把 URL 里的参数文本安全地转成 uint32
 * 会被拒绝的情况：传入指针为空、字符串为空、有非数字字符（比如 "12a"）、
 * 后面还有多余字符、数值超过 32 位上限。
 * 只有返回 true 时，*value 里的值才可信。返回 false 的调用方一律回 400。 */
static bool parse_uint32_arg(const String &text, uint32_t *value)
{
    char *end = 0; /* strtoul 会把它指向第一个没被解析的字符 */
    unsigned long parsed; /* strtoul 的结果先放这里 */
    if (value == 0 || text.length() == 0U) { /* 输出指针为空 或 输入字符串为空，直接拒绝 */
        return false; /* 拒绝 */
    }
    parsed = strtoul(text.c_str(), &end, 10); /* 按十进制解析 */
    if (end == text.c_str() || *end != '\0' || parsed > 0xFFFFFFFFUL) { /* 一个字符都没解析 / 后面还有杂字符 / 超出 32 位 */
        return false; /* 拒绝 */
    }
    *value = (uint32_t)parsed; /* 校验通过，写回结果 */
    return true; /* 成功 */
}

/* format_raw_value()：把一帧里第 index 个测量值格式化成字符串
 * 判断依据是 validMask 的第 index 位：
 *   该位为 1 -> 输出数字
 *   该位为 0 -> 输出 null（那一刻 STM32 没采到或数据无效，不能当 0 用）
 * 6 个值的顺序（与 m2_protocol.h 一致）：
 *   0=电流mA  1=加速度Xmg  2=加速度Ymg  3=加速度Zmg  4=转速x10  5=母线电压mV
 * 其中 1~3 号槽（加速度）自 2026-10-19 起不再写入，网页只展示 0/4/5 三路。 */
static void format_raw_value(const M2Frame *frame, unsigned index,
    char *buffer, size_t capacity)
{
    if (frame == 0 || buffer == 0 || capacity == 0U) { /* 参数自检，任一为空直接返回 */
        return; /* 什么都不写 */
    }
    if ((frame->validMask & (1U << index)) == 0U) { /* 该测量值无效：validMask 对应位为 0 */
        snprintf(buffer, capacity, "null"); /* 输出 null，绝不输出 0，否则上位机会画出假曲线 */
    } else { /* 该测量值有效 */
        snprintf(buffer, capacity, "%ld", (long)frame->value[index]); /* 输出整数，单位见上方顺序表 */
    }
}

/* send_raw()：处理 GET /raw，把历史缓冲里的帧以 JSON 数组返回
 * 支持的 URL 参数：
 *   after=N   只要序号 > N 的帧（给上位机做增量拉取，避免重复拿旧数据）
 *   tail=1    只要最新的若干帧（只看最后一屏时用）
 *   （两个都不写：从最旧的开始给）
 *   limit=N   一次最多给几帧，1~32，默认 16
 * after 和 tail 互斥；参数不合法一律回 400 并说明原因，方便现场调试。
 *
 * 【为什么要分块发送】帧一多，整段 JSON 会超出内存，所以先发响应头和 [
 *   ，然后一小批一小批地 sendContent()。
 *   每发完一批就调 recv_uart(false) 去串口搬一次数据——这样 HTTP 慢吞吞传输的
 *   间隙里，STM32 发来的帧也不会被漏掉（这是本项目不掉帧的关键技巧）。
 * 【setNoDelay(true)】关掉 TCP 的 Nagle 攒包算法，让数据立刻发出去。
 *   否则数据可能在缓冲里躺 200ms，而这段时间 GPIO12 一直在收数据，很容易溢出。 */
/* 2026-10-07：先取一份稳定的 HTTP 快照，每次发送间隙都搬一次串口。 */
static void send_raw()
{
    uint32_t after = 0U; /* after 参数：只要序号大于它的帧 */
    uint32_t tail = 0U; /* tail 参数：只要最新的若干帧 */
    uint32_t requestedLimit = M2_RAW_DEFAULT_LIMIT; /* 一次返回几帧，默认 16 */
    bool hasAfter = g_server.hasArg("after"); /* 调用方有没有传 after */
    bool hasTail = g_server.hasArg("tail"); /* 调用方有没有传 tail */
    M2RawQuery query; /* 查询条件：模式 + 起点 + 条数 */
    M2RawSlice slice; /* 查询结果：命中区间 + 是否溢出/断号 */
    char oldestText[12] = "null"; /* 缓冲里最旧一帧的序号文本，取不到就 null */
    char latestText[12] = "null"; /* 缓冲里最新一帧的序号文本 */
    char header[768]; /* 响应头（JSON 前半段）拼接缓冲 */

    if (hasAfter && !parse_uint32_arg(g_server.arg("after"), &after)) { /* after 存在但解析失败，回 400 */
        g_server.send(400, "text/plain", "invalid after"); /* 明确告诉调用方哪个参数错了 */
        return; /* 结束 */
    }
    if (hasTail && (!parse_uint32_arg(g_server.arg("tail"), &tail) || tail != 1U)) { /* tail 只接受 1 */
        g_server.send(400, "text/plain", "invalid tail"); /* 参数错误 */
        return; /* 结束 */
    }
    if (hasAfter && hasTail) { /* 两个都传了：语义冲突 */
        g_server.send(400, "text/plain", "after and tail are mutually exclusive"); /* 说明冲突原因 */
        return; /* 结束 */
    }
    if (g_server.hasArg("limit") && /* limit 传了就校验 */
        (!parse_uint32_arg(g_server.arg("limit"), &requestedLimit) || /* 解析失败 */
         requestedLimit == 0U || requestedLimit > M2_RAW_MAX_LIMIT)) { /* 或者是 0、或超过上限 32 */
        g_server.send(400, "text/plain", "invalid limit"); /* 参数错误 */
        return; /* 结束 */
    }
    query.mode = hasTail ? M2_RAW_TAIL : (hasAfter ? M2_RAW_AFTER : M2_RAW_OLDEST); /* 定查询模式：取最新 / 增量 / 从最旧开始 */
    query.after = after; /* 增量起点序号 */
    query.limit = (size_t)requestedLimit; /* 最多返回几帧 */
    if (!m2h_select(&g_history, &query, &slice) || /* 在环形缓冲里算出命中区间 */
        !m2h_copy(&g_history, &slice, g_rawSnapshot, M2_RAW_MAX_LIMIT)) { /* 把命中帧复制进快照，之后发数据不再被新帧打断 */
        g_server.send(500, "text/plain", "History unavailable"); /* 查询或复制失败 */
        return; /* 结束 */
    }
    if (g_history.count != 0U) { /* 缓冲非空时算出首尾序号，给上位机做断号判断 */
        const size_t oldest = (g_history.head + M2_HISTORY_CAPACITY - g_history.count) % M2_HISTORY_CAPACITY; /* 最旧一帧在环形数组里的下标 */
        const size_t latest = (g_history.head + M2_HISTORY_CAPACITY - 1U) % M2_HISTORY_CAPACITY; /* 最新一帧在环形数组里的下标 */
        const uint32_t oldestSeq = g_history.frames[oldest].seq; /* 最旧一帧的序号 */
        const uint32_t latestSeq = g_history.frames[latest].seq; /* 最新一帧的序号 */
        snprintf(oldestText, sizeof(oldestText), "%lu", (unsigned long)oldestSeq); /* 转成文本塞进 JSON */
        snprintf(latestText, sizeof(latestText), "%lu", (unsigned long)latestSeq); /* 同上 */
    }

    const int headerLength = snprintf(header, sizeof(header), /* 拼 JSON 前半段：各种计数器 + 帧数组开头 */
        "{\"version\":\"%s\",\"link\":\"%s\",\"count\":%lu," /* version 版本 / link 链路状态 / count 本次条数 */
        "\"overflow\":%s,\"sequence_gap\":%s,\"oldest_seq\":%s,\"latest_seq\":%s," /* overflow 缓冲溢出 / sequence_gap 有断号 / 首尾序号 */
        "\"frames_total\":%lu,\"rejected\":%lu,\"reject_version\":%lu," /* frames_total 累计有效帧 / rejected 累计被拒帧 */
        "\"reject_length\":%lu,\"reject_format\":%lu,\"reject_crc\":%lu," /* 被拒原因分解：长度 / 格式 / CRC */
        "\"reject_range\":%lu,\"reject_other\":%lu,\"uart_overflows\":%lu," /* 被拒原因分解：超范围 / 其它 / 串口溢出 */
        "\"uart_backlog_events\":%lu,\"uart_backlog_bytes\":%lu," /* 串口积压次数 / 积压字节数 */
        "\"external_discards\":%lu,\"duplicates\":%lu,\"resyncs\":%lu,\"frames\":[", /* 丢弃次数 / 重复帧 / 重新同步次数；帧数组开始 */
        TELEMETRY_VERSION, /* 实参：版本号 */
        m2_link(&g_receiver, millis()) == M2_LIVE ? "live" : "stale", /* 实参 link：数据是否新鲜 */
        (unsigned long)slice.count, slice.overflow ? "true" : "false", /* 实参 count / overflow */
        slice.sequenceGap ? "true" : "false", oldestText, latestText, /* 实参 sequence_gap / oldest_seq / latest_seq */
        (unsigned long)g_receiver.accepted, (unsigned long)g_receiver.rejected, /* 实参 frames_total / rejected */
        (unsigned long)g_receiver.rejectVersion, (unsigned long)g_receiver.rejectLength, /* 实参 reject_version / reject_length */
        (unsigned long)g_receiver.rejectFormat, (unsigned long)g_receiver.rejectCrc, /* 实参 reject_format / reject_crc */
        (unsigned long)g_receiver.rejectRange, (unsigned long)g_receiver.rejectOther, /* 实参 reject_range / reject_other */
        (unsigned long)g_uartOverflows, (unsigned long)g_uartBacklogEvents, /* 实参 uart_overflows / uart_backlog_events */
        (unsigned long)g_uartBacklogBytes, (unsigned long)g_externalDiscards, /* 实参 uart_backlog_bytes / external_discards */
        (unsigned long)g_receiver.duplicates, (unsigned long)g_receiver.resyncs); /* 实参 duplicates / resyncs，实参到此结束 */
    if (headerLength < 0 || static_cast<size_t>(headerLength) >= sizeof(header)) { /* 拼不下会被截断，回 500 */
        g_server.send(500, "text/plain", "Raw header unavailable"); /* 明确报错 */
        return; /* 结束 */
    }
    size_t chunkLength = static_cast<size_t>(headerLength); /* 分块缓冲当前已填长度，先放入 JSON 前半段 */
    memcpy(g_rawChunk, header, chunkLength); /* 把 header 拷进分块缓冲 */
// GPIO12 正在收 STM32 遥测时，避免 TCP 把小的写入延迟发出。
    g_server.client().setNoDelay(true); /* 关掉 Nagle 攒包，数据立刻发出，避免 GPIO12 期间积压 */
    g_server.setContentLength(CONTENT_LENGTH_UNKNOWN); /* 长度未知 -> 用分块传输编码 */
    g_server.sendHeader("Cache-Control", "no-store"); /* 长响应同样禁止缓存 */
    g_server.send(200, "application/json", ""); /* 先发响应头和空体，真正内容随后分块发 */
    recv_uart(false); /* 传输间隙搬一次串口，防止 STM32 的帧被漏掉 */
    for (size_t offset = 0U; offset < slice.count; ++offset) { /* 逐帧拼 JSON 并分批发送 */
        const M2Frame *frame = &g_rawSnapshot[offset]; /* 当前这一帧，取自快照，不受新数据影响 */
        char values[6][16]; /* 6 个测量值各自的文本，每个最多 15 字符 */
        char item[384]; /* 单帧的 JSON 文本 */
        for (unsigned valueIndex = 0U; valueIndex < 6U; ++valueIndex) { /* 把 6 个值都转成文本，无效位输出 null */
            format_raw_value(frame, valueIndex, values[valueIndex], sizeof(values[valueIndex])); /* 单个值转换 */
        }
        const int itemLength = snprintf(item, sizeof(item), /* 拼这一帧的 JSON 对象 */
            "%s{\"seq\":%lu,\"uptime_ms\":%lu,\"level\":%u,\"duty\":%u," /* 分隔逗号 / 序号 / 上电毫秒 / 档位 / 占空比 */
            "\"feature_valid_mask\":%u,\"ai_state\":%u,\"current_mA\":%s,\"accel_x_mg\":%s," /* 特征有效位 / AI 状态 / 电流 / X 加速度 */
            "\"accel_y_mg\":%s,\"accel_z_mg\":%s,\"rpm_x10\":%s,\"battery_mV\":%s}", /* Y 加速度 / Z 加速度 / 转速 / 母线电压 */
            offset == 0U ? "" : ",", (unsigned long)frame->seq, /* 第一帧前不加逗号，之后每帧前加 */
            (unsigned long)frame->uptimeMs, (unsigned)frame->level, (unsigned)frame->duty, /* 实参 uptime_ms / level / duty */
            (unsigned)frame->validMask, (unsigned)frame->aiState, /* 实参 feature_valid_mask / ai_state */
            values[0], values[1], values[2], values[3], /* 实参 current_mA / accel_x / accel_y / accel_z */
            values[4], values[5]); /* 实参 rpm_x10 / battery_mV */
        if (itemLength < 0 || static_cast<size_t>(itemLength) >= sizeof(item)) { /* 单帧文本超长，断开连接，不冒险发半截 JSON */
            g_server.client().stop(); /* 主动断开，让上位机重试 */
            return; /* 结束 */
        }
        const size_t length = static_cast<size_t>(itemLength); /* 本帧文本长度 */
        if (chunkLength + length > sizeof(g_rawChunk)) { /* 缓冲快满了：先发出已拼的，再继续拼 */
            g_server.sendContent(g_rawChunk, chunkLength); /* 发出这一批 */
            recv_uart(false); /* 发送间隙搬一次串口 */
            chunkLength = 0U; /* 缓冲复位，重新开始拼 */
        }
        memcpy(g_rawChunk + chunkLength, item, length); /* 把本帧追加到分块缓冲 */
        chunkLength += length; /* 更新已填长度 */
        recv_uart(false); /* 每帧之间都搬一次串口，这是不掉帧的关键 */
    }
    g_server.sendContent(g_rawChunk, chunkLength); /* 发送最后残留的一批 */
    recv_uart(false); /* 再搬一次串口 */
    g_server.sendContent("]}"); /* 收尾：关闭帧数组和整份 JSON */
    recv_uart(false); /* 响应结束前最后搬一次 */
}

/* setup()：上电只跑一次的初始化
 * 顺序是刻意安排的：
 *   ① 开 USB 日志串口 -> 打印固件版本、芯片 ID、CPU 频率自检
 *   ② 开软件串口收 STM32 数据（并检查是否初始化成功）
 *   ③ 注册 1ms 周期回调，保证空闲时也在收数据（这就是主循环之外的收数通道）
 *   ④ 连 WiFi（STA），设 20 秒超时，超时交给 service_network() 转 AP
 * 【CPU 频率自检】软件串口的波特率是拿 CPU 频率算出来的，
 *   频率不对 -> 波特率就偏 -> 收到的全是错帧，所以这里要显式检查并打印。 */
void setup()
{
    Serial.begin(115200); /* 开 USB 日志串口，与电脑通信并接收命令 */
    Serial.printf("\nFW=%s CHIP=%06X CONFIG_CRYSTAL_MHZ=%u\n", TELEMETRY_VERSION, /* 开机横幅：固件版本 / 芯片 ID / 配置的晶振频率 */
        ESP.getChipId(), static_cast<unsigned>(F_CRYSTAL / 1000000U)); /* 实参：芯片 ID / 晶振频率 MHz */
// 2026-10-07：开机时把 CPU 实际频率与波特率时序一起打印出来对比。
    const unsigned cpuMHz = ESP.getCpuFreqMHz(); /* 实际 CPU 频率 */
    const unsigned expectedCpuMHz = F_CPU / 1000000U; /* 编译期预期频率 */
    Serial.printf("CPU_MHZ=%u EXPECTED_CPU_MHZ=%u\n", cpuMHz, expectedCpuMHz); /* 两者都打印，便于对比 */
    if (cpuMHz != expectedCpuMHz) { /* 频率不一致则波特率一定偏，必须明确报警 */
        Serial.println("CPU_FREQUENCY_FAILED"); /* 失败标记，上位机会抓这个字符串 */
    }
// TX=-1 且收发引脚不同，避免进入单线模式；永远不会去驱动 STM32 的输入。
// 沿用已验证的边沿时间戳接收方式与 D6 接线，波特率 57600。
    g_uart.begin(STM32_UART_BAUD, SWSERIAL_8N1, 12, -1, false, 512, 2048);  // 57600 8N1，RX=GPIO12(D6)，TX=-1 表示不发；接收缓冲 512 字节
    g_uart.enableTx(false);  // 明确禁止发送，永远不会去驱动 STM32 的引脚
    g_uart.enableRxGPIOPullUp(false); /* 关掉内部上拉：外部已有上下拉，避免重复 */
    if (!g_uart) { /* 软件串口初始化失败，极少见 */
        Serial.println("UART_FAILED"); /* 明确标记失败 */
    }
    Serial.printf("STM32_UART_BAUD=%lu\n", static_cast<unsigned long>(STM32_UART_BAUD)); /* 打印实际启用的波特率 */
// 2026-10-07：在 HTTP / WiFi 的让出点去服务接收缓冲。
// 该回调运行在 CONT 上下文而不是中断里；recv_uart 内禁止让出或延时。
    if (!schedule_recurrent_function_us([]() {  // 注册 1ms 周期回调：即使主循环在忙，串口也在被搬运 /* 注册 1ms 周期回调：主循环忙时串口照样被搬运 */
            recv_uart();
            return true; /* 返回 true 表示继续周期执行 */
        }, 1000U)) { /* 周期 1000 微秒即 1ms */
        Serial.println("UART_SERVICE_FAILED"); /* 注册失败 */
    } else { /* 注册成功 */
        Serial.println("UART_SERVICE_US=1000"); /* 打印周期，确认生效 */
    }
    WiFi.persistent(false); /* 不让 WiFi 参数写 Flash */
    WiFi.setAutoReconnect(true); /* 断线自动重连 */
    if (!WiFi.mode(WIFI_STA)) { /* 切 STA 模式；失败转 AP 兜底 */
        Serial.println("STA_MODE_FAILED; starting AP fallback"); /* 打印原因 */
        start_ap_fallback(); /* 转 AP */
        return; /* setup 直接结束，已经靠 AP 模式跑起来 */
    }
// 2026-10-07：GPIO12 收 UART 期间让 STA 保持清醒、不休眠。
// CRC 仍会拒掉时序错误的帧；该设置在开机自检里会再确认一次。
    const WiFiSleepType_t previousSleep = WiFi.getSleepMode(); /* 记下原先的睡眠设置，便于对比 */
    if (!WiFi.setSleepMode(WIFI_NONE_SLEEP) || WiFi.getSleepMode() != WIFI_NONE_SLEEP) {
        Serial.println("WIFI_SLEEP_FAILED"); /* 设成不休眠失败，明确报警 */
    } else { /* 设置成功 */
        Serial.printf("UART_WIFI_SLEEP=%u PREVIOUS_SLEEP=%u\n", /* 打印新设置和原设置，便于对比 */
            static_cast<unsigned>(WiFi.getSleepMode()), static_cast<unsigned>(previousSleep)); /* 实参：当前睡眠模式 / 原先睡眠模式 */
    }
    WiFi.begin(D1_WIFI_SSID, D1_WIFI_PASSWORD); /* 开始连接路由器 */
    g_staDeadline = millis() + D1_WIFI_CONNECT_TIMEOUT_MS; /* 20 秒超时，超时由 service_network() 转 AP */
    Serial.printf("STA_CONNECT ssid=%s timeout_ms=%lu\n", D1_WIFI_SSID, /* 打印正在连哪个热点，不含密码 */
        D1_WIFI_CONNECT_TIMEOUT_MS); /* 超时毫秒数 */
}

/* usb_send_frame()：抓包模式下，把一帧原样编码成 42 字节发给电脑
 * 三重保险，宁可丢一帧也绝不阻塞主循环：
 *   编码失败            -> encode_errors++（不看数据）
 *   USB 发送缓冲不够    -> dropped++（不等待，直接丢）
 *   实际写入字节数不对  -> write_errors++
 * 只在主循环里被调用（不在中断里），所以这里可以放心用阻塞式 Serial.write。
 * 抓包期间不改动任何测量值的有效性，原样转发，保证数据可用于离线分析。 */
/* 2026-10-07：在受保护的主循环 / CONT 上下文里做带检查的 FIFO 写入，绝不在中断里。
 * 抓包期间没有别的 USB 写入者；不等待、不刷新、不改动数据的有效性。 */
static void usb_send_frame(const M2Frame &frame)
{
    uint8_t bytes[M2_FRAME_SIZE]; /* 编码后的 42 字节帧，放栈上 */
    ++g_usbSeen; /* 见到一帧就计数，不管后面发不发得出去 */
    g_usbLastFrameMs = millis(); /* 记时刻，用于源端超时判断 */
    if (m2_encode(&frame, bytes, sizeof(bytes)) != sizeof(bytes)) { /* 重新编码；长度不对说明数据有问题 */
        ++g_usbEncodeErrors; /* 编码失败计数 */
        return; /* 宁可丢一帧，也不发垃圾数据 */
    }
    if (Serial.availableForWrite() < static_cast<int>(sizeof(bytes))) { /* USB 写缓冲不够 */
        ++g_usbDropped; /* 记「丢弃」而不是等待，绝不阻塞主循环 */
        return; /* 结束 */
    }
    if (Serial.write(bytes, sizeof(bytes)) != sizeof(bytes)) { /* 实际写入字节数不符 */
        ++g_usbWriteErrors; /* 写入错误计数 */
        return; /* 结束 */
    }
    ++g_usbSent; /* 成功转发一帧 */
}

/* recv_uart()：从软件串口搬数据 + 喂给收帧状态机（整个程序的心脏）
 * 每 1ms 至少被调用一次（loop 里一次 + 1ms 周期回调一次），两处调用都安全。
 *
 * 参数 waitFrame：
 *   true  = 在 9ms 上限内尽量多搬（正常情况）
 *   false = 搬一轮就走，用于 HTTP 分块发送的间隙（不能长时间占着 CPU）
 *
 * 完整流程：
 *   1. 重入保护：如果上一次还没收完（回调又进来）就直接返回，避免状态错乱
 *   2. 在 UART_FRAME_WAIT_US(9ms) 内反复 read()，最多攒 512 字节
 *   3. 检查软件串口 FIFO 是否溢出：溢出说明数据确实丢过 -> 重置状态机 + 计数
 *      顺带统计「一次读到超过一帧的数据」= 上游有积压，说明我们搬得太慢
 *   4. 每收到一个字节就喂给 m2_feed()：它负责找同步头 A5 5A、收满 42 字节、
 *      校验版本/长度/CRC/数值范围；凑齐一帧就：
 *        抓包中 -> usb_send_frame() 转发给电脑
 *        无论何时 -> m2h_push() 存进历史缓冲（网页 /raw 靠它）
 *   5. 缓冲读空（used==0）就收工
 *   6. 半截帧超过 30ms 没有新字节 -> 判定源端断了，丢弃重来（防止永久卡在半帧）
 *   7. 一直读满 9ms 还没收完 -> 计一次 wait_limits（说明 CPU 太忙或源端发太快）
 * 【禁止事项】这个函数里不能 delay() / yield() / 打印日志，
 *   因为它在定时回调上下文里被调用，一让出 CPU 就可能打乱软件串口的位采样。 */
/* 2026-10-07：沿用已验证的「9ms / 512 字节」上限，保留半截帧。
 * 只有连续 30ms 收不到字节才判定半帧过期。批量读取过程中不让出 CPU，
 * 中断保持使能。HTTP 侧只搬一轮、不等待。 */
static void recv_uart(bool waitFrame)
{
    if (g_uartReceiving) { /* 重入保护：上一次还没收完，直接返回 */
        return; /* 避免状态错乱 */
    }
    g_uartReceiving = true; /* 标记「正在搬数据」 */
    const uint32_t startedUs = micros(); /* 记住开始时刻，用来卡 9ms 上限 */
    size_t total = 0U; /* 本轮已搬的字节数 */
    while (total < sizeof(g_uartBytes) && micros() - startedUs < UART_FRAME_WAIT_US) { /* 缓冲没满 且 没超 9ms 就继续搬 */
        const int received = g_uart.read(g_uartBytes, sizeof(g_uartBytes) - total); /* 一次读一批，最多填满缓冲 */
        if (g_uart.overflow()) { /* 软件串口 FIFO 溢出 = 数据确实丢过 */
            ++g_uartOverflows; /* 溢出计数 */
            ++g_externalDiscards; /* 这批数据全部作废 */
            m2_reset(&g_receiver); /* 重置状态机，重新找帧同步头 */
            g_uartReceiving = false; /* 解除重入标记 */
            return; /* 直接返回 */
        }
        if (received > static_cast<int>(M2_FRAME_SIZE)) { /* 一次读到超过一帧 = 上游有积压 */
            ++g_uartBacklogEvents; /* 积压次数 */
            g_uartBacklogBytes += static_cast<uint32_t>(received - static_cast<int>(M2_FRAME_SIZE)); /* 多出来的字节数累计 */
        }
        total += static_cast<size_t>(received); /* 累计本轮搬的字节数 */
        if (received > 0) { /* 这次确实读到了字节 */
            g_uartLastByteUs = micros(); /* 更新「最近收到字节的时刻」 */
        }
        for (int i = 0; i < received; ++i) { /* 逐字节喂给收帧状态机 */
            if (m2_feed(&g_receiver, g_uartBytes[i], millis())) { /* 返回 true = 凑齐一帧 */
                if (g_usbActive) { /* 抓包模式下转发给电脑 */
                    usb_send_frame(g_receiver.latest); /* 原样编码转发，三重保险防阻塞 */
                }
                m2h_push(&g_history, &g_receiver.latest); /* 无论抓不抓包，都存进历史缓冲供 /raw 查询 */
            }
        }
        if (g_receiver.used == 0U) { /* 缓冲读空 = 这轮干完了 */
            g_uartReceiving = false; /* 解除重入标记 */
            return; /* 提前收工 */
        }
        if (received == 0 && micros() - g_uartLastByteUs >= UART_PARTIAL_IDLE_US) { /* 没读到新字节且已静默 30ms */
            ++g_uartPartialTimeouts; /* 半帧超时计数 */
            ++g_externalDiscards; /* 丢掉这半截 */
            m2_discard(&g_receiver); /* 清空接收进度，等下一帧重新开始 */
            g_uartReceiving = false; /* 解除重入标记 */
            return; /* 返回 */
        }
        if (!waitFrame) { /* 调用方要求「搬一轮就走」 */
            g_uartReceiving = false; /* 解除重入标记 */
            return; /* 立刻返回，不占 CPU */
        }
    }
    if (g_receiver.used != 0U) { /* 读到 9ms 上限仍有半截帧没收完 */
        ++g_uartWaitLimits; /* 计一次「等超时」：CPU 太忙或源端发太快 */
    }
    g_uartReceiving = false; /* 正常出口：解除重入标记 */
}

/* usb_print_ready()：本地就绪自检（USB 串口敲 ?）
 * 打印：固件版本、芯片 ID、CPU 频率、波特率、网络是否就绪、链路状态(live/stale)、
 *       特征有效位、当前档位、占空比、转速。
 * 用途：现场一眼确认「板子活着没 / STM32 在不在发 / 链路通不通」。
 * 【安全】输出里不含任何 WiFi 密码或凭据。 */
/* 2026-10-07：本地就绪自检查询，输出里不含任何访问凭据。 */
static void usb_print_ready()
{
    Serial.printf("USB_READY {\"version\":\"%s\",\"chip\":\"%06X\",\"cpu_mhz\":%u," /* 就绪信息 JSON：版本 / 芯片 / CPU 频率 */
        "\"baud\":%lu,\"ready\":%s,\"link\":\"%s\",\"feature_valid_mask\":%u," /* 波特率 / 网络就绪 / 链路状态 / 特征有效位 */
        "\"level\":%u,\"duty\":%u,\"rpm_x10\":%ld}\n", TELEMETRY_VERSION, /* 档位 / 占空比 / 转速；不含任何凭据 */
        ESP.getChipId(), ESP.getCpuFreqMHz(), static_cast<unsigned long>(STM32_UART_BAUD), /* 实参：芯片 ID / CPU 频率 / 波特率 */
        g_networkReady ? "true" : "false", m2_link(&g_receiver, millis()) == M2_LIVE ? "live" : "stale", /* 实参：网络就绪 / 链路状态 */
        g_receiver.latest.validMask, g_receiver.latest.level, g_receiver.latest.duty, /* 实参：特征有效位 / 档位 / 占空比 */
        static_cast<long>(g_receiver.latest.value[4])); /* 实参：转速 x10 */
}

/* usb_start_capture()：开始 60 秒原始抓包（USB 串口敲 u）
 * 前置条件（不满足直接回 USB_ERROR not_ready）：
 *   ① 网络已就绪（说明初始化都过了）  ② 链路是 live（说明 STM32 正在正常发帧）
 * 动作顺序：
 *   关 WiFi（理由见 suspend_network_for_capture）
 *   -> 再搬一次串口，把残留的半帧清掉，保证抓到的是干净起点
 *   -> 打印 USB_BEGIN 标记（电脑端看到这行才开始存文件）
 *   -> 把所有计数器记一个「起点值」(xxxStart)
 * 记起点值是为了最后算「本次抓包区间内的增量」：用无符号相减，
 * 即使计数器中途回绕，算出来的增量依然正确。 */
/* 2026-10-12：在打印开始标记之前关掉 WiFi，这样它的调度器
 * 无法在严格抓包期间打扰 GPIO12 的软件串口。
 * 每个被拒类别都与总数在同一段时间区间内取起点快照。
 * 固定 60 秒上限保证即使上位机掉线也能恢复网络。 */
static void usb_start_capture()
{
    if (!g_networkReady || m2_link(&g_receiver, millis()) != M2_LIVE) { /* 前置条件：网络就绪 且 链路 live */
        Serial.println("USB_ERROR not_ready"); /* 不满足就明确拒绝，不去抓一堆废数据 */
        return; /* 结束 */
    }
    suspend_network_for_capture(); /* 先关 WiFi，理由见上面的 suspend_network_for_capture */
    recv_uart(); /* 再搬一次串口，清掉残留半帧，保证起点干净 */
    Serial.printf("USB_BEGIN {\"version\":\"%s\",\"duration_ms\":%lu,\"frame_size\":%u}\n", /* 打印开始标记：电脑端看到这行才开始存文件 */
        TELEMETRY_VERSION, static_cast<unsigned long>(USB_CAPTURE_MS), M2_FRAME_SIZE); /* 实参：版本 / 时长 / 帧长 */
    g_usbSeen = 0U; /* 本次统计清零 */
    g_usbSent = 0U; /* 同上 */
    g_usbDropped = 0U; /* 同上 */
    g_usbEncodeErrors = 0U; /* 同上 */
    g_usbWriteErrors = 0U; /* 同上 */
    g_usbRejectedStart = g_receiver.rejected; /* 记各类计数器的起点值，结束时相减得本次增量 */
    g_usbRejectVersionStart = g_receiver.rejectVersion; /* 起点值：版本号被拒 */
    g_usbRejectLengthStart = g_receiver.rejectLength; /* 起点值：帧长被拒 */
    g_usbRejectFormatStart = g_receiver.rejectFormat; /* 起点值：帧头格式被拒 */
    g_usbRejectCrcStart = g_receiver.rejectCrc; /* 起点值：CRC 被拒 */
    g_usbRejectRangeStart = g_receiver.rejectRange; /* 起点值：数值超范围被拒 */
    g_usbRejectOtherStart = g_receiver.rejectOther; /* 起点值：其它原因被拒 */
    g_usbOverflowStart = g_uartOverflows; /* 起点值：串口溢出 */
    g_usbDiscardStart = g_externalDiscards; /* 起点值：整批丢弃 */
    g_usbPartialStart = g_uartPartialTimeouts; /* 起点值：半帧超时 */
    g_usbDuplicateStart = g_receiver.duplicates; /* 起点值：重复帧 */
    g_usbResyncStart = g_receiver.resyncs; /* 起点值：重新同步 */
    g_usbStartedMs = millis(); /* 抓包开始时刻 */
    g_usbLastFrameMs = g_usbStartedMs; /* 最近收帧时刻从这一刻起算 */
    g_usbActive = true; /* 正式进入抓包模式 */
}

/* usb_stop_capture()：结束抓包
 * 触发条件有 4 种：电脑敲 s / 源端 3 秒没数据 / 满 60 秒 / （网络恢复流程兜底）
 * 顺序：先把 g_usbActive 置 false，再打印文字。
 *   必须这样！否则二进制帧和文本日志会混在一起，电脑端解析就乱了。
 * 最后打印 USB_END JSON，报本次区间增量，用来判断丢帧原因：
 *   dropped / write_errors 大   -> 电脑侧读得慢，USB 缓冲撑爆
 *   source_reject_crc 大        -> 串口时序有问题（WiFi 干扰、波特率偏差、线太长）
 *   uart_overflows 大           -> 软件串口 FIFO 溢出，CPU 被别的活占太久
 *   partial_timeouts 大         -> 对方发一半就不发了（复位 / 断线）
 * 结尾安排 2 秒后恢复网络，等 USB 缓冲里的数据先排空，避免开 WiFi 时抢中断。 */
/* 2026-10-07：先关掉二进制模式，再在打印文本之前取各类计数的差值快照。
 * 用无符号相减，即使 uint32 计数器回绕也算得对。
 * 二进制模式期间命令与网络服务不会打印任何凭据。 */
static void usb_stop_capture(const char *reason)
{
    char report[832]; /* 结束报告缓冲；放栈上但只有 832 字节，安全 */
    g_usbActive = false; /* 先退出抓包模式，否则二进制帧会和下面的文本混在一起 */
    const int length = snprintf(report, sizeof(report), /* 拼结束报告 JSON */
        "USB_END {\"version\":\"%s\",\"reason\":\"%s\",\"elapsed_ms\":%lu," /* 版本 / 结束原因 / 持续时长 */
        "\"seen\":%lu,\"sent\":%lu,\"dropped\":%lu,\"encode_errors\":%lu," /* 见到 / 发出 / 丢弃 / 编码失败 */
        "\"write_errors\":%lu,\"source_rejected\":%lu," /* USB 写入失败 / 源端被拒总数 */
        "\"source_reject_version\":%lu,\"source_reject_length\":%lu," /* 被拒分解：版本 / 长度 */
        "\"source_reject_format\":%lu,\"source_reject_crc\":%lu," /* 被拒分解：格式 / CRC */
        "\"source_reject_range\":%lu,\"source_reject_other\":%lu,\"uart_overflows\":%lu," /* 被拒分解：超范围 / 其它 / 串口溢出 */
        "\"external_discards\":%lu,\"partial_timeouts\":%lu,\"duplicates\":%lu,\"resyncs\":%lu}\n", /* 丢弃 / 半帧超时 / 重复 / 重同步 */
        TELEMETRY_VERSION, reason, static_cast<unsigned long>(millis() - g_usbStartedMs), /* 实参：版本 / 原因 / 实际时长 */
        static_cast<unsigned long>(g_usbSeen), static_cast<unsigned long>(g_usbSent), /* 实参：seen / sent */
        static_cast<unsigned long>(g_usbDropped), static_cast<unsigned long>(g_usbEncodeErrors), /* 实参：dropped / encode_errors */
        static_cast<unsigned long>(g_usbWriteErrors), /* 实参：write_errors */
        static_cast<unsigned long>(g_receiver.rejected - g_usbRejectedStart), /* 用无符号相减算增量，计数器回绕也算得对 */
        static_cast<unsigned long>(g_receiver.rejectVersion - g_usbRejectVersionStart), /* 增量：版本号被拒 */
        static_cast<unsigned long>(g_receiver.rejectLength - g_usbRejectLengthStart), /* 增量：帧长被拒 */
        static_cast<unsigned long>(g_receiver.rejectFormat - g_usbRejectFormatStart), /* 增量：格式被拒 */
        static_cast<unsigned long>(g_receiver.rejectCrc - g_usbRejectCrcStart), /* 增量：CRC 被拒 */
        static_cast<unsigned long>(g_receiver.rejectRange - g_usbRejectRangeStart), /* 增量：数值超范围被拒 */
        static_cast<unsigned long>(g_receiver.rejectOther - g_usbRejectOtherStart), /* 增量：其它原因被拒 */
        static_cast<unsigned long>(g_uartOverflows - g_usbOverflowStart), /* 增量：串口 FIFO 溢出，CPU 被占太久 */
        static_cast<unsigned long>(g_externalDiscards - g_usbDiscardStart), /* 增量：整批丢弃 */
        static_cast<unsigned long>(g_uartPartialTimeouts - g_usbPartialStart), /* 增量：半帧超时，对方发一半就停 */
        static_cast<unsigned long>(g_receiver.duplicates - g_usbDuplicateStart), /* 增量：重复帧 */
        static_cast<unsigned long>(g_receiver.resyncs - g_usbResyncStart)); /* 增量：重新同步，帧头错位后重找 */
    if (length > 0 && static_cast<size_t>(length) < sizeof(report)) { /* 拼成功且没被截断 */
        Serial.write(reinterpret_cast<const uint8_t *>(report), static_cast<size_t>(length)); /* 一次性写出，减少分次调用 */
    } else { /* 拼失败，几乎不可能 */
        Serial.println("USB_ERROR report"); /* 兜底错误标记 */
    }
    g_networkResumeAtMs = millis() + NETWORK_RESUME_DELAY_MS; /* 排定 2 秒后恢复网络，让 USB 缓冲先排空 */
    g_networkResumePending = true; /* 交给 loop() 去执行恢复 */
}

/* loop()：主循环。每轮都尽量短（约 1ms 一轮），三种状态互斥：
 *   ① 抓包中：只搬串口 + 检查结束条件（敲 s / 源端超时 / 满 60 秒），
 *             不处理网页请求、不打印日志——这样才能保证串口时序干净
 *   ② 等待恢复网络：等到 NETWORK_RESUME_DELAY_MS 到了就调 resume_network_after_capture()
 *   ③ 正常：service_network() 推进连接 -> handleClient() 处理网页 -> 处理 USB 命令
 * USB 命令（单字符）：u=开始抓包   ?=打印就绪信息   p=打印访问方式（含 AP 密码）
 * 每轮结尾 delay(1)：一方面让 WiFi 协议栈有机会跑，另一方面给软件串口的中断留时间。
 * 【别删这个 delay(1)】删了会出现「网页能打开但数据全是 null」这类怪问题。 */
// 每轮限制输入处理量，让 HTTP 与 WiFi 调度器也能推进。
void loop()
{
    recv_uart(); /* 每轮先搬一次串口，默认 9ms 内尽量多搬 */
    if (g_usbActive) { /* 状态①：抓包中 */
        if (Serial.available() > 0 && Serial.read() == 's') { /* 电脑敲了 s 主动结束 */
            usb_stop_capture("host_stop"); /* 记录结束原因 */
        } else if (millis() - g_usbLastFrameMs >= M2_TIMEOUT_MS) { /* 源端 3 秒没发数据则视为断了 */
            usb_stop_capture("source_timeout"); /* 结束并说明原因 */
        } else if (millis() - g_usbStartedMs >= USB_CAPTURE_MS) { /* 或者满了 60 秒上限 */
            usb_stop_capture("duration"); /* 结束 */
        }
// 只有抓包期间才屏蔽 HTTP 处理与连接提示日志。
        delay(1);
        return; /* 抓包期间直接跳过后面所有网络与命令处理 */
    }
    if (g_networkResumePending) { /* 状态②：抓包刚结束，等延时后恢复网络 */
        if (static_cast<int32_t>(millis() - g_networkResumeAtMs) < 0) { /* 时间还没到；无符号相减转有符号，回绕也安全 */
            delay(1); /* 让出 CPU，等下一轮 */
            return; /* 本轮什么都不做 */
        }
        resume_network_after_capture(); /* 时间到了，恢复 WiFi 和网页 */
    }
    service_network(); /* 状态③：推进网络连接状态机，非阻塞 */
    if (g_networkReady) { /* 网络可用时才处理网页请求 */
        g_server.handleClient(); /* 处理一次 HTTP 请求 */
    }
    if (Serial.available() > 0) { /* 处理 USB 单字符命令 */
        const int command = Serial.read(); /* 读一个字符 */
        if (command == 'u') { /* u = 开始抓包 */
            usb_start_capture(); /* 启动 60 秒原始抓包 */
        } else if (command == '?') { /* ? = 打印就绪信息 */
            usb_print_ready(); /* 本地自检输出 */
        } else if (command == 'p') { /* p = 打印访问方式，含 AP 密码 */
            print_access(); /* 只在本地 USB 显式请求时才打印密码 */
        }
    }
    delay(1); /* 每轮结束让出 1ms：给 WiFi 协议栈和串口中断留时间 */
}
