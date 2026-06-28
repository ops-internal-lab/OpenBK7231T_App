#include "dash_frontend.h"
#include "../new_common.h"  /* defines rtos_delay_milliseconds per platform */
#if !PLATFORM_ESPIDF
#include "rtos_pub.h"       /* Beken/BK7231 only — ESP32 uses vTaskDelay via new_common.h */
#endif
#include "dash_gz.h"  // pre-gzipped /dash page (g_dashGz / DASH_GZ_LEN)
#include <string.h>   // strstr

// Serves the dashboard page (HTML/CSS/JS) at /dash.
// Live data is fetched client-side from /api_dash (core/energy/bms/net/chginv),
// implemented in drv_bl_shared.c. iOS 5 Safari compatible: XHR + base64 binary
// (no fetch/color-mix/clamp/grid/aspect-ratio), -webkit-box flex fallbacks.

int http_fn_custom_dash(http_request_t *request) {
    // Serve the pre-gzipped dashboard when the client accepts gzip (every
    // browser, incl. iOS Safari, does). Regenerate dash_gz.h after edits.
    if (request->received != NULL && strstr(request->received, "gzip") != NULL) {
        poststr(request, "HTTP/1.1 200 OK\r\n");
        poststr(request, "Content-Type: text/html\r\n");
        poststr(request, "Content-Encoding: gzip\r\n");
        poststr(request, "Cache-Control: max-age=86400\r\n");
        poststr(request, "Connection: close\r\n");
        poststr(request, "\r\n");
        postany(request, (const char*)g_dashGz, DASH_GZ_LEN);
        poststr(request, NULL);
        return 0;
    }
    http_setup(request, "text/html");

    // --- CSS part 1: shell, top-stats, columns ---
    poststr(request,
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<meta name='apple-mobile-web-app-capable' content='yes'>"
        "<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent'>"
        "<title>My Dashboard</title>"
        "<style>"
        "html,body{height:100%;margin:0;background:#000;}"
        "#dash-container{max-width:1200px;width:100%;margin:0 auto;min-height:100%;min-height:100vh;background:#121212;padding:10px 20px 20px;-webkit-box-sizing:border-box;box-sizing:border-box;font-family:-apple-system,sans-serif;color:#eee;position:relative;}"
        ".top-stats{display:-webkit-box;-webkit-box-orient:horizontal;-webkit-box-align:center;-webkit-box-pack:justify;display:-webkit-flex;display:flex;-webkit-justify-content:space-between;justify-content:space-between;-webkit-align-items:center;align-items:center;background:#222;padding:18px;border-radius:8px;text-align:center;margin-top:15px;width:100%;-webkit-box-sizing:border-box;box-sizing:border-box;white-space:nowrap;}"
        ".top-stats div{display:-webkit-box;-webkit-box-orient:vertical;-webkit-box-align:stretch;-webkit-box-pack:center;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;-webkit-justify-content:center;justify-content:center;margin:0 10px;}"
        ".top-stats label{color:#888;font-size:20px;text-transform:uppercase;margin-bottom:6px;display:block;}"
        ".top-stats b{font-size:38px;font-weight:600;}"
        ".c-exp{color:#4caf50;}.c-imp{color:#f44336;}"
        ".dash-row{display:-webkit-box;-webkit-box-orient:horizontal;-webkit-box-align:stretch;display:-webkit-flex;display:flex;margin-top:15px;-webkit-align-items:stretch;align-items:stretch;}"
        ".left-col{-webkit-box-flex:0;-webkit-flex:0 0 250px;flex:0 0 250px;width:250px;background:#222;padding:12px 14px;border-radius:8px;margin-right:15px;-webkit-box-sizing:border-box;box-sizing:border-box;}"
        ".center-col{-webkit-box-flex:1;-webkit-flex:1;flex:1;display:-webkit-box;-webkit-box-orient:vertical;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;min-width:0;}"
        ".right-col{-webkit-box-flex:0;-webkit-flex:0 0 210px;flex:0 0 210px;width:210px;margin-left:15px;display:-webkit-box;-webkit-box-orient:vertical;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;-webkit-box-sizing:border-box;box-sizing:border-box;}"
    );
    rtos_delay_milliseconds(1);

    // --- CSS part 2: left table, graph, clock ---
    poststr(request,
        ".sep-lbl{font-size:12px;color:#888;text-transform:uppercase;letter-spacing:.5px;margin:0 0 8px;}"
        ".grp-lbl{font-size:12px;color:#888;text-transform:uppercase;letter-spacing:.5px;margin:18px 0 8px;}"
        ".reserved{height:46px;border:1px dashed #333;border-radius:6px;}"
        ".etbl{width:100%;border-collapse:collapse;font-size:14px;}"
        ".etbl th{font-size:10px;color:#888;font-weight:600;text-transform:uppercase;letter-spacing:.4px;padding:0 0 6px;border-bottom:1px solid #333;text-align:right;}"
        ".etbl th.nm{text-align:left;}"
        ".etbl td{padding:6px 0;border-bottom:1px solid #2c2c2c;color:#cfcfcf;}"
        ".etbl td.num{text-align:right;font-family:'Courier New',monospace;font-weight:bold;font-size:13px;width:60px;}"
        ".imp{color:#D97757;}.exp{color:#4caf50;}"
        ".unit{font-size:9px;color:#666;}"
        ".etbl tr.tot td{border-bottom:none;border-top:1px solid #444;padding-top:8px;font-weight:bold;}"
        ".etbl tr.tot td.nm{color:#fff;text-transform:uppercase;font-size:12px;}"
        ".leg-box{margin-top:18px;font-size:14px;color:#eee;padding:15px;background:#1a1a1a;border-radius:6px;border:1px solid #333;}"
        ".leg-row{display:-webkit-box;-webkit-box-orient:horizontal;-webkit-box-align:center;display:-webkit-flex;display:flex;-webkit-align-items:center;align-items:center;margin-bottom:10px;}"
        ".leg-swatch{display:inline-block;width:18px;height:4px;margin-right:12px;}"
        ".graph-col{-webkit-box-flex:1;-webkit-flex:1;flex:1;background:#222;padding:15px;border-radius:8px;display:-webkit-box;-webkit-box-orient:horizontal;-webkit-box-align:center;-webkit-box-pack:center;display:-webkit-flex;display:flex;-webkit-align-items:center;align-items:center;-webkit-justify-content:center;justify-content:center;-webkit-box-sizing:border-box;box-sizing:border-box;overflow:hidden;}"
        "canvas{width:100%;max-width:592px;height:auto;display:block;margin:0 auto;}"
        ".clk-row{background:#222;border-radius:8px;padding:12px 25px;margin-top:15px;display:-webkit-box;-webkit-box-orient:horizontal;-webkit-box-align:center;-webkit-box-pack:center;display:-webkit-flex;display:flex;-webkit-align-items:center;align-items:center;-webkit-justify-content:center;justify-content:center;-webkit-box-sizing:border-box;box-sizing:border-box;}"
        ".clk-text{display:-webkit-box;-webkit-box-orient:vertical;-webkit-box-align:start;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;-webkit-align-items:flex-start;align-items:flex-start;margin-left:18px;}"
        "#d-clk{font-size:64px;font-weight:bold;color:#09F;font-family:monospace;line-height:1;letter-spacing:-2px;}"
        "#d-day{font-size:20px;font-weight:600;color:#eee;text-transform:uppercase;letter-spacing:1px;}"
        "#d-date{font-size:14px;color:#888;}"
        ".close-btn{position:absolute;top:10px;right:15px;font-size:16px;color:#666;cursor:pointer;z-index:10;}"
    );
    rtos_delay_milliseconds(1);

    // --- CSS part 3: battery gauge + tempblock ---
    poststr(request,
        ".batt{background:#11161e;border:1px solid #222d3b;border-radius:8px;padding:10px 10px 12px;}"
        ".batt-head{text-align:center;margin-bottom:6px;}"
        ".batt-head h1{font-size:13px;font-weight:700;color:#e7eef6;font-family:'Courier New',monospace;letter-spacing:.5px;margin:0;}"
        ".gauge{position:relative;width:188px;height:188px;margin:0 auto 8px;}"
        ".gauge svg{width:188px;height:188px;-webkit-transform:rotate(-90deg);transform:rotate(-90deg);}"
        ".gauge .track{fill:none;stroke:#171f2a;stroke-width:11;}"
        ".gauge .fill{fill:none;stroke:#37d67a;stroke-width:11;stroke-linecap:round;}"
        ".gc{position:absolute;top:0;left:0;right:0;bottom:0;display:-webkit-box;-webkit-box-orient:vertical;-webkit-box-align:center;-webkit-box-pack:center;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;-webkit-align-items:center;align-items:center;-webkit-justify-content:center;justify-content:center;text-align:center;padding:0 12%;}"
        ".gc .power{font-family:'Courier New',monospace;font-size:10px;color:#7c8a9a;}"
        ".gc .bigsoc{font-family:'Courier New',monospace;font-weight:700;font-size:36px;line-height:1;color:#37d67a;}"
        ".gc .bigsoc .pct{font-size:17px;font-weight:600;}"
        ".gc .state-lbl{font-size:10px;font-weight:600;color:#37d67a;margin-top:1px;}"
        ".gc .av{display:-webkit-box;-webkit-box-pack:center;-webkit-box-align:baseline;display:-webkit-flex;display:flex;-webkit-justify-content:center;justify-content:center;-webkit-align-items:baseline;align-items:baseline;font-family:'Courier New',monospace;font-size:11px;font-weight:600;margin-top:3px;}"
        ".gc .av span{margin:0 4px;}"
        ".gc .cap{font-family:'Courier New',monospace;font-size:10px;font-weight:600;margin-top:1px;}"
        ".gc .cells{font-family:'Courier New',monospace;font-size:9px;color:#7c8a9a;margin-top:4px;}"
        ".gc .delta{font-family:'Courier New',monospace;font-size:9px;color:#7c8a9a;margin-top:1px;}"
        ".gc .rule{width:80%;height:1px;background:#222d3b;margin:5px 0;}"
        ".gc .u{color:#7c8a9a;font-weight:500;}"
        ".tempblock{margin-top:8px;}"
        ".trow{display:-webkit-box;-webkit-box-align:center;display:-webkit-flex;display:flex;-webkit-align-items:center;align-items:center;font-family:'Courier New',monospace;font-size:10px;padding:3px 1px;}"
        ".trow .ic{width:14px;height:14px;margin-right:6px;}"
        ".trow .lbl{color:#7c8a9a;min-width:54px;}"
        ".trow .val{color:#e7eef6;font-weight:500;}"
        ".trow.bon .val{color:#37d67a;}"
        ".tbrule{height:1px;background:#222d3b;margin:3px 0;}"
        ".cdrow{display:-webkit-box;-webkit-box-pack:center;-webkit-box-align:center;display:-webkit-flex;display:flex;-webkit-justify-content:center;justify-content:center;-webkit-align-items:center;align-items:center;padding:4px 1px 2px;}"
        ".cd-item{display:-webkit-box;-webkit-box-align:center;display:-webkit-flex;display:flex;-webkit-align-items:center;align-items:center;margin:0 12px;}"
        ".cd-item .ic{width:15px;height:15px;opacity:.28;margin-right:5px;}"
        ".cd-item.on .ic{opacity:1;}"
        ".cd-item .dot2{width:8px;height:8px;border-radius:50%;background:#2a3340;}"
        ".cd-item.on .dot2{background:#4aa3ff;}"
        ".ic-bolt{fill:#f2b84b;}.ic-batt{stroke:#37d67a;}.ic-batt .lvl{fill:#37d67a;stroke:none;}"
        ".ic-dis{stroke:#4aa3ff;}.ic-dis .lvl{fill:#4aa3ff;stroke:none;}"
        ".ic-thermo{stroke:#ef6a6a;}.ic-scale{stroke:#9aa7b5;}"
    );
    rtos_delay_milliseconds(1);

    // --- CSS part 4: button box + sliders ---
    poststr(request,
        ".btn-box{-webkit-box-flex:1;-webkit-flex:1;flex:1;background:#222;border-radius:8px;padding:14px;margin-top:15px;display:-webkit-box;-webkit-box-orient:vertical;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;-webkit-box-sizing:border-box;box-sizing:border-box;}"
        ".btn-row{display:-webkit-box;-webkit-box-orient:horizontal;-webkit-box-align:stretch;display:-webkit-flex;display:flex;-webkit-align-items:stretch;align-items:stretch;}"
        ".btn{border:none;color:#fff;border-radius:6px;font-weight:bold;cursor:pointer;display:-webkit-box;-webkit-box-pack:center;-webkit-box-align:center;display:-webkit-flex;display:flex;-webkit-justify-content:center;justify-content:center;-webkit-align-items:center;align-items:center;-webkit-box-sizing:border-box;box-sizing:border-box;}"
        ".btn-auto{-webkit-box-flex:1;-webkit-flex:1;flex:1;margin-right:8px;font-size:16px;background:#09F;}"
        ".btn-right{-webkit-box-flex:1;-webkit-flex:1;flex:1;display:-webkit-box;-webkit-box-orient:vertical;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;}"
        ".btn-half{height:44px;font-size:13px;background:#555;}"
        ".btn-half.top{margin-bottom:8px;}"
        ".btn-div{height:42px;margin-top:8px;font-size:14px;background:#555;}"
        ".tmr{display:none;width:13px;height:13px;margin-right:6px;}"
        ".tmr.show{display:inline-block;}"
        ".sld-block{margin-top:12px;}"
        ".sld-block label{display:block;font-size:11px;color:#888;margin-bottom:5px;text-transform:uppercase;letter-spacing:.5px;}"
        ".sld-block input{width:100%;-webkit-box-sizing:border-box;box-sizing:border-box;}"
        "</style></head><body>"
    );
    rtos_delay_milliseconds(1);

    // --- Layout: top stats ---
    poststr(request,
        "<div id='dash-container'>"
        "<div class='close-btn' onclick='window.location.href=\"/index\"'>&#x2715;</div>"
        "<div class='top-stats'>"
        "<div><label>Voltage &amp; Current</label><b id='d-va'>--</b></div>"
        "<div><label>Power</label><b id='d-pwr'>--</b></div>"
        "<div><label>Now / 15min Est.</label><b><span id='d-bal'>--</span> / <span id='d-est'>--</span></b></div>"
        "<div id='d-chg-box'><label id='c-lbl'>ESS Status:</label><b id='c-v'>--</b><span id='c-chg' style='display:block;font-size:14px;color:#4caf50;margin-top:4px;'></span></div>"
        "</div>"
        "<div class='dash-row'>"
    );
    rtos_delay_milliseconds(1);

    // --- Layout: left column (reserved + import/export table + legend) ---
    poststr(request,
        "<div class='left-col'>"
        "<div class='sep-lbl'>Sensor Data</div>"
        "<div class='reserved'></div>"
        "<div class='grp-lbl'>Consumption Details</div>"
        "<table class='etbl'>"
        "<tr><th class='nm'></th><th>Import <span class='unit'>kWh</span></th><th>Export <span class='unit'>kWh</span></th></tr>"
        "<tr><td class='nm'>Last Hour</td><td class='num imp' id='d-clh-i'>--</td><td class='num exp' id='d-clh-e'>--</td></tr>"
        "<tr><td class='nm'>Today</td><td class='num imp' id='d-ctoday-i'>--</td><td class='num exp' id='d-ctoday-e'>--</td></tr>"
        "<tr><td class='nm'>Yesterday</td><td class='num imp' id='d-cyest-i'>--</td><td class='num exp' id='d-cyest-e'>--</td></tr>"
        "<tr><td class='nm'>2 Days Ago</td><td class='num imp' id='d-c2d-i'>--</td><td class='num exp' id='d-c2d-e'>--</td></tr>"
        "<tr><td class='nm'>3 Days Ago</td><td class='num imp' id='d-c3d-i'>--</td><td class='num exp' id='d-c3d-e'>--</td></tr>"
        "<tr class='tot'><td class='nm'>Total</td><td class='num imp' id='d-tot-i'>--</td><td class='num exp' id='d-tot-e'>--</td></tr>"
        "</table>"
        "<div class='leg-box'>"
        "<div class='sep-lbl' style='margin-bottom:12px;'>Graph Legend</div>"
        "<div class='leg-row'><span class='leg-swatch' style='background:#aaa;'></span><b>Total Energy</b></div>"
        "<div class='leg-row'><span class='leg-swatch' style='background:#4caf50;'></span><b>Charger Avg</b></div>"
        "<div class='leg-row'><span class='leg-swatch' style='background:#ff9800;'></span><b>Inverter Avg</b></div>"
        "</div>"
        "</div>"
    );
    rtos_delay_milliseconds(1);

    // --- Layout: center column (graph + clock) ---
    poststr(request,
        "<div class='center-col'>"
        "<div class='graph-col'>"
        "<div style='width:100%;max-width:592px;margin:0 auto;'>"
        "<div style='position:relative;width:100%;padding-bottom:57.43%;'>"
        "<canvas id='dynCanvas' style='position:absolute;top:0;left:0;width:100%;height:100%;'></canvas>"
        "</div></div></div>"
        "<div class='clk-row'>"
        "<div id='d-clk'>--:--</div>"
        "<div class='clk-text'><div id='d-day'>--</div><div id='d-date'>--</div></div>"
        "</div>"
        "</div>"
    );
    rtos_delay_milliseconds(1);

    // --- Layout: right column (battery gauge) ---
    poststr(request,
        "<div class='right-col'>"
        "<div class='batt'>"
        "<div class='batt-head'><h1 id='bt-title'>--</h1></div>"
        "<div class='gauge'>"
        "<svg viewBox='0 0 200 200' xmlns='http://www.w3.org/2000/svg'>"
        "<circle class='track' cx='100' cy='100' r='90'></circle>"
        "<circle class='fill' id='gaugeFill' cx='100' cy='100' r='90'></circle>"
        "</svg>"
        "<div class='gc'>"
        "<div class='power'><span id='bt-pwr'>--</span> <span class='u'>W</span></div>"
        "<div class='bigsoc'><span id='bt-soc'>--</span><span class='pct'>%</span></div>"
        "<div class='rule'></div>"
        "<div class='state-lbl' id='bt-state'>--</div>"
        "<div class='av'><span><span id='bt-a'>--</span> <span class='u'>A</span></span><span><span id='bt-v'>--</span> <span class='u'>V</span></span></div>"
        "<div class='rule'></div>"
        "<div class='cap'><b id='bt-cap'>--</b> <span class='u'>/ <span id='bt-capf'>--</span> Ah</span></div>"
        "<div class='cells'>Cells <span id='bt-cells'>-- - --</span> V</div>"
        "<div class='delta'>&#916; <span id='bt-delta'>--</span> mV</div>"
        "</div>"
        "</div>"
    );
    rtos_delay_milliseconds(1);

    // --- Layout: battery tempblock ---
    poststr(request,
        "<div class='tempblock'>"
        "<div class='trow'>"
        "<svg class='ic ic-batt' viewBox='0 0 24 24' fill='none' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><rect x='2' y='7.5' width='16' height='9' rx='2'/><path d='M21 11v2'/><rect class='lvl' x='4' y='9.5' width='9' height='5' rx='1'/></svg>"
        "<span class='lbl'>Bat:</span><span class='val'><span id='bt-tbat'>--</span> <span class='u'>&#176;C</span></span>"
        "</div>"
        "<div class='trow'>"
        "<svg class='ic ic-thermo' viewBox='0 0 24 24' fill='none' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><path d='M14 14.76V5a2 2 0 1 0-4 0v9.76a4 4 0 1 0 4 0z'/></svg>"
        "<span class='lbl'>MOSFET:</span><span class='val'><span id='bt-tmos'>--</span> <span class='u'>&#176;C</span></span>"
        "</div>"
        "<div class='tbrule'></div>"
        "<div class='trow' id='bt-balrow'>"
        "<svg class='ic ic-scale' viewBox='0 0 24 24' fill='none' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><path d='m16 16 3-8 3 8c-.87.65-1.92 1-3 1s-2.13-.35-3-1Z'/><path d='m2 16 3-8 3 8c-.87.65-1.92 1-3 1s-2.13-.35-3-1Z'/><path d='M7 21h10M12 3v18M3 7h2c2 0 5-1 7-2 2 1 5 2 7 2h2'/></svg>"
        "<span class='lbl'>Balancer:</span><span class='val'><span id='bt-bal'>--</span> <span class='u'>A</span></span>"
        "</div>"
        "<div class='tbrule'></div>"
        "<div class='cdrow'>"
        "<div class='cd-item' id='bt-chg'><svg class='ic ic-bolt' viewBox='0 0 24 24'><path d='M13 2 3 14h7l-1 8 10-12h-7z'/></svg><span class='dot2'></span></div>"
        "<div class='cd-item' id='bt-dis'><svg class='ic ic-dis' viewBox='0 0 24 24' fill='none' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'><rect x='2' y='7.5' width='16' height='9' rx='2'/><path d='M21 11v2'/><rect class='lvl' x='4' y='9.5' width='9' height='5' rx='1'/></svg><span class='dot2'></span></div>"
        "</div>"
        "</div>"
        "</div>"
    );
    rtos_delay_milliseconds(1);

    // --- Layout: button box (AUTO tall + INV/CHG halves, Diversion, sliders) ---
    poststr(request,
        "<div class='btn-box'>"
        "<div class='sep-lbl'>ESS System Modes</div>"
        "<div class='btn-row'>"
        "<button id='m-btn' class='btn btn-auto' onclick='tm()'>"
        "<span class='tmr' id='m-tmr'><svg viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2.5' stroke-linecap='round' stroke-linejoin='round' style='width:13px;height:13px;'><circle cx='12' cy='12' r='9'></circle><path d='M12 7v5l3 2'></path></svg></span>"
        "<span id='m-txt'>--</span></button>"
        "<div class='btn-right'>"
        "<button id='inv-btn' class='btn btn-half top' onclick='t_inv()'>INVERTER</button>"
        "<button id='chg-btn' class='btn btn-half' onclick='t_chg()'>CHARGER</button>"
        "</div>"
        "</div>"
        "<button id='div-btn' class='btn btn-div' onclick='td()'>"
        "<span class='tmr' id='div-tmr'><svg viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2.5' stroke-linecap='round' stroke-linejoin='round' style='width:13px;height:13px;'><circle cx='12' cy='12' r='9'></circle><path d='M12 7v5l3 2'></path></svg></span>"
        "<span id='div-txt'>DIVERSION</span></button>"
        "<div class='sld-block'><label>Max Pwr (<span id='lbl-pwr'></span>%)</label>"
        "<input type='range' id='sld-pwr' min='18' max='100' value='100' onchange='s_pwr(this.value)'></div>"
        "<div class='sld-block'><label>Export (<span id='lbl-exp'></span> Wh)</label>"
        "<input type='range' id='sld-exp' min='10' max='100' value='20' onchange='s_exp(this.value)'></div>"
        "<div class='sld-block'><label>Diversion (<span id='lbl-div'></span> Wh)</label>"
        "<input type='range' id='sld-div' min='30' max='255' value='60' onchange='s_div(this.value)'></div>"
        "</div>"
        "</div>"   // right-col
        "</div>"   // dash-row
        "</div>"   // dash-container
    );
    rtos_delay_milliseconds(1);

    // --- JS part 1: vars, helpers, decoders ---
    poststr(request,
        "<script>"
        "var busy=false,busySince=0,lastEv=-1,lastEnergyT=0,lastGraphT=0,graphIdx=0,lastClk='';"
        "var state_import=[],state_export=[],state_chg=[],state_inv=[];"
        "var GTYPES=['net','chginv'];"
        "var mode=0,dmp=0,tpa=100,tpm=100,texp=20,dthr=60,divUser=0,divOn=0,macSet=false;"
        "var DAYS=['SUNDAY','MONDAY','TUESDAY','WEDNESDAY','THURSDAY','FRIDAY','SATURDAY'];"
        "var MOS=['January','February','March','April','May','June','July','August','September','October','November','December'];"
        "var gf=document.getElementById('gaugeFill');"
        "var CIRC=2*Math.PI*90;"
        "if(gf){gf.style.strokeDasharray=CIRC;gf.style.strokeDashoffset=CIRC;}"
        "function setV(id,v){var e=document.getElementById(id);if(e){if(e.tagName==='INPUT')e.value=v;else e.innerHTML=v;}}"
        "function setS(id,v){var e=document.getElementById(id);if(e)e.style.color=v;}"
        "function setCls(id,on,base){var e=document.getElementById(id);if(e)e.className=base+(on?' on':'');}"
        "function show(id,on){var e=document.getElementById(id);if(e)e.className=on?'tmr show':'tmr';}"
        "function _b64toBytes(s){var bin=atob(s),len=bin.length,out=new Uint8Array(len),i;for(i=0;i<len;i++)out[i]=bin.charCodeAt(i);return out;}"
        "function _u16(b,o){return b[o]|(b[o+1]<<8);}"
        "function _u32(b,o){return (b[o]|(b[o+1]<<8)|(b[o+2]<<16)|(b[o+3]<<24))>>>0;}"
        "function _s16(b,o){var v=_u16(b,o);return v&0x8000?v-0x10000:v;}"
        "function _k2(v){return (v/100).toFixed(2);}"
        "function xhr(url,cb){var r=new XMLHttpRequest();var done=false;"
        "function finish(v){if(done)return;done=true;clearTimeout(tmr);try{r.abort();}catch(e){}cb(v);}"
        "var tmr=setTimeout(function(){finish(null);},8000);"
        "r.onreadystatechange=function(){if(r.readyState!==4)return;try{finish(r.status===200?JSON.parse(r.responseText):null);}catch(e){finish(null);}};"
        "r.onerror=function(){finish(null);};"
        "r.open('GET',url+'&t='+Date.now(),true);r.send();}"
    );
    rtos_delay_milliseconds(1);

    // --- JS part 2: control handlers ---
    poststr(request,
        "function refreshSliders(){"
        "var mp=(mode===0)?tpa:tpm;setV('lbl-pwr',mp);if(mp>=18)setV('sld-pwr',mp);"
        "setV('lbl-exp',texp);setV('sld-exp',texp);"
        "var dmin=texp+10,de=document.getElementById('sld-div');if(de)de.min=dmin;if(dthr<dmin)dthr=dmin;"
        "setV('lbl-div',dthr);setV('sld-div',dthr);}"
        "function btnColor(){"
        "var auto=(mode===0);"
        "setV('m-txt',auto?'AUTO':'MANUAL');"
        "var mb=document.getElementById('m-btn');"
        "if(mb)mb.style.background=auto?'#09F':(mode===1?'#8e44ad':'#f44336');"
        "show('m-tmr',mode===1);"
        "var i=document.getElementById('inv-btn'),c=document.getElementById('chg-btn');"
        "if(i)i.style.background=(dmp===5)?'#ff9800':'#555';"
        "if(c)c.style.background=(dmp>18)?'#4caf50':((dmp>=10&&dmp<=18)?'#8bc34a':'#555');"
        "var db=document.getElementById('div-btn');"
        "if(db)db.style.background=(divUser===1)?'#8e44ad':((divUser===2)?'#f44336':(divOn?'#ff9800':'#555'));"
        "show('div-tmr',divUser===1);}"
        "function tm(){mode=(mode+1)%3;xhr('/cm?cmnd=SetChargerMode%20'+mode,function(){});btnColor();refreshSliders();}"
        "function td(){divUser=(divUser+1)%3;xhr('/cm?cmnd=SetDivertUser%20'+divUser,function(){});btnColor();}"
        "function setDump(v){dmp=v;xhr('/cm?cmnd=SetDumpLoad%20'+v,function(){});btnColor();}"
        "function t_inv(){if(mode===0)return;setDump(dmp===5?0:5);}"
        "function t_chg(){if(mode===0)return;setDump(dmp>=18?0:18);}"
        "function s_pwr(v){xhr('/cm?cmnd=SetTargetPower%20'+v,function(){});setV('lbl-pwr',v);if(mode===0)tpa=parseInt(v,10);else tpm=parseInt(v,10);}"
        "function s_exp(v){xhr('/cm?cmnd=SetTargetExport%20'+v,function(){});texp=parseInt(v,10);setV('lbl-exp',v);"
        "var dmin=texp+10,de=document.getElementById('sld-div');if(de){de.min=dmin;if(parseInt(de.value,10)<dmin){de.value=dmin;dthr=dmin;setV('lbl-div',dmin);xhr('/cm?cmnd=SetDivertThreshold%20'+dmin,function(){});}}}"
        "function s_div(v){var dmin=texp+10;if(parseInt(v,10)<dmin)v=dmin;xhr('/cm?cmnd=SetDivertThreshold%20'+v,function(){});dthr=parseInt(v,10);setV('sld-div',v);setV('lbl-div',v);}"
    );
    rtos_delay_milliseconds(1);

    // --- JS part 3: applyCore ---
    poststr(request,
        "function applyCore(d){"
        "var b=_b64toBytes(d.c);"
        "var volt=_u16(b,0)/10,curr=_u16(b,2)/100,pwr=_s16(b,4),cpwr=_s16(b,6),bal=_s16(b,8),est=_s16(b,10);"
        "dmp=b[12];mode=b[13];tpa=b[14];texp=b[15];"
        "var clk_h=b[16],clk_m=b[17];var ev=_u16(b,20);var flags=b[22];tpm=b[23];divUser=b[24];dthr=b[25];"
        "var hntp=flags&1;divOn=(flags&2)?1:0;"
        "setV('d-va',volt.toFixed(0)+'V / '+curr.toFixed(2)+'A');"
        "var pe=document.getElementById('d-pwr');"
        "if(pe)pe.innerHTML='<span class=\"'+(pwr<0?'c-exp':'c-imp')+'\">'+pwr+' W</span> / <span class=\"'+(cpwr<0?'c-exp':'c-imp')+'\">'+cpwr+' W</span>';"
        "setV('d-bal',bal+' Wh');setS('d-bal',bal<0?'#4caf50':'#f44336');"
        "setV('d-est',est+' Wh');setS('d-est',est<0?'#4caf50':'#f44336');"
        "var cv,cc;if(dmp===0){cv='Idle';cc='#888';}else if(dmp===5){cv='Battery';cc='#4caf50';}else{cv=dmp+'%';cc='#0099FF';}"
        "setV('c-v',cv);setS('c-v',cc);"
        "setV('c-chg',(dmp>=18&&dmp<=100)?'Charging':'');"
        "var cs=('0'+clk_h).slice(-2)+':'+('0'+clk_m).slice(-2);"
        "if(cs!==lastClk){setV('d-clk',cs);lastClk=cs;}"
        "refreshSliders();btnColor();"
        "var dt=new Date();setV('d-day',DAYS[dt.getDay()]);setV('d-date',MOS[dt.getMonth()]+' '+dt.getDate()+', '+dt.getFullYear());"
        "return hntp?ev:null;}"
    );
    rtos_delay_milliseconds(1);

    // --- JS part 4: applyEnergy + applyBms ---
    poststr(request,
        "function applyEnergy(d){"
        "var b=_b64toBytes(d.e);"
        "setV('d-tot-i',_k2(_u32(b,0)));setV('d-tot-e',_k2(_u32(b,4)));"
        "setV('d-clh-i',_k2(_u16(b,8)));setV('d-ctoday-i',_k2(_u16(b,10)));setV('d-cyest-i',_k2(_u16(b,12)));setV('d-c2d-i',_k2(_u16(b,14)));setV('d-c3d-i',_k2(_u16(b,16)));"
        "setV('d-clh-e',_k2(_u16(b,18)));setV('d-ctoday-e',_k2(_u16(b,20)));setV('d-cyest-e',_k2(_u16(b,22)));setV('d-c2d-e',_k2(_u16(b,24)));setV('d-c3d-e',_k2(_u16(b,26)));"
        "lastEv=d.ev;lastEnergyT=Date.now();}"
        "function applyBms(d){"
        "if(!macSet&&d.mac){setV('bt-title',d.mac);macSet=true;}"
        "if(!d.b)return;var b=_b64toBytes(d.b);var fl=b[0];"
        "if(!(fl&8)){setV('bt-state','offline');setS('bt-state','#7c8a9a');if(gf)gf.style.stroke='#3a4452';return;}"
        "var soc=b[1],volt=_u16(b,2)/100,curr=_s16(b,4)/100,rem=_u16(b,6)/10,full=_u16(b,8)/10;"
        "var cmin=_u16(b,10)/1000,cmax=_u16(b,12)/1000,t1=_s16(b,14)/10,t2=_s16(b,16)/10,tmos=_s16(b,18)/10,bcur=_s16(b,20)/100;"
        "var st=curr>0.05?'Charging':(curr<-0.05?'Discharging':'Idle');"
        "var col=curr>0.05?'#37d67a':(curr<-0.05?'#e8a317':'#4aa3ff');"
        "var s2=Math.max(0,Math.min(100,soc));"
        "if(gf){gf.style.strokeDashoffset=CIRC*(1-s2/100);gf.style.stroke=col;}"
        "setV('bt-pwr',Math.round(volt*curr));setV('bt-soc',soc);setS('bt-soc',col);"
        "setV('bt-state',st);setS('bt-state',col);"
        "setV('bt-a',(curr>=0?'+':'')+curr.toFixed(2));setV('bt-v',volt.toFixed(2));"
        "setV('bt-cap',rem.toFixed(1));setV('bt-capf',full.toFixed(1));"
        "setV('bt-cells',cmin.toFixed(3)+' - '+cmax.toFixed(3));"
        "setV('bt-delta',Math.round((cmax-cmin)*1000));"
        "setV('bt-tbat',t1.toFixed(1)+' | '+t2.toFixed(1));setV('bt-tmos',tmos.toFixed(1));"
        "setV('bt-bal',bcur.toFixed(2));"
        "setCls('bt-chg',(fl&1),'cd-item');setCls('bt-dis',(fl&2)?1:0,'cd-item');"
        "setCls('bt-balrow',(fl&4)?1:0,'trow');}"
    );
    rtos_delay_milliseconds(1);

    // --- JS part 5: graph decode + poll cycle ---
    poststr(request,
        "function _decodeNet(d){if(!d.net)return;var b=_b64toBytes(d.net),n=b.length,i;state_import=new Array(n);state_export=new Array(n);for(i=0;i<n;i++){var v=b[i]*2-150;state_import[i]=v>0?v:0;state_export[i]=v<0?v:0;}}"
        "function _decodeChgInv(s){var b=_b64toBytes(s),n=b.length,i;var chg=new Array(n),inv=new Array(n);for(i=0;i<n;i++){var v=b[i];if(v&0x80)v-=0x100;chg[i]=v>0?v:0;inv[i]=v<0?-v:0;}return {chg:chg,inv:inv};}"
        "function applyGraph(d){_decodeNet(d);if(d.chginv){var ci=_decodeChgInv(d.chginv);state_chg=ci.chg;state_inv=ci.inv;}renderGraph();}"
        "function runCycle(){"
        "if(busy&&(Date.now()-busySince)<15000)return;busy=true;busySince=Date.now();var now=Date.now();"
        "xhr('/api_dash?req=core',function(d){"
        "var needEnergy=false;if(d&&d.c){var ev=applyCore(d);needEnergy=(ev!==null)&&(ev!==lastEv||now-lastEnergyT>=60000);}"
        "xhr('/api_dash?req=bms',function(bd){if(bd)applyBms(bd);"
        "function doGraph(){if(now-lastGraphT<20000){busy=false;return;}xhr('/api_dash?req='+GTYPES[graphIdx%2],function(gd){if(gd){applyGraph(gd);lastGraphT=Date.now();graphIdx++;}busy=false;});}"
        "if(needEnergy){xhr('/api_dash?req=energy',function(ed){if(ed)applyEnergy(ed);doGraph();});}else{doGraph();}"
        "});});}"
        "function loadAll(){busy=true;busySince=Date.now();"
        "xhr('/api_dash?req=core',function(d){if(d&&d.c)applyCore(d);"
        "xhr('/api_dash?req=energy',function(ed){if(ed)applyEnergy(ed);"
        "xhr('/api_dash?req=bms',function(bd){if(bd)applyBms(bd);"
        "xhr('/api_dash?req=net',function(g){if(g)applyGraph(g);"
        "xhr('/api_dash?req=chginv',function(g2){if(g2)applyGraph(g2);lastGraphT=Date.now();graphIdx=0;busy=false;});});});});});}"
        "</script>"
    );
    rtos_delay_milliseconds(1);

    // --- JS part 6: graph drawing + init/start (unchanged renderer) ---
    poststr(request,
        "<script>"
        "function _buildPath(ctx,p){ctx.beginPath();ctx.moveTo(p[0].x,p[0].y);"
        "for(var i=0;i<47;i++){var xc=(p[i].x+p[i+1].x)/2,yc=(p[i].y+p[i+1].y)/2;ctx.quadraticCurveTo(p[i].x,p[i].y,xc,yc);}"
        "ctx.lineTo(p[47].x,p[47].y);}"
        "function drawSmooth(ctx,arr,baseY,clamp,fill,col,lw){if(!arr||arr.length===0)return;var p=[],h,i;"
        "for(i=0;i<48;i++){h=(arr[i]||0)/2;if(clamp){if(h>150)h=150;if(h<-75)h=-75;}p.push({x:i*11+60,y:baseY-h});}"
        "if(fill){_buildPath(ctx,p);ctx.lineTo(577,baseY);ctx.lineTo(60,baseY);ctx.fillStyle=fill;ctx.fill();}"
        "_buildPath(ctx,p);ctx.strokeStyle=col;ctx.lineWidth=lw;ctx.stroke();"
        "ctx.beginPath();ctx.arc(p[47].x,p[47].y,lw*1.5,0,2*Math.PI);ctx.fillStyle=col;ctx.fill();}"
        "var gridCanvas=null;"
        "function initGrid(){var gc=document.createElement('canvas');var r=window.devicePixelRatio||1;"
        "gc.width=Math.round(592*r);gc.height=Math.round(340*r);var ctx=gc.getContext('2d');ctx.scale(r,r);"
        "ctx.fillStyle='#181818';ctx.fillRect(60,10,517,50);ctx.fillRect(60,75,517,235);"
        "ctx.lineWidth=1;ctx.strokeStyle='#333';ctx.beginPath();"
        "ctx.moveTo(60,35);ctx.lineTo(577,35);ctx.moveTo(60,75);ctx.lineTo(577,75);ctx.moveTo(60,150);ctx.lineTo(577,150);ctx.moveTo(60,300);ctx.lineTo(577,300);ctx.stroke();"
        "ctx.strokeStyle='#444';ctx.beginPath();ctx.moveTo(60,60);ctx.lineTo(577,60);ctx.moveTo(60,225);ctx.lineTo(577,225);ctx.stroke();"
        "ctx.strokeStyle='#666';ctx.beginPath();ctx.moveTo(60,10);ctx.lineTo(60,60);ctx.moveTo(60,75);ctx.lineTo(60,310);ctx.moveTo(577,10);ctx.lineTo(577,60);ctx.moveTo(577,75);ctx.lineTo(577,310);ctx.stroke();"
        "ctx.fillStyle='#aaa';ctx.font='14px sans-serif';ctx.textAlign='right';ctx.textBaseline='middle';"
        "ctx.fillText('100',48,10);ctx.fillText('0',48,60);ctx.fillText('+300',48,75);ctx.fillText('+150',48,150);"
        "ctx.fillStyle='#ccc';ctx.fillText('0 Wh',48,225);ctx.fillStyle='#aaa';ctx.fillText('-150',48,300);"
        "ctx.lineWidth=1;ctx.beginPath();ctx.font='12px sans-serif';ctx.textBaseline='top';"
        "for(var i=0;i<=47;i++){var x=(47-i)*11+60;ctx.moveTo(x,310);if(i%4===0){ctx.lineTo(x,316);ctx.textAlign=(i===0)?'right':'center';ctx.fillText((i===0)?'Now':'-'+(i/4)+'h',x,320);}else{ctx.lineTo(x,313);}}"
        "ctx.stroke();gridCanvas=gc;}"
        "function renderGraph(){var c=document.getElementById('dynCanvas');if(!c||!c.getContext)return;var ctx=c.getContext('2d');var r=window.devicePixelRatio||1;"
        "c.width=Math.round(592*r);c.height=Math.round(340*r);ctx.scale(r,r);ctx.clearRect(0,0,592,340);if(gridCanvas)ctx.drawImage(gridCanvas,0,0,592,340);"
        "if(state_import.length>0){var gi=ctx.createLinearGradient(0,75,0,225);gi.addColorStop(0,'rgba(244,67,54,.5)');gi.addColorStop(1,'rgba(244,67,54,.05)');drawSmooth(ctx,state_import,225,true,gi,'#ef5350',2.5);}"
        "if(state_export.length>0){var ge=ctx.createLinearGradient(0,225,0,310);ge.addColorStop(0,'rgba(55,214,122,.05)');ge.addColorStop(1,'rgba(55,214,122,.5)');drawSmooth(ctx,state_export,225,true,ge,'#37d67a',2.5);}"
        "if(state_chg.length>0)drawSmooth(ctx,state_chg,60,false,null,'#4caf50',2.5);"
        "if(state_inv.length>0)drawSmooth(ctx,state_inv,60,false,null,'#ff9800',2.5);}"
        "initGrid();loadAll();setInterval(runCycle,10000);"
        "</script></body></html>"
    );
    rtos_delay_milliseconds(1);

    poststr(request, NULL);
    return 0;
}
