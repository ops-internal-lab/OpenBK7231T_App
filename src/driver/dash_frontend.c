#include "dash_frontend.h"
#include "../new_common.h"  /* defines rtos_delay_milliseconds per platform */
#if !PLATFORM_ESPIDF
#include "rtos_pub.h"       /* Beken/BK7231 only — ESP32 uses vTaskDelay via new_common.h */
#endif
#include "dash_gz.h"  // pre-gzipped /dash page (g_dashGz / DASH_GZ_LEN)
#include <string.h>   // strstr

// Serves the dashboard page (HTML/CSS/JS) at /dash.
// Data is fetched client-side from /api_dash, implemented in
// drv_bl_shared.c (see http_fn_api_dash).

// ====================================================================
// OPTIMIZED DASHBOARD FRONTEND (Sequential State Machine Javascript)
// Includes Apple Full-Screen Web App Settings
// iOS 5 Safari compatible (legacy -webkit-box flexbox fallback,
// vh fallback, box-sizing prefix) while still rendering correctly
// on modern browsers (Chrome/Firefox/modern Safari use the
// unprefixed flex/box-sizing/vh declarations, which win the cascade)
// ====================================================================
int http_fn_custom_dash(http_request_t *request) {
    // Serve the pre-gzipped dashboard when the client accepts gzip (every
    // browser, incl. iOS Safari, does). Cached for a day so each browser
    // fetches it once. Falls through to the uncompressed page if not accepted.
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

    // --- CHUNK 1a: Header & CSS (part 1) ---
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
        ".left-col{-webkit-box-flex:0;-webkit-flex:0 0 230px;flex:0 0 230px;width:230px;background:#222;padding:10px;border-radius:8px;overflow-y:auto;margin-right:15px;-webkit-box-sizing:border-box;box-sizing:border-box;}"
        ".right-side{-webkit-box-flex:1;-webkit-flex:1;flex:1;display:-webkit-box;-webkit-box-orient:vertical;-webkit-box-align:stretch;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;min-width:0;}"
        ".top-row{display:-webkit-box;-webkit-box-orient:horizontal;-webkit-box-align:stretch;display:-webkit-flex;display:flex;height:400px;-webkit-align-items:stretch;align-items:stretch;}"
        ".sens-tbl{width:100%;font-size:14px;border-collapse:collapse;}"
        ".sens-tbl td{padding:5px 0;border-bottom:1px solid #333;font-weight:normal;}"
        ".sens-tbl td:last-child{font-weight:bold;text-align:right;}"
        ".sens-grp-lbl{font-size:12px;color:#888;text-transform:uppercase;margin:14px 0 8px;}"
    );
    rtos_delay_milliseconds(1);

    // --- CHUNK 1b: Header & CSS (part 2) ---
    poststr(request,
        ".graph-col{-webkit-box-flex:1;-webkit-flex:1;flex:1;background:#222;padding:15px;border-radius:8px;display:-webkit-box;-webkit-box-orient:horizontal;-webkit-box-align:center;-webkit-box-pack:center;display:-webkit-flex;display:flex;-webkit-align-items:center;align-items:center;-webkit-justify-content:center;justify-content:center;-webkit-box-sizing:border-box;box-sizing:border-box;margin-right:15px;overflow:hidden;}"
        "canvas{width:100%;max-width:592px;height:auto;display:block;margin:0 auto;}"
        ".right-col{-webkit-box-flex:0;-webkit-flex:0 0 240px;flex:0 0 240px;width:240px;background:#222;padding:20px;border-radius:8px;display:-webkit-box;-webkit-box-orient:vertical;-webkit-box-align:stretch;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;-webkit-box-sizing:border-box;box-sizing:border-box;}"
        ".btn-tgl{width:100%;height:50px;border:none;color:#fff;border-radius:6px;font-weight:bold;cursor:pointer;font-size:16px;margin-bottom:12px;display:block;}"
        ".sld-v-block{margin-top:10px;width:100%;}"
        ".sld-v-block label{display:block;font-size:11px;color:#888;margin-bottom:6px;text-transform:uppercase;letter-spacing:.5px;}"
        ".bottom-clk-row{background:#222;border-radius:8px;padding:10px 25px;margin-top:15px;display:-webkit-box;-webkit-box-orient:horizontal;-webkit-box-align:center;-webkit-box-pack:center;display:-webkit-flex;display:flex;-webkit-align-items:center;align-items:center;-webkit-justify-content:center;justify-content:center;-webkit-box-sizing:border-box;box-sizing:border-box;width:100%;}"
        ".clk-text-wrap{display:-webkit-box;-webkit-box-orient:vertical;-webkit-box-align:start;display:-webkit-flex;display:flex;-webkit-flex-direction:column;flex-direction:column;-webkit-align-items:flex-start;align-items:flex-start;margin-left:20px;text-align:left;}"
        "#d-clk{font-size:120px;font-weight:bold;color:#09F;font-family:monospace;line-height:1;letter-spacing:-3px;}"
        "#d-day{font-size:26px;font-weight:600;color:#eee;text-transform:uppercase;font-family:sans-serif;letter-spacing:2px;margin-bottom:2px;}"
    );
    rtos_delay_milliseconds(1);

    // --- CHUNK 1c: Header & CSS (part 3) ---
    poststr(request,
        "#d-date{font-size:16px;color:#888;font-family:sans-serif;}"
        ".close-btn{position:absolute;top:10px;right:15px;font-size:16px;color:#666;cursor:pointer;z-index:10;}"
        ".sep-lbl{font-size:12px;color:#888;margin-bottom:8px;text-transform:uppercase;}"
        ".leg-row{display:-webkit-box;-webkit-box-orient:horizontal;-webkit-box-align:center;display:-webkit-flex;display:flex;-webkit-align-items:center;align-items:center;margin-bottom:10px;}"
        ".leg-swatch{display:inline-block;width:18px;height:4px;margin-right:12px;}"
        ".param-lbl{font-size:12px;color:#888;text-transform:uppercase;margin-top:10px;margin-bottom:5px;}"
        "#c-chg{display:block;font-size:14px;font-weight:normal;color:#4caf50;margin-top:4px;}"
        "</style></head><body>"
    );
    rtos_delay_milliseconds(1);

    // --- CHUNK 2: Core Layout Structure ---
    poststr(request,
        "<div id='dash-container'>"
        "<div class='close-btn' onclick='window.location.href=\"/index\"'>&#x2715;</div>"
        "<div class='top-stats'>"
        "<div><label>Voltage &amp; Current</label><b id='d-va'>--</b></div>"
        "<div><label>Power</label><b id='d-pwr'>--</b></div>"
        "<div><label>Now / 15min Est.</label><b><span id='d-bal'>--</span> / <span id='d-est'>--</span></b></div>"
        "<div id='d-chg-box'><label id='c-lbl'>ESS Status:</label><b id='c-v'>--</b><span id='c-chg'></span></div>"
        "</div>"
    );
    rtos_delay_milliseconds(1);

    // --- CHUNK 3a: Left column (Sensor Data + Energy Totals + Consumption Details + Graph Legend)
        poststr(request,
            "<div class='dash-row'>"
            "<div class='left-col'>"
            "<div class='sep-lbl'>Sensor Data</div>"
            "<table class='sens-tbl'><tbody>"
            "<tr><td>Power Factor</td><td id='d-pf'>--</td></tr>"
            "<tr><td>Loop ms</td><td id='d-dbg'>--</td></tr>"
            "</tbody></table>"
            "<div class='sens-grp-lbl'>Energy Totals</div>"
            "<table class='sens-tbl'><tbody>"
            "<tr><td>Consumption</td><td id='d-econs'>--</td></tr>"
            "<tr><td>Generation</td><td id='d-egen'>--</td></tr>"
            "</tbody></table>"
            "<div class='sens-grp-lbl'>Consumption Details</div>"
            "<table class='sens-tbl'><tbody>"
            "<tr><td>Last Hour</td><td id='d-clh'>--</td></tr>"
            "<tr><td>Today</td><td id='d-ctoday'>--</td></tr>"
            "<tr><td>Yesterday</td><td id='d-cyest'>--</td></tr>"
            "<tr><td>2 Days Ago</td><td id='d-c2d'>--</td></tr>"
            "<tr><td>3 Days Ago</td><td id='d-c3d'>--</td></tr>"
            "</tbody></table>"
            "<div style='margin-top:20px;font-size:14px;color:#eee;padding:15px;background:#1a1a1a;border-radius:6px;border:1px solid #333;'>"
            "<div class='sep-lbl' style='margin-bottom:12px;'>Graph Legend</div>"
            "<div class='leg-row'><span class='leg-swatch' style='background:#aaa;'></span><b>Total Energy</b></div>"
            "<div class='leg-row'><span class='leg-swatch' style='background:#4caf50;'></span><b>Charger Avg</b></div>"
            "<div class='leg-row'><span class='leg-swatch' style='background:#ff9800;'></span><b>Inverter Avg</b></div>"
            "</div>"
            "</div>"
            "<div class='right-side'>"
            "<div class='top-row'>"
        );
        rtos_delay_milliseconds(1);

        // --- CHUNK 3b: Graph column, right column ("ESS System Modes")
        poststr(request,
            "<div class='graph-col'>"
            "<div style='width:100%;max-width:592px;margin:0 auto;'>"
            "<div style='position:relative;width:100%;padding-bottom:57.43%;'>"
            "<canvas id='dynCanvas' style='position:absolute;top:0;left:0;width:100%;height:100%;'></canvas>"
            "</div></div></div>"
            "<div class='right-col'>"
            "<div class='sep-lbl'>ESS System Modes</div>"
            "<button id='m-btn' class='btn-tgl' onclick='tm()'>--</button>"
            "<button id='inv-btn' class='btn-tgl' onclick='t_inv()'>INVERTER</button>"
            "<button id='chg-btn' class='btn-tgl' onclick='t_chg()'>CHARGER</button>"
            "<div class='param-lbl'>Parameters</div>"
            "<div class='sld-v-block'>"
            "<label>Max Pwr (<span id='lbl-pwr'></span>%)</label>"
            "<input type='range' id='sld-pwr' min='18' max='100' value='100' onchange='s_pwr(this.value)' style='width:100%;'>"
            "</div>"
            "<div class='sld-v-block' style='margin-top:15px;'>"
            "<label>Export (<span id='lbl-exp'></span> Wh)</label>"
            "<input type='range' id='sld-exp' min='10' max='100' value='20' onchange='s_exp(this.value)' style='width:100%;'>"
            "</div>"
            "</div>"
            "</div>"
            "<div class='bottom-clk-row'>"
            "<div id='d-clk'>--:--</div>"
            "<div class='clk-text-wrap'><div id='d-day'>--</div><div id='d-date'>--</div></div>"
            "</div>"
            "</div>"
            "</div>"
        );
        rtos_delay_milliseconds(1);

    // --- CHUNK 6a: State machine JS (vars, helpers, polling logic) ---
    poststr(request,
        "<script>"
        "var busy        = false;"
        "var busySince   = 0;"
        "var lastEv      = -1;"
        "var lastEnergyT = 0;"
        "var lastGraphT  = 0;"
        "var graphIdx    = 0;"
        "var lastClk     = '';"
        "var state_import = [];"
        "var state_export = [];"
        "var state_chg   = [];"
        "var state_inv   = [];"
        "var GTYPES      = ['net','chginv'];"
        "var dmp=0,auto=0;"
        "var DAYS=['SUNDAY','MONDAY','TUESDAY','WEDNESDAY','THURSDAY','FRIDAY','SATURDAY'];"
        "var MOS=['January','February','March','April','May','June','July','August','September','October','November','December'];"

        "function setV(id,v){var e=document.getElementById(id);if(e){if(e.tagName==='INPUT')e.value=v;else e.innerHTML=v;}}"
        "function setC(id,v){var e=document.getElementById(id);if(e)e.className=v;}"
        "function setS(id,v){var e=document.getElementById(id);if(e)e.style.color=v;}"

        "function xhr(url, cb) {"
        "  var r = new XMLHttpRequest();"
        "  var done = false;"
        "  function finish(v) {"
        "    if (done) return;"
        "    done = true;"
        "    clearTimeout(tmr);"
        "    try { r.abort(); } catch(e) {}"
        "    cb(v);"
        "  }"
        "  var tmr = setTimeout(function(){ finish(null); }, 8000);"
        "  r.onreadystatechange = function() {"
        "    if (r.readyState !== 4) return;"
        "    try { finish(r.status === 200 ? JSON.parse(r.responseText) : null); }"
        "    catch(e) { finish(null); }"
        "  };"
        "  r.onerror = function() { finish(null); };"
        "  r.open('GET', url + '&t=' + Date.now(), true);"
        "  r.send();"
        "}"

        "function s_pwr(v){xhr('/cm?cmnd=SetTargetPower%20'+v,function(){});setV('lbl-pwr',v);}"
        "function s_exp(v){xhr('/cm?cmnd=SetTargetExport%20'+v,function(){});setV('lbl-exp',v);}"
        "function upd(v){if(auto===1)return;if(v>=18){setV('sld-pwr',v);}s_pwr(v);dmp=parseInt(v,10);btnColor();}"
        "function t_inv(){upd(dmp===5?0:5);}"
        "function t_chg(){upd(dmp>=10?0:18);}"
        "function tm(){auto=(auto===1)?0:1;xhr('/cm?cmnd=ToggleAuto',function(){});btnColor();}"
        "function btnColor(){"
        "  var i=document.getElementById('inv-btn'),c=document.getElementById('chg-btn'),m=document.getElementById('m-btn');"
        "  if(i)i.style.background=(dmp===5)?'#ff9800':'#555';"
        "  if(c)c.style.background=(dmp>18)?'#4caf50':((dmp>=10&&dmp<=18)?'#8bc34a':'#555');"
        "  if(m){m.innerHTML=(auto===1)?'AUTO':'MANUAL';m.style.background=(auto===1)?'#09F':'#f44336';}"
        "}"

        "function applyCore(d) {"
        "  var b=_b64toBytes(d.c);"
        "  var volt  =(b[0]|(b[1]<<8))/10;"
        "  var curr  =(b[2]|(b[3]<<8))/100;"
        "  var pwr   =_s16(b,4);"
        "  var cpwr  =_s16(b,6);"
        "  var bal   =_s16(b,8);"
        "  var est   =_s16(b,10);"
        "  var dmp_v =b[12];"
        "  var auto_v=b[13];"
        "  var t_pwr =b[14];"
        "  var t_exp =b[15];"
        "  var clk_h =b[16];"
        "  var clk_m =b[17];"
        "  var lms   =b[18]|(b[19]<<8);"
        "  var ev_v  =b[20]|(b[21]<<8);"
        "  var hntp  =b[22]&1;"
        "  setV('d-va', volt.toFixed(0)+'V / '+curr.toFixed(2)+'A');"
        "  var pe=document.getElementById('d-pwr');"
        "  if(pe)pe.innerHTML='<span class=\"'+(pwr<0?'c-exp':'c-imp')+'\">'+pwr+' W</span> / <span class=\"'+(cpwr<0?'c-exp':'c-imp')+'\">'+cpwr+' W</span>';"
        "  setV('d-bal',bal+' Wh'); setC('d-bal',bal<0?'c-exp':'c-imp');"
        "  setV('d-est',est+' Wh'); setC('d-est',est<0?'c-exp':'c-imp');"
        "  var chg_v,chg_c;"
        "  if(dmp_v===0){chg_v='Idle';chg_c='#888';}"
        "  else if(dmp_v===5){chg_v='Battery';chg_c='#4caf50';}"
        "  else{chg_v=dmp_v+'%';chg_c='#0099FF';}"
        "  setV('c-v',chg_v); setS('c-v',chg_c);"
        "  setV('c-chg',(dmp_v>=18&&dmp_v<=100)?'Charging':'');"
        "  setV('d-dbg',lms);"
        "  var clk_str=('0'+clk_h).slice(-2)+':'+(' 0'+clk_m).slice(-2);"
        "  if(clk_str!==lastClk){setV('d-clk',clk_str);lastClk=clk_str;}"
        "  if(t_pwr>=18)setV('sld-pwr',t_pwr);"
        "  setV('lbl-pwr',t_pwr);setV('sld-exp',t_exp);setV('lbl-exp',t_exp);"
        "  dmp=dmp_v;auto=auto_v;btnColor();"
        "  var dt=new Date();"
        "  setV('d-day',DAYS[dt.getDay()]);"
        "  setV('d-date',MOS[dt.getMonth()]+' '+dt.getDate()+', '+dt.getFullYear());"
        "  return hntp?ev_v:null;"
        "}"

        "function _b64toBytes(s){"
        "var bin=atob(s),len=bin.length,out=new Uint8Array(len);"
        "for(var i=0;i<len;i++)out[i]=bin.charCodeAt(i);"
        "return out;"
        "}"

        "function _u16(b,o){return b[o]|(b[o+1]<<8);}"
        "function _u32(b,o){return (b[o]|(b[o+1]<<8)|(b[o+2]<<16)|(b[o+3]<<24))>>>0;}"
        "function _s16(b,o){var v=_u16(b,o);return v&0x8000?v-0x10000:v;}"
        "function _kwh(v){return (v/100).toFixed(2)+' kWh';}"

        "function applyEnergy(d) {"
        "  var b=_b64toBytes(d.e);"
        "  setV('d-pf',     (b[0]/100).toFixed(2));"
        "  setV('d-econs',  _kwh(_u32(b,1)));"
        "  setV('d-egen',   _kwh(_u32(b,5)));"
        "  setV('d-clh',    _kwh(_u16(b,9)));"
        "  setV('d-ctoday', _kwh(_u16(b,11)));"
        "  setV('d-cyest',  _kwh(_u16(b,13)));"
        "  setV('d-c2d',    _kwh(_u16(b,15)));"
        "  setV('d-c3d',    _kwh(_u16(b,17)));"
        "  lastEv      = d.ev;"
        "  lastEnergyT = Date.now();"
        "}"

        "function _decodeNet(d){"
        "if(!d.net)return;"
        "var b=_b64toBytes(d.net),n=b.length;"
        "state_import=new Array(n);state_export=new Array(n);"
        "for(var i=0;i<n;i++){"
        "var v=b[i]*2-150;"
        "state_import[i]=v>0?v:0;"
        "state_export[i]=v<0?v:0;"
        "}}"

        "function _decodeChgInv(s){"
        "var b=_b64toBytes(s),n=b.length;"
        "var chg=new Array(n),inv=new Array(n);"
        "for(var i=0;i<n;i++){"
        "var v=b[i];if(v&0x80)v-=0x100;"
        "chg[i]=v>0?v:0;"
        "inv[i]=v<0?-v:0;"
        "}"
        "return {chg:chg,inv:inv};"
        "}"

        "function applyGraph(d) {"
        "  _decodeNet(d);"
        "  if (d.chginv) { var ci=_decodeChgInv(d.chginv); state_chg=ci.chg; state_inv=ci.inv; }"
        "  renderGraph();"
        "}"

        "function runCycle() {"
        "  if (busy && (Date.now() - busySince) < 15000) return;"
        "  busy = true; busySince = Date.now();"
        "  var now = Date.now();"
        "  xhr('/api_dash?req=core', function(d) {"
        "    var needEnergy = false;"
        "    if (d&&d.c) {"
        "      var ev=applyCore(d);"
        "      needEnergy=(ev!==null)&&(ev!==lastEv||now-lastEnergyT>=60000);"
        "    }"
        "    function doGraph() {"
        "      if (now - lastGraphT < 20000) { busy = false; return; }"
        "      xhr('/api_dash?req=' + GTYPES[graphIdx % 2], function(gd) {"
        "        if (gd) { applyGraph(gd); lastGraphT = Date.now(); graphIdx++; }"
        "        busy = false;"
        "      });"
        "    }"
        "    if (needEnergy) {"
        "      xhr('/api_dash?req=energy', function(ed) {"
        "        if (ed) applyEnergy(ed);"
        "        doGraph();"
        "      });"
        "    } else {"
        "      doGraph();"
        "    }"
        "  });"
        "}"

        "function loadAll() {"
        "  busy = true; busySince = Date.now();"
        "  xhr('/api_dash?req=core', function(d) {"
        "    if (d&&d.c) applyCore(d);"
        "    xhr('/api_dash?req=energy', function(ed) {"
        "      if (ed) applyEnergy(ed);"
        "      xhr('/api_dash?req=net', function(g) {"
        "        if (g) applyGraph(g);"
        "        xhr('/api_dash?req=chginv', function(g) {"
        "          if (g) applyGraph(g);"
        "          lastGraphT = Date.now();"
        "          graphIdx   = 0;"
        "          busy       = false;"
        "        });"
        "      });"
        "    });"
        "  });"
        "}"
        "</script>"
    );
    rtos_delay_milliseconds(1);

    // --- CHUNK 6b: Graph drawing JS + init/start ---
    poststr(request,
        "<script>"
        "function _buildPath(ctx,p){"
        "ctx.beginPath();ctx.moveTo(p[0].x,p[0].y);"
        "for(var i=0;i<47;i++){"
        "var xc=(p[i].x+p[i+1].x)/2,yc=(p[i].y+p[i+1].y)/2;"
        "ctx.quadraticCurveTo(p[i].x,p[i].y,xc,yc);"
        "}"
        "ctx.lineTo(p[47].x,p[47].y);"
        "}"

        "function drawSmooth(ctx,arr,baseY,clamp,fill,col,lw){"
        "if(!arr||arr.length===0)return;"
        "var p=[],h,i;"
        "for(i=0;i<48;i++){"
        "h=(arr[i]||0)/2;"
        "if(clamp){if(h>150)h=150;if(h<-75)h=-75;}"
        "p.push({x:i*11+60,y:baseY-h});"
        "}"
        "if(fill){"
        "_buildPath(ctx,p);"
        "ctx.lineTo(577,baseY);ctx.lineTo(60,baseY);"
        "ctx.fillStyle=fill;ctx.fill();"
        "}"
        "_buildPath(ctx,p);"
        "ctx.strokeStyle=col;ctx.lineWidth=lw;ctx.stroke();"
        "ctx.beginPath();ctx.arc(p[47].x,p[47].y,lw*1.5,0,2*Math.PI);"
        "ctx.fillStyle=col;ctx.fill();"
        "}"

        "var gridCanvas=null;"
        "function initGrid(){"
        "var gc=document.createElement('canvas');"
        "var r=window.devicePixelRatio||1;"
        "gc.width=Math.round(592*r);gc.height=Math.round(340*r);"
        "var ctx=gc.getContext('2d');"
        "ctx.scale(r,r);"
        "ctx.fillStyle='#181818';"
        "ctx.fillRect(60,10,517,50);ctx.fillRect(60,75,517,235);"
        "ctx.lineWidth=1;ctx.strokeStyle='#333';ctx.beginPath();"
        "ctx.moveTo(60,35);ctx.lineTo(577,35);"
        "ctx.moveTo(60,75);ctx.lineTo(577,75);"
        "ctx.moveTo(60,150);ctx.lineTo(577,150);"
        "ctx.moveTo(60,300);ctx.lineTo(577,300);ctx.stroke();"
        "ctx.strokeStyle='#444';ctx.beginPath();"
        "ctx.moveTo(60,60);ctx.lineTo(577,60);"
        "ctx.moveTo(60,225);ctx.lineTo(577,225);ctx.stroke();"
        "ctx.strokeStyle='#666';ctx.beginPath();"
        "ctx.moveTo(60,10);ctx.lineTo(60,60);"
        "ctx.moveTo(60,75);ctx.lineTo(60,310);"
        "ctx.moveTo(577,10);ctx.lineTo(577,60);"
        "ctx.moveTo(577,75);ctx.lineTo(577,310);ctx.stroke();"
        "ctx.fillStyle='#aaa';ctx.font='14px sans-serif';ctx.textAlign='right';ctx.textBaseline='middle';"
        "ctx.fillText('100',48,10);ctx.fillText('0',48,60);"
        "ctx.fillText('+300',48,75);ctx.fillText('+150',48,150);"
        "ctx.fillStyle='#ccc';ctx.fillText('0 Wh',48,225);"
        "ctx.fillStyle='#aaa';ctx.fillText('-150',48,300);"
        "ctx.lineWidth=1;ctx.beginPath();ctx.font='12px sans-serif';ctx.textBaseline='top';"
        "for(var i=0;i<=47;i++){"
        "var x=(47-i)*11+60;ctx.moveTo(x,310);"
        "if(i%4===0){"
        "ctx.lineTo(x,316);ctx.textAlign=(i===0)?'right':'center';"
        "ctx.fillText((i===0)?'Now':'-'+(i/4)+'h',x,320);"
        "}else{ctx.lineTo(x,313);}"
        "}"
        "ctx.stroke();"
        "gridCanvas=gc;"
        "}"

        "function renderGraph(){"
        "var c=document.getElementById('dynCanvas');"
        "if(!c||!c.getContext)return;"
        "var ctx=c.getContext('2d');"
        "var r=window.devicePixelRatio||1;"
        "c.width=Math.round(592*r);c.height=Math.round(340*r);"
        "ctx.scale(r,r);"
        "ctx.clearRect(0,0,592,340);"
        "if(gridCanvas)ctx.drawImage(gridCanvas,0,0,592,340);"
        "if(state_import.length>0){"
        "var gradImp=ctx.createLinearGradient(0,75,0,225);"
        "gradImp.addColorStop(0,'rgba(244,67,54,.5)');"
        "gradImp.addColorStop(1,'rgba(244,67,54,.05)');"
        "drawSmooth(ctx,state_import,225,true,gradImp,'#ef5350',2.5);"
        "}"
        "if(state_export.length>0){"
        "var gradExp=ctx.createLinearGradient(0,225,0,310);"
        "gradExp.addColorStop(0,'rgba(55,214,122,.05)');"
        "gradExp.addColorStop(1,'rgba(55,214,122,.5)');"
        "drawSmooth(ctx,state_export,225,true,gradExp,'#37d67a',2.5);"
        "}"
        "if(state_chg.length>0)drawSmooth(ctx,state_chg,60,false,null,'#4caf50',2.5);"
        "if(state_inv.length>0)drawSmooth(ctx,state_inv,60,false,null,'#ff9800',2.5);"
        "}"

        "initGrid(); loadAll(); setInterval(runCycle, 10000);"
        "</script></body></html>"
    );
   rtos_delay_milliseconds(1);

poststr(request, NULL);
    return 0;
}
