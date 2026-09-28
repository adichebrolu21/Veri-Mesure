// The operator page.
//
// One shirt is a short list of steps, and the page shows exactly one of them at
// a time: the camera's picture, as large as the screen allows, one line saying
// what to do now, and one big button for what comes next. A verdict lands as a
// band of colour across the top of the picture, big enough to read from across
// the bench. Recalibrate sits in the bar along the top, where it is always in
// reach; everything else an engineer needs - checking the scale, the camera's
// exposure, the picture's brightness, targets, adding a style, the log - is
// under Settings, out of the operator's way.
//
// Every number is rendered as text here rather than burned into the video,
// which is both far more legible and far easier to restyle than putText.

#pragma once

namespace verimeasure {

inline const char* const STATION_PAGE = R"VLPAGE(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="color-scheme" content="dark">
<title>VeriMeasure station</title>
<style>
  /* One accent, the blue of the cloth the garments lie on, for everything the
     operator can act on and for the live outline. Green, amber and red mean
     pass, warn and fail and nothing else. Nothing is loaded from the network:
     Inter when the machine has it, Segoe UI when it does not. */
  :root {
    --bg:#0A0F14; --bg2:#0E151C;
    --glass:rgba(16,23,31,.74); --glass-strong:rgba(14,20,27,.9);
    --solid:#111921; --raised:#17212B; --raised2:#1E2A36;
    --line:rgba(148,163,184,.13); --line2:rgba(148,163,184,.26);
    --text:#E8EEF4; --muted:#95A3B3; --dim:#5F6C7B;
    --accent:#36C8FF; --accent-ink:#021019; --accent-soft:rgba(54,200,255,.13);
    --pass:#22C55E; --pass-ink:#02160A; --pass-soft:rgba(34,197,94,.14);
    --warn:#F5A524; --warn-ink:#1C1200; --warn-soft:rgba(245,165,36,.14);
    --fail:#F04646; --fail-ink:#1F0404; --fail-soft:rgba(240,70,70,.14);
    --r:14px; --r-sm:10px;
    --touch:56px;
    --fast:200ms; --ease:cubic-bezier(.2,.75,.25,1);
    --sans:"Inter var","Inter","Segoe UI Variable Text","Segoe UI",system-ui,-apple-system,"Helvetica Neue",Arial,sans-serif;
  }
  * { box-sizing:border-box; }
  html,body { height:100%; margin:0; overflow:hidden; }
  body {
    background:var(--bg); color:var(--text); font-family:var(--sans);
    font-size:15px; line-height:1.45; -webkit-font-smoothing:antialiased;
    display:flex; flex-direction:column; overflow:hidden;
    font-feature-settings:"cv11","ss01";
  }
  .num { font-variant-numeric:tabular-nums; font-feature-settings:"tnum"; }
  [hidden] { display:none !important; }

  /* ---- buttons: a finger, a glove, a mouse ---- */
  button {
    font:inherit; font-weight:600; color:var(--text); background:var(--raised);
    border:1px solid var(--line2); border-radius:var(--r-sm);
    min-height:48px; padding:10px 16px; cursor:pointer;
    touch-action:manipulation; -webkit-tap-highlight-color:transparent; user-select:none;
    transition:background-color var(--fast) var(--ease), border-color var(--fast) var(--ease), transform 90ms ease;
  }
  button:hover:not(:disabled) { border-color:var(--muted); background:var(--raised2); }
  button:active:not(:disabled) { transform:scale(.98); }
  button:disabled { opacity:.38; cursor:default; }
  button:focus-visible { outline:2px solid var(--accent); outline-offset:2px; }
  button.primary {
    width:100%; min-height:64px; font-size:19px; font-weight:700; letter-spacing:-.005em;
    background:var(--accent); color:var(--accent-ink); border-color:var(--accent);
  }
  button.primary:hover:not(:disabled) { background:#5CD4FF; border-color:#5CD4FF; }
  button.big { width:100%; min-height:var(--touch); font-size:17px; }
  button.warnBtn { background:var(--warn-soft); border-color:rgba(245,165,36,.55); color:#FFD58A; }
  button.warnBtn:hover:not(:disabled) { background:rgba(245,165,36,.22); border-color:var(--warn); }
  button.ghost { background:transparent; }
  button.small { min-height:48px; padding:8px 14px; font-size:14px; }
  .iconBtn { width:48px; height:48px; min-height:48px; padding:0; display:grid; place-items:center;
             border-radius:12px; background:transparent; border-color:transparent; }
  .iconBtn:hover:not(:disabled) { background:var(--raised); border-color:var(--line2); }
  .iconBtn svg { width:24px; height:24px; }

  select, input {
    font:inherit; color:var(--text); background:var(--raised);
    border:1px solid var(--line2); border-radius:var(--r-sm);
  }
  select { min-height:48px; padding:0 40px 0 14px; font-size:15px; font-weight:600;
           appearance:none; -webkit-appearance:none; max-width:100%;
           background-image:linear-gradient(45deg,transparent 50%,var(--muted) 50%),linear-gradient(135deg,var(--muted) 50%,transparent 50%);
           background-position:calc(100% - 20px) 50%, calc(100% - 14px) 50%; background-size:6px 6px; background-repeat:no-repeat; }
  input { min-height:48px; padding:0 12px; font-size:16px; width:100%; }
  select:focus-visible, input:focus-visible { outline:2px solid var(--accent); outline-offset:-1px; }

  /* ---- the bar along the top ---- */
  .top {
    flex:0 0 auto; display:flex; align-items:center; gap:14px;
    padding:8px 12px 8px 18px; min-height:64px;
    background:var(--bg2); border-bottom:1px solid var(--line);
  }
  .brand { display:flex; align-items:center; gap:10px; min-width:0; flex:0 1 auto; }
  .brand svg { width:26px; height:26px; flex:0 0 auto; }
  .brand b { font-size:16px; font-weight:700; letter-spacing:-.01em; white-space:nowrap; }
  .shirtNo { color:var(--muted); font-size:14px; white-space:nowrap; padding:4px 10px;
             border:1px solid var(--line2); border-radius:999px; }
  .stylePick { flex:0 1 300px; min-width:0; }
  .stylePick select { width:100%; }
  .top .spacer { flex:1 1 0; min-width:0; }
  /* The shift at a glance: a strip of three numbers, a way into the detail. */
  button.stats { display:flex; align-items:baseline; gap:18px; white-space:nowrap; min-width:0; overflow:hidden;
                 background:transparent; border-color:transparent; padding:6px 12px; font-weight:400; }
  button.stats:hover:not(:disabled) { background:var(--raised); border-color:var(--line2); }
  .stats div { display:flex; align-items:baseline; gap:6px; }
  .stats b { font-size:22px; font-weight:750; letter-spacing:-.02em; }
  .stats span { color:var(--muted); font-size:13px; }
  .lamp { display:flex; align-items:center; gap:8px; color:var(--muted); font-size:13px; white-space:nowrap; }
  .lamp span { max-width:110px; overflow:hidden; text-overflow:ellipsis; }
  .lamp i { width:10px; height:10px; border-radius:50%; background:var(--dim); flex:0 0 auto; }
  .lamp.live i { background:var(--accent); box-shadow:0 0 0 3px var(--accent-soft); }
  .lamp.held i { background:var(--text); }
  .lamp.busy i { background:var(--accent); }
  .lamp.fault i { background:var(--fail); box-shadow:0 0 0 3px var(--fail-soft); }
  .lamp.fault { color:#FFB4B4; }
  /* Wide enough for "Recalibrating..." too, so the bar does not jump when it
     starts. */
  .top .calBtn { flex:0 0 auto; min-width:10.5em; white-space:nowrap; }

  /* One row, exactly the window's height left: a tall panel scrolls inside
     itself rather than stretching the page. */
  main { flex:1 1 auto; min-height:0; display:grid; grid-template-columns:minmax(0,1fr) 400px;
         grid-template-rows:minmax(0,1fr); }

  /* ---- the picture ---- */
  .feed { position:relative; min-height:0; min-width:0; background:#05080B;
          display:flex; align-items:center; justify-content:center; overflow:hidden; }
  /* Sized by fitVideo to fill the feed, larger or smaller than the stream,
     keeping its shape: the element's box stays exactly the picture, which is
     what a tap is measured against. */
  .feed img { max-width:100%; max-height:100%; display:block; -webkit-user-drag:none; user-select:none; }
  .feed .waiting { color:var(--dim); font-size:15px; padding:20px; text-align:center; }
  /* Drawn over the picture by the page: the drag as it is drawn, the marks of
     a step revisited, the glow of an outline arriving. Never take a tap. */
  .feed canvas.ink { position:absolute; pointer-events:none; }
  .bar { position:absolute; left:0; right:0; bottom:0; height:4px; background:rgba(255,255,255,.06); }
  .bar i { display:block; height:100%; width:0; background:var(--accent); transition:width .12s linear; }
  .bar.held i { background:var(--text); }
  /* A step that takes taps or a box says so with an accent edge round the
     picture, and hands a finger dragged across it to the page: left to itself
     Chromium takes a sideways swipe as "go back" and navigates away. */
  .feed.pointable img, .feed.boxable img { cursor:crosshair; touch-action:none; }
  .feed.pointable::after, .feed.boxable::after {
    content:""; position:absolute; inset:0; pointer-events:none; z-index:1;
    box-shadow:inset 0 0 0 3px var(--accent);
  }

  /* ---- the verdict, across the top of the picture ----
     Solid colour, big type, the whole width: read from a metre away. It never
     takes a tap meant for the picture under it; only its button does. On a
     step drawn on - a box, a seam - it is a slim strip, out of the way. */
  .banner {
    position:absolute; z-index:4; top:12px; left:12px; right:12px;
    display:grid; grid-template-columns:auto minmax(0,1fr) auto; align-items:center; gap:18px;
    padding:14px 22px; border-radius:var(--r); pointer-events:none;
    box-shadow:0 10px 40px rgba(0,0,0,.35);
  }
  .banner button { pointer-events:auto; }
  .banner.pass { background:rgba(34,197,94,.95); color:var(--pass-ink); }
  .banner.fail { background:rgba(240,70,70,.95); color:#fff; }
  .banner.warn { background:rgba(245,165,36,.96); color:var(--warn-ink); }
  .banner.enter { animation:bannerIn var(--fast) var(--ease); }
  .bIcon { width:56px; height:56px; border-radius:50%; display:grid; place-items:center;
           font-size:32px; font-weight:800; line-height:1; background:rgba(0,0,0,.16); }
  .banner.fail .bIcon { background:rgba(0,0,0,.18); }
  /* Sized from the picture's box, so a portrait tablet whose feed is the
     whole width gets the big word too. */
  .bWord { font-size:clamp(34px,calc(var(--fw,1000px) * .064),64px); font-weight:800; letter-spacing:-.025em; line-height:1; }
  .bSub { margin-top:6px; font-size:clamp(14px,calc(var(--fw,1000px) * .019),18px); font-weight:600; opacity:.86;
          white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .bValue { text-align:right; }
  .bValue b { font-size:clamp(34px,calc(var(--fw,1000px) * .064),64px); font-weight:800; letter-spacing:-.025em; line-height:1; }
  .bValue small { font-size:clamp(16px,calc(var(--fw,1000px) * .024),22px); font-weight:700; margin-left:4px; opacity:.8; }
  .bValue div { margin-top:6px; font-size:clamp(14px,calc(var(--fw,1000px) * .019),18px); font-weight:700; opacity:.88; }
  /* A pass counts down to the next step on the band itself: the seconds
     left, big, what comes next, and the way to go now. */
  .bGo { grid-column:1 / -1; display:flex; align-items:center; gap:16px; margin-top:2px; }
  .bCount { flex:0 0 auto; width:68px; height:68px; border-radius:50%; display:grid; place-items:center;
            font-size:38px; font-weight:800; line-height:1; background:rgba(0,0,0,.24); color:#fff; }
  .bNextWrap { flex:1 1 auto; min-width:0; }
  .bNext { font-size:clamp(15px,calc(var(--fw,1000px) * .021),21px); font-weight:700;
           white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .bMeter { height:8px; margin-top:8px; border-radius:4px; background:rgba(0,0,0,.2); overflow:hidden; }
  .bMeter i { display:block; height:100%; width:0; background:rgba(0,0,0,.55); transition:width .25s linear; }
  .banner .bGo button { flex:0 0 auto; min-width:160px; min-height:56px; font-size:17px; font-weight:700;
                        background:var(--pass-ink); color:#fff; border-color:transparent; }
  .banner .bGo button:hover:not(:disabled) { background:#073a1a; border-color:transparent; }
  /* Folded down: one line, the word and the number. */
  .banner.mini { padding:8px 16px; gap:12px; top:8px; left:8px; right:8px; opacity:.94; }
  .banner.mini .bIcon { width:32px; height:32px; font-size:18px; }
  .banner.mini .bWord, .banner.mini .bValue b { font-size:24px; }
  .banner.mini .bSub, .banner.mini .bValue div { display:none; }

  /* The step just arrived at, said for a moment, over the picture. */
  .toast {
    position:absolute; z-index:4; top:14px; left:50%; transform:translateX(-50%);
    max-width:calc(100% - 28px); padding:12px 20px; border-radius:999px;
    background:var(--glass-strong); border:1px solid var(--line2);
    -webkit-backdrop-filter:blur(14px); backdrop-filter:blur(14px);
    font-size:17px; font-weight:700; white-space:nowrap; overflow:hidden; text-overflow:ellipsis;
    pointer-events:none; animation:toastIn var(--fast) var(--ease);
  }
  .toast span { color:var(--accent); margin-right:8px; }

  /* ---- a card over the picture: a finished shirt, recalibrating, the scale ---- */
  .over { position:absolute; inset:0; z-index:5; display:flex; align-items:center; justify-content:center;
          padding:16px; overflow:auto; background:rgba(5,8,11,.5); }
  .over.clearBg { background:transparent; pointer-events:none; align-items:flex-end; }
  .over.clearBg .card { pointer-events:auto; }
  .card {
    width:min(460px,100%); max-height:100%; overflow:auto; padding:20px 22px;
    background:var(--glass-strong); border:1px solid var(--line2); border-radius:18px;
    -webkit-backdrop-filter:blur(18px); backdrop-filter:blur(18px);
    box-shadow:0 20px 60px rgba(0,0,0,.5); animation:cardIn var(--fast) var(--ease);
  }
  .cardHead { display:flex; align-items:center; gap:12px; }
  .spin { width:22px; height:22px; flex:0 0 auto; border-radius:50%;
          border:2.5px solid var(--line2); border-top-color:var(--accent); animation:spin .8s linear infinite; }
  .glyph { width:26px; height:26px; flex:0 0 auto; border-radius:50%; display:grid; place-items:center;
           font-size:14px; font-weight:800; color:var(--bg); background:var(--muted); }
  .card.ok .glyph { background:var(--pass); }
  .card.fail .glyph { background:var(--fail); }
  .card.warn .glyph, .card.wait .glyph { background:var(--warn); }
  .cardTitle { flex:1; font-size:18px; font-weight:700; }
  .card.ok .cardTitle { color:var(--pass); }
  .card.fail .cardTitle { color:#FF8A8A; }
  .card.warn .cardTitle, .card.wait .cardTitle { color:var(--warn); }
  .cardTime { color:var(--dim); font-size:13px; }
  .cardStep { margin-top:12px; font-size:15px; font-weight:600; }
  .cardCount { margin-top:3px; color:var(--muted); font-size:13px; }
  .cardMeter { margin-top:14px; height:6px; border-radius:3px; background:var(--line2); overflow:hidden; }
  .cardMeter i { display:block; height:100%; width:0; background:var(--accent); transition:width .25s linear; }
  .cardMeter.busy i { width:35%; animation:sweep 1.1s ease-in-out infinite; }
  .cardNote { margin-top:12px; color:var(--muted); font-size:14px; }
  .cardNote.big { color:var(--warn); }
  .cardDetail { margin-top:8px; color:var(--dim); font-size:13px; word-break:break-word; }
  .cardDetail.ok { color:var(--pass); }
  .cardDetail.bad { color:#FF8A8A; }
  .cardActions { display:flex; justify-content:flex-end; gap:10px; margin-top:16px; }
  .cardActions button { min-width:110px; }
  .scaleRead { margin-top:12px; font-size:26px; font-weight:700; letter-spacing:-.01em; }
  .scaleRead.waiting { font-size:15px; font-weight:500; color:var(--muted); }
  .scaleForm { display:grid; grid-template-columns:2fr 1fr; gap:10px; margin-top:14px; }
  .scaleForm label { display:block; font-size:12px; color:var(--muted); }
  .scaleForm input { margin-top:5px; }
  /* A finished shirt. */
  .shirtCard { text-align:center; width:min(560px,100%); padding:28px; }
  .shirtIcon { width:72px; height:72px; margin:0 auto 12px; border-radius:50%; display:grid; place-items:center;
               font-size:40px; font-weight:800; background:var(--pass); color:var(--pass-ink); }
  .shirtCard.fail .shirtIcon { background:var(--fail); color:#fff; }
  .shirtCard.warn .shirtIcon { background:var(--warn); color:var(--warn-ink); }
  .shirtLabel { font-size:14px; font-weight:700; letter-spacing:.14em; text-transform:uppercase; color:var(--muted); }
  .shirtMain { margin-top:6px; font-size:clamp(26px,3.4vw,42px); font-weight:800; letter-spacing:-.02em; line-height:1.1; }
  .shirtSub { margin-top:8px; color:var(--muted); font-size:16px; }
  .shirtCard button { margin-top:20px; }

  /* ---- the step, beside the picture ---- */
  .side { min-height:0; overflow-y:auto; overscroll-behavior:contain;
          background:var(--bg2); border-left:1px solid var(--line);
          display:flex; flex-direction:column; }
  .sideBody { padding:14px 18px 8px; display:flex; flex-direction:column; gap:14px; }

  /* One segment per step, each a way straight to it. The bar is six pixels and
     a finger is not: the button round it is a full target. */
  .pips { display:flex; gap:0; }
  button.pip { flex:1 1 0; min-width:0; min-height:48px; padding:21px 3px; margin:-8px 0; border:0; border-radius:0;
               background:none; }
  button.pip:hover:not(:disabled) { background:none; }
  button.pip:active:not(:disabled) { transform:none; }
  .pip::before { content:""; display:block; height:6px; border-radius:3px; background:var(--raised2);
                 transition:background-color var(--fast) var(--ease), transform var(--fast) var(--ease); }
  .pip.pass::before { background:var(--pass); }
  .pip.fail::before { background:var(--fail); }
  .pip.flagged::before { background:var(--warn); }
  .pip.current::before { box-shadow:0 0 0 2px var(--bg2), 0 0 0 4px var(--accent); }
  .pip:hover::before { transform:scaleY(1.5); }
  button.pip:focus-visible { outline:none; }
  .pip:focus-visible::before { outline:2px solid var(--accent); outline-offset:4px; }

  .stepHead.enter, .sideBody .enter { animation:stepIn var(--fast) var(--ease); }
  .stepOf { color:var(--muted); font-size:13px; font-weight:600; letter-spacing:.04em; text-transform:uppercase; }
  .stepName { margin:2px 0 0; font-size:28px; font-weight:750; letter-spacing:-.02em; line-height:1.15; word-break:break-word; }
  /* The one line that says what to do now. */
  .doNow { margin:10px 0 0; font-size:21px; font-weight:650; line-height:1.3; color:var(--text); }
  .doNow.fail { color:#FF8A8A; }
  .doNow.warn { color:var(--warn); }
  .doNow.pass { color:var(--pass); }
  .doNow.quiet { color:var(--muted); }
  /* Why the station will not read what is on the mat, in its own words. */
  .said { margin-top:6px; color:var(--warn); font-size:14px; line-height:1.4; }

  /* The reading of this step. */
  .result { padding:16px 18px; border-radius:var(--r); background:var(--solid); border:1px solid var(--line); }
  .result.pass { border-color:rgba(34,197,94,.45); background:linear-gradient(0deg,var(--pass-soft),var(--pass-soft)),var(--solid); }
  .result.fail { border-color:rgba(240,70,70,.45); background:linear-gradient(0deg,var(--fail-soft),var(--fail-soft)),var(--solid); }
  .result.warn { border-color:rgba(245,165,36,.45); background:linear-gradient(0deg,var(--warn-soft),var(--warn-soft)),var(--solid); }
  .rTop { display:flex; align-items:center; justify-content:space-between; gap:10px; }
  .chip { display:inline-flex; align-items:center; gap:6px; padding:5px 12px; border-radius:999px;
          font-size:14px; font-weight:800; letter-spacing:.06em; }
  .chip.pass { background:var(--pass); color:var(--pass-ink); }
  .chip.fail { background:var(--fail); color:#fff; }
  .chip.warn { background:var(--warn); color:var(--warn-ink); }
  .chip.idle { background:var(--raised2); color:var(--muted); letter-spacing:.02em; font-weight:700; }
  .reading { margin-top:10px; font-size:52px; font-weight:800; letter-spacing:-.03em; line-height:1; }
  .reading small { font-size:20px; font-weight:600; color:var(--muted); margin-left:6px; letter-spacing:0; }
  .dev { margin-top:8px; font-size:22px; font-weight:700; }
  .dev.pass { color:var(--pass); }
  .dev.fail { color:#FF7A7A; }
  .dev small { font-size:14px; color:var(--muted); font-weight:500; margin-left:6px; }
  .band { margin-top:8px; color:var(--muted); font-size:13px; }
  .why { margin-top:6px; color:#FF9A9A; font-size:14px; }
  .result.warn .why { color:var(--warn); }
  .moving { margin-top:6px; color:var(--pass); font-size:13px; }

  /* What a pointing, collar or perimeter step needs from the operator. */
  .kind { padding:14px 16px; border-radius:var(--r); background:var(--solid); border:1px solid var(--line); }
  .kindHead { font-size:13px; color:var(--muted); font-weight:600; letter-spacing:.04em; text-transform:uppercase; }
  .kindHead b { color:var(--text); font-size:15px; letter-spacing:0; text-transform:none; }
  .kindHint { margin-top:6px; font-size:14px; color:var(--muted); line-height:1.45; }
  .kindHint.bad { color:#FF9A9A; }
  .kindHint.warn { color:var(--warn); }
  .kindRow { display:flex; flex-wrap:wrap; gap:8px; margin-top:12px; }
  .kindRow button { flex:1 1 auto; }
  .lines { margin-top:10px; font-size:14px; color:var(--muted); }
  .lines div { display:flex; justify-content:space-between; gap:12px; padding:4px 0; border-top:1px solid var(--line); }
  .lines div:first-child { border-top:0; }
  .lines .chosen { color:var(--pass); font-weight:700; }

  /* The one action, and the few beside it. Kept in reach at the bottom of the
     panel however far it has been scrolled. */
  .actions { position:sticky; bottom:0; z-index:2; margin-top:auto; padding:12px 18px 16px;
             background:var(--bg2); border-top:1px solid var(--line);
             display:flex; flex-direction:column; gap:10px; }
  .actionRow { display:flex; gap:10px; }
  .actionRow button { flex:1 1 0; min-width:0; min-height:52px; padding:10px 8px; white-space:nowrap;
                      overflow:hidden; text-overflow:ellipsis; }
  /* Held in place, shown or not, so no button moves under a finger when a
     verdict or a refusal arrives. */
  .actions .reserve { visibility:hidden; }

  /* How to lay it: the reference picture and the style file's words. */
  details.guide { border-radius:var(--r); background:var(--solid); border:1px solid var(--line); overflow:hidden; }
  details.guide summary { list-style:none; cursor:pointer; min-height:48px; padding:12px 16px;
                          display:flex; align-items:center; justify-content:space-between; gap:10px;
                          font-weight:600; color:var(--muted); user-select:none; }
  details.guide summary::-webkit-details-marker { display:none; }
  details.guide summary::after { content:""; width:8px; height:8px; border-right:2px solid var(--muted);
                                 border-bottom:2px solid var(--muted); transform:rotate(45deg);
                                 transition:transform var(--fast) var(--ease); margin-right:4px; }
  details.guide[open] summary::after { transform:rotate(-135deg); }
  .ref { margin:0 12px; border-radius:var(--r-sm); background:var(--raised); overflow:hidden; }
  .ref img { display:block; width:100%; height:auto; }
  .ref .none { padding:22px 14px; text-align:center; color:var(--dim); font-size:13px; }
  .instruction { padding:12px 16px 16px; color:var(--text); font-size:15px; }

  /* The step's saved picture: what the feed showed when its verdict was
     filed. Beside the feed, never on it - the feed is only ever the mat now. */
  .shot { display:grid; grid-template-columns:minmax(0,58%) minmax(0,1fr); gap:12px; align-items:center;
          padding:10px; border-radius:var(--r); background:var(--solid); border:1px solid var(--line); }
  button.shotThumb { position:relative; display:block; width:100%; min-height:0; aspect-ratio:3/2; padding:0;
                     overflow:hidden; border-radius:var(--r-sm); background:var(--raised); border:1px solid var(--line2);
                     cursor:zoom-in; }
  button.shotThumb img { display:block; width:100%; height:100%; object-fit:contain; background:#000; }
  .shotWait { position:absolute; inset:0; display:flex; align-items:center; justify-content:center;
              padding:8px; text-align:center; color:var(--muted); font-size:13px; font-weight:500; }
  .shotText { display:flex; flex-direction:column; align-items:flex-start; gap:6px; min-width:0; }
  .shotNote { color:var(--muted); font-size:13px; }
  .lightbox { position:fixed; inset:0; z-index:40; display:flex; flex-direction:column; background:var(--bg); }
  .lbBar { display:flex; align-items:center; gap:12px; padding:10px 16px; background:var(--bg2); border-bottom:1px solid var(--line); }
  .lbBar b { font-size:15px; }
  .lbBar span { color:var(--muted); font-size:13px; flex:1 1 auto; min-width:0; }
  .lbBar button { min-height:48px; padding:0 22px; }
  .lightbox img { flex:1 1 auto; min-height:0; width:100%; object-fit:contain; cursor:zoom-out; }

  /* ---- the panel's two tabs: the step, and the settings ----
     Always on show at the top of the panel, so a setting is one tap away and
     opens beside the picture rather than over it. */
  .tabs { position:sticky; top:0; z-index:3; display:flex; gap:8px; padding:10px 18px;
          background:var(--bg2); border-bottom:1px solid var(--line); }
  button.tab { flex:1 1 0; min-height:48px; font-size:16px; font-weight:700; color:var(--muted);
               background:transparent; border:1px solid var(--line2); }
  button.tab[aria-selected="true"] { color:var(--text); background:var(--raised2); border-color:var(--accent);
                                     box-shadow:inset 0 -3px 0 var(--accent); }
  .settingsView { gap:0; }
  .stepper { display:flex; align-items:center; gap:12px; }
  .stepper button { width:64px; min-height:56px; padding:0; font-size:26px; font-weight:700; }
  .stepper b { min-width:96px; text-align:center; font-size:30px; font-weight:800; }
  .stepper.words button { width:auto; flex:1 1 0; padding:0 14px; font-size:16px; }
  .rTop button.small { min-height:40px; padding:6px 12px; font-size:13px; color:var(--muted); }
  .sec { padding:16px 0; border-bottom:1px solid var(--line); }
  .sec:last-child { border-bottom:0; }
  .sec h3 { margin:0 0 10px; font-size:13px; color:var(--muted); font-weight:700; letter-spacing:.06em; text-transform:uppercase; }
  .secRow { display:flex; gap:10px; flex-wrap:wrap; }
  .secRow button { flex:1 1 150px; }
  .facts { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:8px; margin-bottom:12px; }
  .fact { padding:10px 12px; border-radius:var(--r-sm); background:var(--raised); }
  .fact b { display:block; font-size:16px; font-weight:700; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .fact span { color:var(--muted); font-size:12px; }
  .fact.ok b { color:var(--pass); }
  .fact.no b { color:var(--warn); }
  .fact.bad b { color:#FF8A8A; }
  .fact.big b { font-size:26px; }
  .recent { margin-top:4px; font-size:13px; }
  .recent div { display:grid; grid-template-columns:auto 1fr auto auto; gap:10px; padding:6px 0;
                border-top:1px solid var(--line); color:var(--muted); }
  .recent div:first-child { border-top:0; }
  .recent b { color:var(--text); font-weight:600; }
  .recent .pass { color:var(--pass); font-weight:700; }
  .recent .fail { color:#FF8A8A; font-weight:700; }
  .recent .warn { color:var(--warn); font-weight:700; }
  .note { color:var(--muted); font-size:13px; margin-top:8px; }
  .range { display:flex; align-items:center; gap:12px; }
  .range input[type=range] { flex:1; min-height:48px; padding:0; border:0; background:transparent; accent-color:var(--accent); }
  .edit { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:10px; }
  .edit label { display:block; font-size:12px; color:var(--muted); margin-bottom:5px; }
  .saved { font-size:13px; margin-top:8px; min-height:18px; }
  /* Adding a style: a name, the style its steps are copied from, and each
     step's numbers. */
  .nsField { margin-bottom:12px; }
  .nsField label, .nsTol label { display:block; font-size:12px; color:var(--muted); margin-bottom:5px; }
  .nsField select { width:100%; }
  .nsTol { display:grid; grid-template-columns:minmax(0,1fr) auto; align-items:end; gap:8px; margin-bottom:6px; }
  .nsTol label { grid-column:1 / -1; margin:0; }
  .nsTol button { min-height:48px; }
  .nsStep { padding:10px 0; border-top:1px solid var(--line); }
  .nsStep:first-child { border-top:0; }
  .nsHead { display:flex; align-items:center; gap:12px; min-height:44px; cursor:pointer; }
  .nsHead input { width:24px; height:24px; min-height:0; padding:0; flex:0 0 auto; accent-color:var(--accent); }
  .nsHead b { font-weight:600; }
  .nsHead span { color:var(--muted); font-size:12px; margin-left:auto; }
  .nsStep .edit { margin-top:6px; }
  .nsStep.off .edit { opacity:.35; }
  .nsSteps { margin-bottom:6px; }
  .toggle { position:relative; display:flex; align-items:center; justify-content:space-between; gap:14px; min-height:56px; cursor:pointer; }
  .toggle input { position:absolute; opacity:0; width:1px; height:1px; }
  .toggle .sw { flex:0 0 auto; width:56px; height:32px; border-radius:16px; background:var(--raised2);
                border:1px solid var(--line2); position:relative; transition:background-color var(--fast) var(--ease); }
  .toggle .sw::after { content:""; position:absolute; top:3px; left:3px; width:24px; height:24px; border-radius:50%;
                       background:var(--muted); transition:transform var(--fast) var(--ease), background-color var(--fast) var(--ease); }
  .toggle input:checked + .sw { background:var(--accent-soft); border-color:var(--accent); }
  .toggle input:checked + .sw::after { transform:translateX(24px); background:var(--accent); }
  .toggle input:focus-visible + .sw { outline:2px solid var(--accent); outline-offset:2px; }
  .log { font-size:12px; color:var(--dim); }
  .log div { padding:4px 0; border-top:1px solid var(--line); word-break:break-word; }
  .log div:first-child { border-top:0; }
  .log b { color:var(--muted); font-weight:500; margin-right:8px; }

  /* What a pip says on hover, for a mouse. */
  .tip { position:fixed; z-index:30; width:max-content; max-width:min(300px, calc(100vw - 16px));
         padding:10px 12px; border:1px solid var(--line2); border-radius:12px; background:var(--glass-strong);
         -webkit-backdrop-filter:blur(14px); backdrop-filter:blur(14px);
         color:var(--text); font-size:13px; line-height:1.4; pointer-events:none; box-shadow:0 10px 30px rgba(0,0,0,.5); }
  .tipHead { color:var(--muted); font-size:11px; letter-spacing:.08em; text-transform:uppercase; }
  .tipName { margin-top:1px; font-size:15px; font-weight:700; }
  .tipState { margin-top:4px; }
  .tipState.pass { color:var(--pass); }
  .tipState.fail { color:#FF8A8A; }
  .tipState.flagged { color:var(--warn); }
  .tipState.pending { color:var(--muted); }
  .tipMeta { margin-top:2px; color:var(--muted); }
  .tipHint { margin-top:6px; color:var(--muted); font-size:12px; }
  .tipImg { display:block; width:240px; max-width:100%; margin-top:8px; border-radius:8px; background:#000; }

  @keyframes spin { to { transform:rotate(360deg); } }
  @keyframes sweep { from { transform:translateX(-100%); } to { transform:translateX(290%); } }
  @keyframes fade { from { opacity:0; } }
  @keyframes bannerIn { from { opacity:0; transform:translateY(-10px) scale(.985); } }
  @keyframes toastIn { from { opacity:0; transform:translate(-50%,-8px); } }
  @keyframes cardIn { from { opacity:0; transform:translateY(8px); } }
  @keyframes stepIn { from { opacity:0; transform:translateX(10px); } }
  @media (prefers-reduced-motion: reduce) {
    *, *::before, *::after { animation-duration:1ms !important; animation-iteration-count:1 !important;
                             transition-duration:1ms !important; }
    .spin { animation:none; }
  }

  /* ---- narrow screens: the picture on top, the step under it ---- */
  .brand, .shirtNo, .lamp { flex-shrink:0; }
  @media (max-width:1000px) {
    /* One row for the bar, however long the shirt number gets, with the
       style picker giving way first; the shift's figures on a second row. */
    .top { gap:8px; padding:6px 6px 6px 10px; flex-wrap:wrap; }
    .stylePick { flex:1 1 0; min-width:0; order:2; }
    .top .spacer { display:none; }
    button.stats { order:4; flex:1 1 100%; gap:14px; min-height:48px; padding:4px 6px; justify-content:space-around; }
    .stats b { font-size:20px; }
    .lamp { order:3; }
    .top .calBtn { order:3; min-width:0; }
  }
  @media (max-width:980px) {
    main { grid-template-columns:1fr; grid-template-rows:minmax(200px,52vh) minmax(0,1fr); }
    .side { border-left:0; border-top:1px solid var(--line); }
  }
  @media (max-width:720px) {
    .brand b { display:none; }
    .stats b { font-size:18px; }
    button.stats { justify-content:space-between; }
    .lamp span { display:none; }
    /* The cards are the whole screen on a phone: inside the picture they
       would be cut off. */
    .over, .over.clearBg { position:fixed; z-index:45; align-items:flex-end; padding:10px; }
    .over.clearBg { background:rgba(5,8,11,.35); }
    /* The picture is as tall as a phone's width makes it anyway; the rest is
       the step's, so the instruction is not under the buttons. */
    main { grid-template-rows:minmax(160px,34vh) minmax(0,1fr); }
    .banner { top:8px; left:8px; right:8px; padding:10px 14px; gap:10px; }
    .bIcon { width:40px; height:40px; font-size:22px; }
    .banner .bGo button { min-width:0; }
    .bCount { width:48px; height:48px; font-size:26px; }
    .stepName { font-size:24px; }
    .doNow { font-size:18px; }
    .reading { font-size:42px; }
    .sideBody { padding:12px 14px 6px; }
    .actions { padding:8px 12px 12px; gap:8px; }
    button.primary { min-height:56px; font-size:18px; }
    .actionRow button { min-height:48px; }
    .facts { grid-template-columns:repeat(2,minmax(0,1fr)); }
  }
</style>
</head>
<body>

<header class="top">
  <div class="brand">
    <svg viewBox="0 0 26 26" aria-hidden="true"><rect x="1" y="1" width="24" height="24" rx="7" fill="#36C8FF" fill-opacity=".14" stroke="#36C8FF" stroke-opacity=".6"/><path d="M7 8.5l6 10 6-10" fill="none" stroke="#36C8FF" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round"/></svg>
    <b>VeriMeasure</b>
    <span class="shirtNo num" id="garment">-</span>
  </div>
  <label class="stylePick" title="Style being checked"><select id="style" aria-label="Style being checked"></select></label>
  <div class="spacer"></div>
  <button class="stats num" id="stats" hidden aria-label="This shift: open the figures">
    <div><b id="stShirts">0</b><span>shirts</span></div>
    <div><b id="stAvg">-</b><span>avg</span></div>
    <div><b id="stRate">-</b><span>per hour</span></div>
  </button>
  <div class="lamp" id="lamp"><i></i><span id="state">starting</span></div>
  <button class="calBtn" id="btnCal" title="Retake the empty base: the camera's exposure and the background, and on the rigid mat the geometry as well">Recalibrate</button>
</header>
<svg width="0" height="0" style="position:absolute" aria-hidden="true" focusable="false">
  <filter id="viewGamma" color-interpolation-filters="sRGB">
    <feComponentTransfer>
      <feFuncR type="gamma" amplitude="1" exponent="1" offset="0"/>
      <feFuncG type="gamma" amplitude="1" exponent="1" offset="0"/>
      <feFuncB type="gamma" amplitude="1" exponent="1" offset="0"/>
    </feComponentTransfer>
  </filter>
</svg>

<main>
  <section class="feed" id="feed" aria-label="Camera">
    <img id="video" alt="" hidden draggable="false">
    <canvas class="ink" id="marks" hidden></canvas>
    <canvas class="ink" id="glow" hidden></canvas>
    <canvas class="ink" id="ink" hidden></canvas>
    <div class="waiting" id="videoWait">Waiting for the camera</div>
    <div class="bar" id="bar"><i id="barFill"></i></div>

    <div class="banner" id="banner" role="status" aria-live="polite" hidden>
      <div class="bIcon" id="bIcon" aria-hidden="true"></div>
      <div style="min-width:0">
        <div class="bWord" id="bWord"></div>
        <div class="bSub" id="bSub"></div>
      </div>
      <div class="bValue num" id="bValue"></div>
      <div class="bGo" id="bGo" hidden>
        <div class="bCount num" id="bCount" aria-hidden="true"></div>
        <div class="bNextWrap">
          <div class="bNext" id="bNext"></div>
          <div class="bMeter" aria-hidden="true"><i id="bFill"></i></div>
        </div>
        <button id="bStart">Next step now</button>
      </div>
    </div>
    <div class="toast" id="toast" role="status" aria-live="polite" hidden></div>

    <div class="over" id="flow" hidden>
      <div class="card shirtCard" id="shirtCard" role="status" aria-live="polite">
        <div class="shirtIcon" id="shirtIcon" aria-hidden="true"></div>
        <div class="shirtLabel" id="shirtLabel">Shirt complete</div>
        <div class="shirtMain" id="shirtMain"></div>
        <div class="shirtSub num" id="shirtSub"></div>
        <button class="primary" id="shirtGo">Next shirt</button>
      </div>
    </div>

    <div class="over" id="calib" hidden>
      <div class="card" id="calibBox" role="status" aria-live="polite">
        <div class="cardHead">
          <div class="spin" id="calibSpin" aria-hidden="true"></div>
          <div class="glyph" id="calibGlyph" aria-hidden="true" hidden></div>
          <div class="cardTitle" id="calibTitle">Recalibrating</div>
          <div class="cardTime num" id="calibTime" aria-hidden="true"></div>
        </div>
        <div class="cardStep" id="calibStep"></div>
        <div class="cardCount num" id="calibCount" aria-hidden="true"></div>
        <div class="cardMeter" id="calibMeter" aria-hidden="true"><i id="calibFill"></i></div>
        <div class="cardNote" id="calibNote"></div>
        <div class="cardDetail" id="calibDetail"></div>
        <div class="cardActions" id="calibActions" hidden>
          <button id="calibCancel" hidden>Cancel</button>
          <button id="calibOk">OK</button>
        </div>
      </div>
    </div>

    <div class="over clearBg" id="scale" hidden>
      <div class="card" id="scaleBox" role="dialog" aria-labelledby="scaleTitle">
        <div class="cardHead">
          <div class="glyph" id="scaleGlyph" aria-hidden="true" style="background:var(--accent)">&#8596;</div>
          <div class="cardTitle" id="scaleTitle">Check scale</div>
        </div>
        <div class="cardStep" id="scaleStep">Lay a steel rule flat on the base, on its own, and let it read.</div>
        <div class="scaleRead num" id="scaleRead">Waiting for the reference</div>
        <div class="scaleForm">
          <label>True length, end to end<input id="scaleTrue" class="num" type="number" inputmode="decimal" min="1" step="0.1" placeholder="mm"></label>
          <label>Thickness<input id="scaleThick" class="num" type="number" inputmode="decimal" min="0" max="20" step="0.1"></label>
        </div>
        <div class="cardNote" id="scaleNote"></div>
        <div class="cardDetail" id="scaleOutcome"></div>
        <div class="cardActions">
          <button id="scaleClose">Close</button>
          <button class="primary" id="scaleApply" style="width:auto;min-height:48px" disabled>Apply</button>
        </div>
      </div>
    </div>
  </section>

  <aside class="side" id="side" aria-label="Step and settings">
    <div class="tabs" role="tablist" aria-label="Panel">
      <button class="tab" role="tab" id="tabMeasure" aria-selected="true" aria-controls="measureView">Measure</button>
      <button class="tab" role="tab" id="tabSettings" aria-selected="false" aria-controls="settingsView">Settings</button>
    </div>
    <div class="sideBody" id="measureView" role="tabpanel" aria-labelledby="tabMeasure">
      <nav class="pips" id="strip" aria-label="Steps"></nav>

      <div class="stepHead" id="stepHead">
        <div class="stepOf num" id="stepOf">No style selected</div>
        <h1 class="stepName" id="stepName">-</h1>
        <p class="doNow" id="doNow">Choose a style to start</p>
        <div class="said" id="said" hidden></div>
      </div>

      <div class="result" id="resultCard">
        <div class="rTop">
          <span class="chip idle" id="flag">Waiting</span>
          <button class="small ghost" id="btnEditTarget">Edit target</button>
        </div>
        <div class="reading num" id="reading" hidden><span id="mm">-</span><small>mm</small></div>
        <div class="dev num" id="dev" hidden><span id="devPct">-</span><small id="devNote"></small></div>
        <div class="band num" id="band"></div>
        <div class="why" id="why"></div>
        <div class="moving num" id="moving"></div>
      </div>

      <div class="kind" id="pointing" hidden>
        <div class="kindHead"><b id="pointCount" class="num">0 of 3</b> points</div>
        <div class="kindHint" id="pointHint"></div>
        <div class="kindRow">
          <button class="small" id="btnMarkUndo">Undo point</button>
          <button class="small" id="btnMarkClear">Clear</button>
          <button class="small" id="btnMarkGo">Read the seam</button>
        </div>
      </div>

      <div class="kind" id="collarStrip" hidden>
        <div class="kindHead"><b id="collarHead">Collar</b></div>
        <div class="kindHint" id="collarHint"></div>
        <div class="lines num" id="collarLines"></div>
        <div class="kindRow">
          <button class="small" id="btnCollarClear">Clear the box</button>
        </div>
      </div>

      <div class="kind" id="perimStrip" hidden>
        <div class="kindHead"><b id="perimHead">Full perimeter</b></div>
        <div class="kindHint" id="perimHint"></div>
        <div class="lines num" id="perimHalves"></div>
      </div>

      <section class="shot" id="shot" hidden aria-label="Saved picture of this step">
        <button class="shotThumb" id="shotOpen" type="button" aria-label="Enlarge the saved picture">
          <img id="shotImg" alt="" hidden>
          <span class="shotWait" id="shotWait">Saving...</span>
        </button>
        <div class="shotText">
          <div class="kindHead">Saved picture</div>
          <span class="chip idle" id="shotChip">-</span>
          <div class="shotNote num" id="shotNote"></div>
        </div>
      </section>

      <details class="guide" id="guide" open>
        <summary>How to lay it</summary>
        <div class="ref" id="ref">
          <img id="refImg" alt="" hidden>
          <div class="none" id="refNone">No reference picture for this step</div>
        </div>
        <div class="instruction" id="instruction"></div>
      </details>
    </div>

    <div class="sideBody settingsView" id="settingsView" role="tabpanel" aria-labelledby="tabSettings" hidden>
      <section class="sec">
        <h3>Target for this step</h3>
        <div class="note" id="editFor" style="margin:0 0 10px"></div>
        <div class="edit num">
          <div><label for="eTarget">target mm</label><input id="eTarget" type="number" step="0.1" inputmode="decimal"></div>
          <div><label for="eMinus">minus mm</label><input id="eMinus" type="number" step="0.1" inputmode="decimal"></div>
          <div><label for="ePlus">plus mm</label><input id="ePlus" type="number" step="0.1" inputmode="decimal"></div>
        </div>
        <div class="secRow" style="margin-top:10px"><button id="btnSave">Save to the style file</button></div>
        <div class="saved" id="saved"></div>
      </section>
      <section class="sec" id="styleSec">
        <h3>Styles</h3>
        <div class="secRow" id="nsOpenRow"><button id="nsOpen">Add a style</button></div>
        <div id="nsForm" hidden>
          <div class="nsField"><label for="nsName">Name</label>
            <input id="nsName" type="text" maxlength="60" autocomplete="off" spellcheck="false" placeholder="e.g. IVT BLACK T SHIRT TESTER"></div>
          <div class="nsField"><label for="nsBase">Steps copied from</label><select id="nsBase"></select></div>
          <div class="nsTol num">
            <label for="nsTol">Plus and minus for every step, mm</label>
            <input id="nsTol" type="number" step="0.1" min="0" inputmode="decimal" value="10">
            <button id="nsTolApply">Apply to all</button>
          </div>
          <div class="nsSteps num" id="nsSteps"></div>
          <label class="toggle">
            <span>Check this style as soon as it is added</span>
            <input type="checkbox" id="nsUseNow" checked><span class="sw" aria-hidden="true"></span>
          </label>
          <div class="secRow" style="margin-top:6px"><button id="nsAdd">Add the style</button><button id="nsCancel" class="ghost">Cancel</button></div>
        </div>
        <div class="saved" id="nsMsg"></div>
        <div class="note">Each step keeps the folding, the picture and the words of the style it is copied from; only the numbers are typed in. The new style is saved as its own file in the templates folder and appears in the list at the top.</div>
      </section>
      <section class="sec">
        <h3>After a pass</h3>
        <div class="stepper num">
          <button id="delayMinus" aria-label="Shorter wait">&#8722;</button>
          <b id="delayValue">5 s</b>
          <button id="delayPlus" aria-label="Longer wait">+</button>
        </div>
        <div class="note">The result stays up and counts down for this long, then the station moves to the next step. Next step now skips the wait.</div>
      </section>
      <section class="sec" id="expoSec">
        <h3>Camera exposure</h3>
        <div class="facts num">
          <div class="fact"><b id="expoNow">-</b><span>exposure</span></div>
          <div class="fact"><b id="expoLevelNow">-</b><span>base metered to</span></div>
          <div class="fact" id="expoClipFact"><b id="expoClip">-</b><span>last shirt clipped</span></div>
        </div>
        <div class="stepper words num">
          <button id="expoDarker">&#8722; Darker</button>
          <b id="expoLevel">-</b>
          <button id="expoBrighter">Brighter +</button>
        </div>
        <div class="secRow" style="margin-top:10px"><button id="expoApply" disabled>Set the camera</button></div>
        <div class="note" id="expoNote"></div>
      </section>

      <section class="sec">
        <h3>Screen brightness</h3>
        <label class="range" title="How bright the picture looks on this screen. Only the picture: the camera's exposure and every measurement stay as they are.">
          <span>Brightness</span>
          <input type="range" id="viewLight" min="1" max="3" step="0.1" value="2.5" aria-label="Picture brightness on this screen">
        </label>
        <div class="note">On this screen only. The camera's exposure and every measurement stay as they are.</div>
      </section>
      <section class="sec">
        <h3>Calibration</h3>
        <div class="facts num">
          <div class="fact" id="scaleFact"><b id="scaleState">-</b><span>scale</span></div>
          <div class="fact"><b id="rmse">-</b><span>grid fit, mm</span></div>
          <div class="fact"><b id="fps">-</b><span>frames / s</span></div>
        </div>
        <div class="secRow">
          <button id="btnScale" title="Check the scale against a steel rule of known length laid on the base">Check scale</button>
        </div>
        <div class="note">Recalibrate is in the bar along the top.</div>
      </section>
      <section class="sec">
        <h3>Hand-drawn collar steps</h3>
        <label class="toggle">
          <span>Show tracing the neckline and tapping the shoulder points, as a fallback to the box</span>
          <input type="checkbox" id="manualSteps"><span class="sw" aria-hidden="true"></span>
        </label>
        <div class="note" id="manualNote"></div>
      </section>
      <section class="sec" id="statsSec" hidden>
        <h3>This shift <span id="stSince" style="text-transform:none;letter-spacing:0;font-weight:500"></span></h3>
        <div class="note" style="margin:-4px 0 10px"><b id="sdShirts" style="color:var(--text)">0</b> shirts</div>
        <div class="facts num">
          <div class="fact big" id="sdPassedFact"><b id="sdPassed">0</b><span>passed</span></div>
          <div class="fact big" id="sdFailedFact"><b id="sdFailed">0</b><span>failed</span></div>
          <div class="fact big" id="sdQcFact"><b id="sdQc">0</b><span>manual QC</span></div>
          <div class="fact"><b id="sdAvg">-</b><span>average</span></div>
          <div class="fact"><b id="sdMedian">-</b><span>median</span></div>
          <div class="fact"><b id="sdRate">-</b><span>per hour</span></div>
          <div class="fact"><b id="sdLast">-</b><span id="sdLastLabel">last shirt</span></div>
          <div class="fact"><b id="sdLastAt">-</b><span>finished at</span></div>
          <div class="fact"><b id="sdIdle">0</b><span>idle</span></div>
        </div>
        <div class="recent num" id="sdRecent"></div>
        <div class="secRow" style="margin-top:12px"><button id="btnNewShift">New shift</button></div>
        <div class="note" id="sdNote">A shirt counts when every step has its verdict. A shirt moved on from early is not counted, and a step redone updates the same shirt. Time runs from the first lay to the last verdict. A shirt over 5 minutes is counted as idle and kept out of the average, median and rate. The rate is the pace from one result to the next. Starts again at midnight.</div>
      </section>
      <section class="sec">
        <h3>Station log</h3>
        <div class="log num" id="log"></div>
      </section>
    </div>

    <div class="actions">
      <button class="primary" id="btnPrimary">Next step</button>
      <div class="actionRow" id="secondaryRow" hidden>
        <button class="big warnBtn" id="btnSecondary" style="flex:2 1 0">Flag for manual QC</button>
        <button class="big" id="btnNext" style="flex:1 1 0">Skip step</button>
      </div>
      <div class="actionRow">
        <button id="btnBack">Back</button>
        <button id="btnRetake">Retake</button>
        <button id="btnShirt">Next shirt</button>
      </div>
    </div>
  </aside>
</main>


<div class="lightbox" id="shotBig" role="dialog" aria-modal="true" aria-labelledby="shotBigTitle" hidden>
  <div class="lbBar"><b id="shotBigTitle">Saved picture</b><span id="shotBigNote"></span><button id="shotClose">Close</button></div>
  <img id="shotBigImg" alt="">
</div>

<div class="tip" id="tip" role="tooltip" hidden>
  <div class="tipHead" id="tipHead"></div>
  <div class="tipName" id="tipName"></div>
  <div class="tipState num" id="tipState"></div>
  <div class="tipMeta num" id="tipMeta"></div>
  <div class="tipHint" id="tipHint"></div>
  <img class="tipImg" id="tipImg" alt="" hidden>
</div>

<script>
"use strict";

const $ = (id) => document.getElementById(id);
// Whether a tap on the feed means anything right now. Only on a pointing step,
// and only once the reading is held: before that the picture is still moving
// and a point on it is a point on nothing in particular.
let pointable = false;
// Whether a drag on the feed draws a box round the collar: a collar step, with
// the reading held. The box says where the collar is and nothing more.
let boxable = false;
const fmt = (v, n) => (v === null || v === undefined || isNaN(v)) ? "-" : Number(v).toFixed(n);
// Always signed: which side of the target the garment came out on is the whole
// point of the figure, and an unsigned one reads as though it passed.
const pct = (v, n) => {
  if (v === null || v === undefined || isNaN(v)) return "-";
  const s = Number(v).toFixed(n === undefined ? 2 : n);
  return (Number(s) >= 0 && s[0] !== "-" ? "+" : "") + s + "%";
};
const MINUS = String.fromCharCode(0x2212);
// A minus sign the width of a plus, so a signed figure does not jitter.
const signed = (t) => String(t).replace(/^-/, MINUS);
const TICK = String.fromCharCode(0x2713);
const CROSS = String.fromCharCode(0x2715);
const ARROW = String.fromCharCode(0x2192);
const DOT = "  " + String.fromCharCode(0x00b7) + "  ";
const nice = (name) => {
  const t = String(name || "").replace(/_/g, " ");
  return t.charAt(0).toUpperCase() + t.slice(1);
};
const cap = (t) => t ? t.charAt(0).toUpperCase() + t.slice(1) : "";
const reduced = () => window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;

let shownStep = null;     // which step the reference image currently shows
let editDirty = false;    // do not overwrite numbers being typed
let lastStyleList = "";
let lastStatus = null;

async function send(name, arg) {
  const q = arg === undefined ? "" : "&arg=" + encodeURIComponent(arg);
  try { await fetch("/api/command?name=" + name + q, { method: "POST" }); }
  catch (e) { /* the next poll will show whether it landed */ }
}

// The buttons that move the bench on - the big one, the ones beside it, the
// banner's and the finished shirt's - and the keys that stand for them share
// one short lock. A finger bouncing on the glass is two taps a fifth of a
// second apart, and the big button can say something else by the second one:
// Next on the second-last step becomes Next shirt on the last. Half a second
// is shorter than anyone deliberately presses twice, and longer than a bounce.
// Back and Retake mean the same thing however often they are pressed, so each
// only locks itself, for as long as a bounce lasts.
//
// A move - Next, Next shirt, Redraw, Flag, Start now - locks every other move
// for a second from the press, and for 0.7 s from whenever the big button
// then changes what it says, whichever ends later. The page hears about the
// move only at its next poll, a fifth to half a second later, so a lock timed
// from the relabel alone lets a second press land on the new meaning. Moving
// two steps on purpose is the strip's job.
const MOVE_LOCK_MS = 1000;
const RELABEL_LOCK_MS = 700;
const BOUNCE_MS = 300;
const actionLockUntil = {};
let primaryChangedAt = 0;
function act(fn, group) {
  const g = group || "moves";
  return (e) => {
    const now = Date.now();
    if (now < (actionLockUntil[g] || 0)) return;
    if (g === "moves" && now - primaryChangedAt < RELABEL_LOCK_MS) return;
    actionLockUntil[g] = now + (g === "moves" ? MOVE_LOCK_MS : BOUNCE_MS);
    // A pointer leaves the keyboard where it was, so Enter goes on meaning
    // Next rather than pressing this again.
    if (e && e.detail > 0 && e.currentTarget && e.currentTarget.blur) e.currentTarget.blur();
    fn(e);
  };
}

$("btnMarkUndo").onclick  = () => send("mark_undo");
$("btnMarkClear").onclick = () => send("mark_clear");
$("btnMarkGo").onclick    = () => send("mark_measure");
$("btnCollarClear").onclick = () => send("collar_clear");

// -- drawing on the picture --
//
// A tap puts a point down, a drag draws the seam, and on a collar step a drag
// draws the box. Sent as fractions of the picture, never as pixels: the feed is
// scaled once on the station and again by this screen, and only a fraction
// means the same thing at both ends. The image is sized to its own contents,
// so its bounding box IS the picture.
let drawing = null;

function feedPoint(e, el) {
  const r = el.getBoundingClientRect();
  if (!(r.width > 0 && r.height > 0)) return null;
  const fx = (e.clientX - r.left) / r.width;
  const fy = (e.clientY - r.top) / r.height;
  if (fx < 0 || fx > 1 || fy < 0 || fy > 1) return null;
  return [fx, fy];
}

// The drag, drawn as it happens: the station does not hear about it until the
// finger comes up, and a curve traced blind is not the curve.
const ink = $("ink");
function overPicture(canvas, img) {
  const r = img.getBoundingClientRect();
  const host = img.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.style.left = (r.left - host.left) + "px";
  canvas.style.top = (r.top - host.top) + "px";
  canvas.style.width = r.width + "px";
  canvas.style.height = r.height + "px";
  canvas.width = Math.round(r.width * dpr);
  canvas.height = Math.round(r.height * dpr);
  canvas.hidden = false;
}
const accent = () => getComputedStyle(document.documentElement).getPropertyValue("--accent").trim() || "#36C8FF";
function inkStart(img) { overPicture(ink, img); }
function inkDraw(path) {
  const ctx = ink.getContext("2d");
  ctx.clearRect(0, 0, ink.width, ink.height);
  if (path.length < 2) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  const trace = () => {
    ctx.beginPath();
    path.forEach((p, i) => (i ? ctx.lineTo : ctx.moveTo).call(ctx, p[0] * ink.width, p[1] * ink.height));
    ctx.stroke();
  };
  ctx.strokeStyle = "#0A0F14"; ctx.lineWidth = 5 * dpr; trace();   // the dark backing
  ctx.strokeStyle = accent();  ctx.lineWidth = 2.5 * dpr; trace(); // the line
}
function inkClear() {
  const ctx = ink.getContext("2d");
  ctx.clearRect(0, 0, ink.width, ink.height);
  ink.hidden = true;
}

// The box as it is dragged, dashed. The station draws the box it was sent
// once it has looked inside it, and this one comes off then.
function inkBox(a, b) {
  const ctx = ink.getContext("2d");
  ctx.clearRect(0, 0, ink.width, ink.height);
  const dpr = window.devicePixelRatio || 1;
  const x = Math.min(a[0], b[0]) * ink.width, y = Math.min(a[1], b[1]) * ink.height;
  const w = Math.abs(b[0] - a[0]) * ink.width, h = Math.abs(b[1] - a[1]) * ink.height;
  ctx.setLineDash([]);
  ctx.strokeStyle = "#0A0F14"; ctx.lineWidth = 5 * dpr; ctx.strokeRect(x, y, w, h);
  ctx.fillStyle = "rgba(54,200,255,.08)"; ctx.fillRect(x, y, w, h);
  ctx.setLineDash([9 * dpr, 6 * dpr]);
  ctx.strokeStyle = accent(); ctx.lineWidth = 2.5 * dpr; ctx.strokeRect(x, y, w, h);
  ctx.setLineDash([]);
}

// A point held to the picture's edges, so a box can be dragged out to the
// very border of the feed without the corner being lost.
function feedPointHeld(e, el) {
  const r = el.getBoundingClientRect();
  if (!(r.width > 0 && r.height > 0)) return null;
  const fx = Math.min(1, Math.max(0, (e.clientX - r.left) / r.width));
  const fy = Math.min(1, Math.max(0, (e.clientY - r.top) / r.height));
  return [fx, fy];
}

let boxing = null;       // the box being dragged: {a, b} as fractions of the picture
let collarAsked = null;  // {seq, at}: a box sent and not yet answered

$("video").addEventListener("contextmenu", (e) => { if (pointable || boxable) e.preventDefault(); });

$("video").addEventListener("pointerdown", (e) => {
  if (!e.isPrimary) return;  // a palm or a second finger is not a new drag
  if (boxable) {
    const p = feedPoint(e, e.currentTarget);
    if (!p) return;
    boxing = { a: p, b: p };
    inkStart(e.currentTarget);
    e.currentTarget.setPointerCapture(e.pointerId);
    e.preventDefault();
    return;
  }
  if (!pointable) return;
  const p = feedPoint(e, e.currentTarget);
  if (!p) return;
  drawing = [p];
  inkStart(e.currentTarget);
  e.currentTarget.setPointerCapture(e.pointerId);
  e.preventDefault();
});

$("video").addEventListener("pointermove", (e) => {
  if (!e.isPrimary) return;
  if (boxing) {
    const p = feedPointHeld(e, e.currentTarget);
    if (!p) return;
    boxing.b = p;
    inkBox(boxing.a, boxing.b);
    return;
  }
  if (!drawing) return;
  const p = feedPoint(e, e.currentTarget);
  if (!p) return;
  // Kept only when it has moved, so a finger resting on the glass does not
  // fill the path with copies of one place. A fifth of a percent of the width
  // is about a millimetre and a half on the mat.
  const last = drawing[drawing.length - 1];
  if (Math.hypot(p[0] - last[0], p[1] - last[1]) < 0.002) return;
  drawing.push(p);
  inkDraw(drawing);
});

$("video").addEventListener("pointerup", (e) => {
  if (!e.isPrimary) return;
  if (boxing) {
    const box = boxing;
    boxing = null;
    try { e.currentTarget.releasePointerCapture(e.pointerId); } catch (err) {}
    const w = Math.abs(box.b[0] - box.a[0]), h = Math.abs(box.b[1] - box.a[1]);
    if (w > 0.02 && h > 0.02) {
      // Said at once: the station takes the best part of a second over it.
      const c = lastStatus && lastStatus.collar;
      collarAsked = { seq: c ? (c.seq || 0) : 0, at: Date.now() };
      if (lastStatus) { renderCollar(lastStatus); renderActions(lastStatus); }
      send("box", [box.a[0], box.a[1], box.b[0], box.b[1]].map((v) => v.toFixed(5)).join(","));
    } else {
      // Not a box, a tap: when lines are offered for choosing, it takes one -
      // but only with the hand-drawn collar steps switched on. Otherwise a
      // line the machine could not vouch for is not a reading to be tapped in.
      inkClear();
      if (lastStatus && lastStatus.manual_steps) {
        send("collar_pick", box.a[0].toFixed(5) + "," + box.a[1].toFixed(5));
      }
    }
    return;
  }
  if (!drawing) return;
  const path = drawing;
  drawing = null;
  inkClear();
  try { e.currentTarget.releasePointerCapture(e.pointerId); } catch (err) {}
  const arg = path.map(p => p[0].toFixed(5) + "," + p[1].toFixed(5)).join(";");
  // Three points is a line drawn on purpose; fewer is a tap on one place.
  if (path.length >= 3) send("trace", arg);
  else send("mark", path[0][0].toFixed(5) + "," + path[0][1].toFixed(5));
});

$("video").addEventListener("pointercancel", (e) => {
  if (!e.isPrimary) return;
  drawing = null;
  boxing = null;
  inkClear();
});

// -- the actions --
$("btnBack").onclick   = act(() => send("back"), "back");
$("btnRetake").onclick = act(() => { forgetBox(); send("retake"); }, "retake");
$("btnNext").onclick   = act(() => nextStep());
$("btnShirt").onclick  = act(() => send("next_shirt"));
$("style").onchange    = (e) => send("template", e.target.value);
$("btnSecondary").onclick = act(() => send("flag_manual_qc"));

// The big button does the one thing that comes next, whatever that is now.
let primaryDoes = "next";
$("btnPrimary").onclick = act(() => {
  if (primaryDoes === "next" || primaryDoes === "advance") nextStep();
  else if (primaryDoes === "shirt") send("next_shirt");
  else if (primaryDoes === "redraw") send("collar_clear");
});

for (const id of ["eTarget", "eMinus", "ePlus"]) {
  $(id).oninput = () => { editDirty = true; };
}

$("btnSave").onclick = async () => {
  if (shownStep === null) return;
  const q = new URLSearchParams({
    index: String(shownStep),
    target_mm: $("eTarget").value,
    tol_minus_mm: $("eMinus").value,
    tol_plus_mm: $("ePlus").value
  });
  const out = $("saved");
  try {
    const r = await fetch("/api/step?" + q.toString(), { method: "POST" });
    const j = await r.json();
    editDirty = false;
    out.textContent = j.ok ? "Saved" : ("Not saved: " + (j.error || "refused"));
    out.style.color = j.ok ? "var(--pass)" : "#FF8A8A";
  } catch (e) {
    out.textContent = "Not saved: the station did not answer";
    out.style.color = "#FF8A8A";
  }
  setTimeout(() => { out.textContent = ""; }, 4000);
};

// -- adding a style --
//
// Made from a style already here: the station is asked for that style's
// steps, each is shown with its numbers to change and a tick to keep it, and
// the station copies everything else about each step into a new file.
const ns = { base: "", steps: [] };
const mmText = (v) => String(Math.round(Math.abs(Number(v)) * 10) / 10);
function nsShow(open) {
  $("nsForm").hidden = !open;
  $("nsOpenRow").hidden = open;
  if (!open) return;
  const list = (lastStatus && lastStatus.styles) || [];
  const sel = $("nsBase");
  sel.innerHTML = "";
  for (const st of list) {
    const o = document.createElement("option");
    o.value = st.style;
    o.textContent = st.style;
    sel.appendChild(o);
  }
  const current = lastStatus && lastStatus.style;
  sel.value = current && list.some((st) => st.style === current) ? current : (list[0] ? list[0].style : "");
  $("nsMsg").textContent = "";
  nsLoad(sel.value);
}
async function nsLoad(style) {
  ns.base = style;
  ns.steps = [];
  const box = $("nsSteps");
  box.textContent = style ? "Reading " + style + "..." : "There is no style to copy the steps from.";
  if (!style) return;
  let j = null;
  try { j = await (await fetch("/api/style?name=" + encodeURIComponent(style))).json(); } catch (e) { j = null; }
  if (ns.base !== style) return;  // another was chosen meanwhile
  if (!j || !j.ok) {
    box.textContent = "Could not read " + style + (j && j.error ? ": " + j.error : "");
    return;
  }
  ns.steps = j.steps;
  box.innerHTML = "";
  j.steps.forEach((st, k) => {
    const row = document.createElement("div");
    row.className = "nsStep";
    const head = document.createElement("label");
    head.className = "nsHead";
    const tick = document.createElement("input");
    tick.type = "checkbox";
    tick.checked = !!st.on_bench;
    const name = document.createElement("b");
    name.textContent = nice(st.name);
    const kind = document.createElement("span");
    kind.textContent = st.kind === "edge" ? "fold" : st.kind;
    head.append(tick, name, kind);
    const edit = document.createElement("div");
    edit.className = "edit num";
    for (const [f, label, value] of [["t", "target mm", st.target_mm], ["m", "minus mm", st.tol_minus_mm],
                                     ["p", "plus mm", st.tol_plus_mm]]) {
      const cell = document.createElement("div");
      const id = "ns" + f + k;
      const l = document.createElement("label");
      l.htmlFor = id;
      l.textContent = label;
      const input = document.createElement("input");
      input.id = id;
      input.type = "number";
      input.step = "0.1";
      input.inputMode = "decimal";
      input.dataset.f = f;
      input.value = mmText(value);
      cell.append(l, input);
      edit.appendChild(cell);
    }
    const mark = () => row.classList.toggle("off", !tick.checked);
    tick.onchange = mark;
    mark();
    row.append(head, edit);
    box.appendChild(row);
  });
}
$("nsOpen").onclick = () => nsShow(true);
$("nsCancel").onclick = () => nsShow(false);
$("nsBase").onchange = (e) => nsLoad(e.target.value);
$("nsTolApply").onclick = () => {
  const v = Math.abs(Number($("nsTol").value));
  if (!($("nsTol").value !== "" && isFinite(v))) return;
  for (const i of $("nsSteps").querySelectorAll('input[data-f="m"], input[data-f="p"]')) i.value = mmText(v);
};
$("nsAdd").onclick = async () => {
  if ($("nsAdd").disabled) return;
  const msg = $("nsMsg");
  const refuse = (text) => { msg.textContent = text; msg.style.color = "#FF8A8A"; };
  const name = $("nsName").value.trim();
  if (!name) return refuse("Give the new style a name.");
  const kept = [];
  $("nsSteps").querySelectorAll(".nsStep").forEach((row, k) => {
    if (!row.querySelector('input[type="checkbox"]').checked) return;
    const val = (f) => row.querySelector('input[data-f="' + f + '"]').value.trim();
    kept.push([ns.steps[k].name, val("t"), val("m"), val("p")]);
  });
  if (!kept.length) return refuse("Tick at least one step.");
  for (const k of kept) {
    if (!(Number(k[1]) > 0)) return refuse("Give " + nice(k[0]) + " a target in millimetres.");
    if (k[2] === "" || k[3] === "" || !isFinite(Number(k[2])) || !isFinite(Number(k[3]))) {
      return refuse("Give " + nice(k[0]) + " its minus and plus in millimetres.");
    }
  }
  const q = new URLSearchParams({ name: name, base: ns.base, steps: kept.map((k) => k.join(":")).join(",") });
  $("nsAdd").disabled = true;
  let j;
  try { j = await (await fetch("/api/style_add?" + q.toString(), { method: "POST" })).json(); }
  catch (e) { j = { ok: false, error: "the station did not answer" }; }
  $("nsAdd").disabled = false;
  if (!j.ok) return refuse("Not added: " + (j.error || "the station refused it"));
  const useNow = $("nsUseNow").checked;
  msg.textContent = "Added " + j.style + ", " + kept.length + (kept.length === 1 ? " step" : " steps") +
                    (useNow ? ". Checking it now." : ". Pick it from the list at the top.");
  msg.style.color = "var(--pass)";
  if (useNow) send("template", j.style);
  $("nsName").value = "";
  nsShow(false);
};

// The switch shows what the station says, except for a moment after it is
// flipped here, while the station takes it up.
let manualChangedAt = 0;
$("manualSteps").onchange = (e) => {
  manualChangedAt = Date.now();
  send("manual_steps", e.target.checked ? "on" : "off");
};

// -- the settings drawer --
// -- the panel: the step, or the settings --
//
// Two tabs at the top of the panel, always on show. The settings open in the
// panel itself, one tap away, and the picture stays in view.
function settingsShown() { return !$("settingsView").hidden; }
// Brings a section of the panel to the top of the panel, under its tabs.
// Scrolled by hand: scrollIntoView would move the whole page too, and the page
// is laid out to the window with nothing to scroll.
function panelScrollTo(el) {
  const side = $("side");
  const tabs = document.querySelector(".tabs");
  side.scrollTop += el.getBoundingClientRect().top - side.getBoundingClientRect().top -
                    (tabs ? tabs.offsetHeight : 0) - 8;
}
function showSettings(on) {
  $("settingsView").hidden = !on;
  $("measureView").hidden = on;
  $("tabSettings").setAttribute("aria-selected", on ? "true" : "false");
  $("tabMeasure").setAttribute("aria-selected", on ? "false" : "true");
  $("side").scrollTop = 0;
}
$("tabMeasure").onclick = (e) => { if (e.detail > 0) e.currentTarget.blur(); showSettings(false); };
$("tabSettings").onclick = (e) => { if (e.detail > 0) e.currentTarget.blur(); showSettings(true); };
$("btnEditTarget").onclick = (e) => {
  if (e.detail > 0) e.currentTarget.blur();
  showSettings(true);
  $("eTarget").focus();
};

// How long a pass is counted down before the station moves on. The station
// keeps it; the setting shows what it says, except for a moment after a press.
let delayWanted = null;
let delayChangedAt = 0;
function setDelay(by) {
  const now = Date.now();
  const have = delayWanted !== null && now - delayChangedAt < 1500 ? delayWanted
             : Number((lastStatus && lastStatus.advance_delay_s) || 5);
  const v = Math.min(30, Math.max(1, Math.round(have) + by));
  delayWanted = v;
  delayChangedAt = now;
  $("delayValue").textContent = v + " s";
  send("advance_delay", String(v));
}
$("delayMinus").onclick = () => setDelay(-1);
$("delayPlus").onclick = () => setDelay(1);

// How bright the picture looks on this screen, and nothing else: a gamma on
// the picture shown, so the base comes up and white stays white. The frames
// the station measures, and the exposure, do not change. Remembered in this
// browser.
//
// High by default: the camera is exposed for the palest shirt not to clip,
// which leaves the base dark on screen. The key is new with that default, so a
// screen left at the old one of 1 comes up bright once too.
const VIEW_KEY = "verimeasure.viewLight.2";
const VIEW_DEFAULT = 2.5;
function applyViewLight(value) {
  const lift = Math.max(1, Math.min(3, Number(value) || VIEW_DEFAULT));
  const exponent = String(1 / lift);
  for (const f of document.querySelectorAll("#viewGamma feFuncR, #viewGamma feFuncG, #viewGamma feFuncB")) {
    f.setAttribute("exponent", exponent);
  }
  $("video").style.filter = lift > 1.001 ? "url(#viewGamma)" : "";
  $("viewLight").value = String(lift);
  return lift;
}
(function () {
  let saved = VIEW_DEFAULT;
  try { saved = Number(localStorage.getItem(VIEW_KEY)) || VIEW_DEFAULT; } catch (e) {}
  applyViewLight(saved);
  $("viewLight").addEventListener("input", (e) => {
    const lift = applyViewLight(e.target.value);
    try { localStorage.setItem(VIEW_KEY, String(lift)); } catch (err) {}
  });
})();

// Whether "How to lay it" is open, remembered in this browser.
const GUIDE_KEY = "verimeasure.guideOpen";
(function () {
  try { const v = localStorage.getItem(GUIDE_KEY); if (v !== null) $("guide").open = v === "1"; } catch (e) {}
  $("guide").addEventListener("toggle", () => {
    try { localStorage.setItem(GUIDE_KEY, $("guide").open ? "1" : "0"); } catch (e) {}
  });
})();

// Keyboard, because at a bench a hand is usually holding fabric.
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape" && settingsShown()) { showSettings(false); return; }
  if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
  // Settings, and checking the scale, step nothing on: Enter on their buttons is theirs.
  if (scale.open || settingsShown()) return;
  // A button with the keyboard's focus is pressed like the button it is -
  // Tab to "Redraw the box" and Enter redraws. A pointer never leaves one
  // focused (see act), so at the bench Enter and Space mean Next.
  const own = e.target.tagName === "BUTTON" || e.target.tagName === "SUMMARY";
  if ((e.key === "Enter" || e.key === " ") && !own) { e.preventDefault(); if (!e.repeat) keyNext(); }
  if ((e.key === "n" || e.key === "N") && !e.repeat) keyShirt();
  if ((e.key === "r" || e.key === "R") && !e.repeat) keyRetake();
});
const keyNext = act(() => nextStep());
const keyShirt = act(() => send("next_shirt"));
const keyRetake = act(() => { forgetBox(); send("retake"); }, "retake");

function renderStyles(list, current) {
  const key = JSON.stringify(list) + "|" + String(current);
  if (key === lastStyleList) return;
  lastStyleList = key;
  const sel = $("style");
  sel.innerHTML = "";
  const none = document.createElement("option");
  none.value = ""; none.textContent = "No style";
  sel.appendChild(none);
  for (const s of list || []) {
    const o = document.createElement("option");
    o.value = s.style;
    o.textContent = s.style + "  (" + s.steps + " steps)";
    sel.appendChild(o);
  }
  sel.value = current || "";
}

// -- the strip of steps --
//
// Updated in place rather than rebuilt: the page redraws four times a second,
// and a pip rebuilt between a finger going down and coming up is a press on an
// element that no longer exists.
let stripSteps = [];
const tip = { pip: -1, source: "" };

function pipState(s) {
  return s.manual_qc ? "flagged" : s.state;
}
function pipLabel(s, k) {
  const what = s.manual_qc ? "flagged for manual QC" : s.state === "pass" ? "passed"
             : s.state === "fail" ? "failed" : "not measured";
  return "Step " + (k + 1) + " of " + stripSteps.length + ", " + nice(s.name) + ": " + what +
         (s.current ? " (current step)" : "");
}

function renderStrip(steps) {
  stripSteps = steps || [];
  const strip = $("strip");
  while (strip.children.length > stripSteps.length) strip.removeChild(strip.lastChild);
  while (strip.children.length < stripSteps.length) {
    const b = document.createElement("button");
    const k = strip.children.length;
    b.type = "button";
    b.onclick = (e) => pipClick(e, k);
    // Hover is a mouse's; on a touchscreen the tap is the whole gesture.
    b.onpointerenter = (e) => { if (e.pointerType !== "touch") showTip(k, "hover"); };
    b.onpointerleave = () => { if (tip.source === "hover" && tip.pip === k) hideTip(); };
    b.onfocus = () => { if (b.matches(":focus-visible")) showTip(k, "focus"); };
    b.onblur = () => { if (tip.source === "focus" && tip.pip === k) hideTip(); };
    strip.appendChild(b);
  }
  stripSteps.forEach((s, k) => {
    const b = strip.children[k];
    b.className = "pip " + pipState(s) + (s.current ? " current" : "");
    b.setAttribute("aria-label", pipLabel(s, k));
    if (s.current) b.setAttribute("aria-current", "step");
    else b.removeAttribute("aria-current");
  });
  if (tip.pip >= stripSteps.length) hideTip();
  else if (tip.pip >= 0) showTip(tip.pip);
}

function pipClick(e, k) {
  // A pointer leaves the keyboard where it was, so Enter goes on meaning Next.
  if (e.detail > 0) e.currentTarget.blur();
  const s = stripSteps[k];
  if (!s || s.current) return;
  send("goto", k + 1);
}

function showTip(k, source) {
  const s = stripSteps[k];
  const b = $("strip").children[k];
  if (!s || !b) {
    hideTip();
    return;
  }
  tip.pip = k;
  if (source) tip.source = source;

  let state;
  if (s.manual_qc) {
    state = "! Flagged for manual QC";
  } else if (s.state === "pass" || s.state === "fail") {
    state = (s.state === "pass" ? TICK + " Passed" : CROSS + " Failed");
    if (s.measured_mm !== undefined) {
      state += DOT + fmt(s.measured_mm, 1) + " mm";
      if (s.deviation_pct !== undefined) state += DOT + signed(pct(s.deviation_pct));
      else if (s.deviation_mm !== undefined) {
        state += DOT + signed((s.deviation_mm >= 0 ? "+" : "") + fmt(s.deviation_mm, 2)) + " mm";
      }
    } else {
      state += DOT + "not read";
    }
  } else {
    state = "Not measured yet";
  }
  const meta = ["target " + fmt(s.target_mm, 1) + " mm"];
  if (s.state === "fail" && s.reason && !s.manual_qc) meta.push(s.reason);

  $("tipHead").textContent = "Step " + (k + 1) + " of " + stripSteps.length;
  $("tipName").textContent = nice(s.name);
  $("tipState").textContent = state;
  $("tipState").className = "tipState num " + pipState(s);
  $("tipMeta").textContent = meta.join(DOT);
  $("tipHint").textContent = s.current ? "You are here" : "Click to go to this step";
  // Its saved picture, when it has one on this shirt.
  const img = $("tipImg");
  const c = s.capture;
  if (c && c.state === "saved" && lastStatus) {
    const u = captureUrl(lastStatus.garment, k + 1, c);
    if (img.dataset.u !== u) {
      img.dataset.u = u;
      img.src = u;
    }
    img.hidden = false;
  } else {
    img.hidden = true;
    img.dataset.u = "";
  }

  const el = $("tip");
  el.hidden = false;
  const r = b.getBoundingClientRect();
  const w = el.offsetWidth;
  const cx = r.left + r.width / 2;
  const left = Math.max(8, Math.min(cx - w / 2, window.innerWidth - w - 8));
  el.style.left = left + "px";
  el.style.top = (r.bottom + 6) + "px";
}

function hideTip() {
  tip.pip = -1;
  tip.source = "";
  $("tip").hidden = true;
}

// -- the saved picture of the step on screen --
//
// The last verdict of this step on this shirt, as the station saved it: shown
// beside the feed when the step is come back to, retaken or looked over, and
// gone with a new shirt. Asked for by shirt and step; the id in the address is
// the take, so a new take is fetched and an old one is not asked for twice.
// While the station is still writing it the box says so, and the next poll or
// a short retry asks again.
const shot = { url: "", retry: 0, timer: 0 };
function captureUrl(garment, step, c) {
  return "/api/capture?garment=" + garment + "&step=" + step + "&v=" + c.id;
}
function shotTone(kind) { return kind === "PASS" ? "pass" : kind === "MANUAL" ? "warn" : "fail"; }
function shotWord(kind) { return kind === "PASS" ? TICK + " PASS" : kind === "MANUAL" ? "MANUAL QC" : CROSS + " FAIL"; }
function shotWait(text) {
  $("shotImg").hidden = true;
  $("shotWait").hidden = false;
  $("shotWait").textContent = text;
}
function shotLoad() {
  clearTimeout(shot.timer);
  shotWait("Saving...");
  $("shotImg").src = shot.url + (shot.retry ? "&r=" + shot.retry : "");
}
$("shotImg").onload = () => {
  if (!shot.url || $("shotImg").src.indexOf(shot.url) < 0) return;
  $("shotImg").hidden = false;
  $("shotWait").hidden = true;
};
$("shotImg").onerror = () => {
  if (!shot.url) return;
  shotWait("Saving...");
  clearTimeout(shot.timer);
  if (shot.retry < 30) {
    const url = shot.url;
    shot.timer = setTimeout(() => {
      if (shot.url !== url) return;
      shot.retry++;
      shotLoad();
    }, 700);
  } else {
    // Twenty seconds on: not being written, gone (a retention limit), or unreadable.
    shotWait("Picture not available");
  }
};
function shotClose() {
  $("shotBig").hidden = true;
  $("shotBigImg").removeAttribute("src");
}
$("shotOpen").onclick = () => {
  if (!shot.url || $("shotImg").hidden) return;
  $("shotBigImg").src = $("shotImg").src;
  $("shotBigNote").textContent = $("shotNote").dataset.long || "";
  $("shotBig").hidden = false;
  $("shotClose").focus();
};
$("shotClose").onclick = shotClose;
$("shotBigImg").onclick = shotClose;
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape" && !$("shotBig").hidden) shotClose();
});
function renderShot(s) {
  const step = s.step;
  const pip = step && s.steps ? s.steps[step.index - 1] : null;
  const c = pip && pip.capture ? pip.capture : null;
  const url = c ? captureUrl(s.garment, step.index, c) : "";
  // A picture enlarged for another step, take or shirt goes with it.
  if (!$("shotBig").hidden && url !== shot.url) shotClose();
  if (!c) {
    $("shot").hidden = true;
    clearTimeout(shot.timer);
    shot.url = "";
    shotWait("");
    $("shotImg").removeAttribute("src");
    return;
  }
  $("shot").hidden = false;
  $("shotChip").className = "chip " + shotTone(c.kind);
  $("shotChip").textContent = shotWord(c.kind);
  const at = (c.at || "").slice(11, 19);
  $("shotNote").textContent = (s.result ? "This reading" : "The last take") + (at ? ", " + at : "");
  $("shotNote").dataset.long = "Step " + step.index + " of " + step.total + DOT + nice(step.name) +
                               DOT + "Shirt " + s.garment + (at ? DOT + at : "");
  if (c.state === "saving") {
    // Asked for the moment the station says it is written.
    if (shot.url !== url) {
      clearTimeout(shot.timer);
      shot.url = "";
    }
    shotWait("Saving...");
    return;
  }
  if (c.state !== "saved") {
    clearTimeout(shot.timer);
    shot.url = "";
    shotWait(c.state === "dropped" ? "Not saved: the station was too busy writing" : "Could not be saved");
    return;
  }
  if (url !== shot.url) {
    shot.url = url;
    shot.retry = 0;
    shotLoad();
  }
}

// -- the step on screen --
let enteredStep = "";
function renderStep(s) {
  const step = s.step;
  if (!step) {
    if (enteredStep) {
      enteredStep = "";
      boxing = null;
      drawing = null;
      collarAsked = null;
      inkClear();
      stopGlow();
    }
    $("stepOf").textContent = "No style selected";
    $("stepName").textContent = "-";
    $("instruction").textContent = "Choose a style from the list at the top.";
    $("refImg").hidden = true;
    $("refNone").hidden = false;
    $("refNone").textContent = "No style selected";
    $("editFor").textContent = "No style selected.";
    shownStep = null;
    return;
  }

  // Arriving at a step: it slides in, briefly. Nothing waits on it. And
  // whatever this page was drawing over the picture for the step just left -
  // a box still waiting for its answer, a trace in progress, a glow - goes,
  // because nothing from one step may stay on another's feed.
  const here = (s.style || "") + "|" + (s.garment || 0) + ":" + step.index;
  if (here !== enteredStep) {
    boxing = null;
    drawing = null;
    collarAsked = null;
    collarFailedAt = "";
    inkClear();
    stopGlow();
    if (enteredStep && !reduced()) {
      for (const el of [$("stepHead"), $("resultCard")]) {
        el.classList.remove("enter");
        void el.offsetWidth;
        el.classList.add("enter");
      }
    }
    enteredStep = here;
  }

  $("stepOf").textContent = "Step " + step.index + " of " + step.total;
  $("stepName").textContent = nice(step.name);
  $("instruction").textContent = step.instruction || "";
  $("editFor").textContent = "Step " + step.index + ", " + nice(step.name) + ". Saved into the style file.";

  if (shownStep !== step.index) {
    shownStep = step.index;
    if (step.has_reference) {
      $("refImg").src = "/api/reference?step=" + step.index;
      $("refImg").hidden = false;
      $("refNone").hidden = true;
    } else {
      $("refImg").hidden = true;
      $("refNone").hidden = false;
      $("refNone").textContent = "No reference picture for this step";
    }
    editDirty = false;
  }
  $("refImg").onerror = () => {
    $("refImg").hidden = true;
    $("refNone").hidden = false;
    $("refNone").textContent = "Reference picture missing from the templates folder";
  };

  if (!editDirty) {
    $("eTarget").value = fmt(step.target_mm, 1);
    $("eMinus").value  = fmt(step.tol_minus_mm, 1);
    $("ePlus").value   = fmt(step.tol_plus_mm, 1);
  }
}

// -- a number arriving --
//
// A fresh reading counts up to itself, quickly; one already there is simply
// shown. The figure is tabular, so the count does not make the line jitter.
function countUp(el, to, digits, fresh) {
  // A count already running to this number is left to finish: the page polls
  // faster than it counts.
  if (!fresh && el._raf && el._to === to) return;
  if (el._raf) cancelAnimationFrame(el._raf);
  el._raf = 0;
  el._to = to;
  if (!fresh || reduced() || !isFinite(to)) {
    el.textContent = fmt(to, digits);
    return;
  }
  const t0 = performance.now(), dur = 480, from = to * 0.82;
  const tick = (t) => {
    const u = Math.min(1, (t - t0) / dur);
    const e = 1 - Math.pow(1 - u, 3);
    el.textContent = fmt(from + (to - from) * e, digits);
    el._raf = u < 1 ? requestAnimationFrame(tick) : 0;
  };
  el._raf = requestAnimationFrame(tick);
}

// A verdict that changed while the page was looking at the same step, as
// opposed to one that was already there when the page got to it. Only a fresh
// one is animated.
const verdictWatch = { step: "", res: "", freshKey: "" };
function verdictKey(res) {
  return res ? [res.timestamp, res.result, res.measured_mm].join("|") : "";
}
function watchVerdict(s) {
  const step = s.step, res = s.result;
  const stepKey = step ? s.garment + ":" + step.index : "";
  const resKey = verdictKey(res);
  const fresh = !!res && stepKey === verdictWatch.step && resKey !== verdictWatch.res;
  verdictWatch.step = stepKey;
  verdictWatch.res = resKey;
  if (fresh) verdictWatch.freshKey = stepKey + "|" + resKey;
  return fresh;
}

function renderResult(s, fresh) {
  const result = s.result, step = s.step;
  const card = $("resultCard"), flag = $("flag"), reading = $("reading"), band = $("band"), why = $("why");
  const dev = $("dev");

  // The target is a length so it stays in millimetres. What is allowed around
  // it is a share of it, so the band means the same on a shoulder as a chest.
  if (step) {
    band.textContent = step.has_percent
      ? "Target " + fmt(step.target_mm, 1) + " mm" + DOT + "allowed " +
        signed(pct(step.tol_minus_pct)) + " to " + pct(step.tol_plus_pct)
      : "Target " + fmt(step.target_mm, 1) + " mm" + DOT + "allowed " +
        fmt(step.low_mm, 1) + " to " + fmt(step.high_mm, 1) + " mm";
  } else {
    band.textContent = "";
  }

  if (!result) {
    card.className = "result";
    flag.className = "chip idle";
    flag.textContent = step ? "Not measured yet" : "Waiting";
    reading.hidden = true;
    dev.hidden = true;
    why.textContent = "";
    return;
  }

  const passed = result.result === "PASS";
  const flagged = !!result.manual_qc;
  const tone = flagged ? "warn" : passed ? "pass" : "fail";
  card.className = "result " + tone;
  flag.className = "chip " + tone;
  flag.textContent = flagged ? "MANUAL QC" : (passed ? TICK + " PASS" : CROSS + " FAIL");

  if (result.measured) {
    reading.hidden = false;
    countUp($("mm"), result.measured_mm, 1, fresh);
    dev.hidden = false;
    dev.className = "dev num " + (passed ? "pass" : "fail");
    $("devPct").textContent = signed(result.has_percent
      ? pct(result.deviation_pct)
      : (result.deviation_mm >= 0 ? "+" : "") + fmt(result.deviation_mm, 2) + " mm");
    $("devNote").textContent = "off target";
  } else {
    reading.hidden = true;
    dev.hidden = true;
  }
  why.textContent = flagged ? "Measure this one by hand. Retake to try again."
                            : (result.reason ? cap(result.reason) : "");
}

function renderMoving(seconds) {
  const box = $("moving");
  if (seconds === null || seconds === undefined) {
    box.textContent = "";
    return;
  }
  box.textContent = seconds > 0.05
    ? "Moving to the next step in " + seconds.toFixed(1) + " s"
    : "Moving to the next step";
}

// -- what to do now: one line --
function doNow(s) {
  const step = s.step, res = s.result;
  const calibrating = !!(s.calibration && s.calibration.active);
  if (calibrating) return ["Recalibrating: keep the base clear", "warn"];
  if (!step) return ["Choose a style to start", ""];
  if (res) {
    if (res.manual_qc) return ["Flagged: measure this collar by hand", "warn"];
    if (res.result === "PASS") {
      if (step.last) return ["Passed. Start the next shirt", "pass"];
      return [s.advance_in_s !== null && s.advance_in_s !== undefined ? "Passed. Moving on" : "Passed. Go to the next step", "pass"];
    }
    if (step.kind === "collar" && collarStuck) return [collarStuckWhy, "fail"];
    return ["Failed. Retake, or move on", "fail"];
  }
  const held = !!s.held;
  const busy = s.state === "STABILISING" || s.state === "CAPTURING";
  if (step.kind === "collar") {
    if (!held) return [busy ? "Hold still: reading the shirt" : "Lay the shirt flat, face up, collar down", busy ? "quiet" : ""];
    if (collarAsked) return ["Reading the collar", "quiet"];
    if (collarStuck) return [collarStuckWhy, "fail"];
    const c = s.collar || {};
    if (c.state === "choose" && s.manual_steps) return ["Tap the collar line on the picture", ""];
    return ["Drag a box around the whole collar", ""];
  }
  if (step.kind === "perimeter") {
    const p = s.perimeter || {};
    // The station's full words are in the strip below; the line says only
    // that this half has to be laid again.
    if (p.bad && p.message) return [p.phase === "second" ? "Lay the second half again" : "Lay the first half again", "warn"];
    return [p.phase === "second" ? "Lay the second half, clips in the band" : "Lay the first half, clips in the band", ""];
  }
  if (step.kind === "curve") return [held ? "Point at the seam" : "Lay the garment flat", ""];
  if (step.kind === "distance") return [held ? "Tap the two points" : "Lay the garment flat", ""];
  if (busy) return ["Hold still: reading the fold", "quiet"];
  return ["Lay the fold against the rail", ""];
}

// -- a pointing step --
//
// A curve or distance step is the only place on this bench where the operator
// is part of the instrument, so it is the only place that needs telling what
// to do next and how far through it they are.
function renderPointing(s) {
  const box = $("pointing");
  const step = s.step;
  const isCurve = !!(step && step.kind === "curve");
  const isDistance = !!(step && step.kind === "distance");
  const held = !!s.held;

  pointable = (isCurve || isDistance) && held;
  $("feed").classList.toggle("pointable", pointable);

  if (!isCurve && !isDistance) {
    box.hidden = true;
    return;
  }
  box.hidden = false;

  const want = step.seeds_wanted || 3;
  const have = s.seeds || 0;
  // A traced seam is many points, not "25 of 3".
  $("pointCount").textContent = s.tracing ? "drawing" : have > want ? have + " traced" : (have + " of " + want);

  let hint;
  if (!held) {
    hint = "Lay the garment flat and wait for the reading to hold.";
  } else if (isDistance) {
    hint = have === 0
      ? "Tap one end, then the other - or drag from one to the other. The reading is "
        + "the straight line between the two points, exactly where you put them."
      : "Now tap the other end.";
  } else if (s.tracing) {
    hint = "Drawing the seam - " + (s.traced || 0) +
           " points so far. Keep going to the far end, then take your hand away.";
  } else if (have === 0) {
    hint = "Draw along the seam: drag a finger across the picture, or hold a fingertip " +
           "on the seam until the ring closes, trace it, and stop at the far end. " +
           "Tapping single points still works.";
  } else if (have < want) {
    hint = "Point at " + (want - have) + " more.";
  } else {
    // A full set stays full: seeing this means the last read was refused.
    hint = "All " + want + " points are down. Undo one, or Clear and point again.";
  }
  if (held && isCurve) {
    hint += step.measure === "chord"
      ? " This step measures straight across the seam."
      : " This step measures along the seam.";
  }
  if (s.pointer && s.pointer_dwell > 0) {
    hint = s.tracing
      ? "Holding still - about to finish. Move to carry on."
      : "Hold still... " + Math.round(s.pointer_dwell * 100) + "%";
  }
  $("pointHint").textContent = hint;

  $("btnMarkUndo").disabled  = !held || have === 0;
  $("btnMarkClear").disabled = !held || have === 0;
  $("btnMarkGo").disabled    = !held || have < 2;
}

// -- a collar step --
//
// Read from a box dragged round the collar. The box says where the collar is
// and nothing more: its edges, the shoulder points and the number all come
// from the picture. When the box reads nothing - no band at all, as on a polo,
// or a box that holds no collar - the operator is given the two ways on:
// redraw the box, or hand the shirt on to be measured by hand. No number off a
// line the machine could not vouch for is ever shown as a reading.
let collarStuck = false;
let collarStuckWhy = "";
// Flag stays on offer for the rest of a take in which the box has failed, so
// Redraw - which clears the failed box - does not take the other way on away.
let collarFailedAt = "";
let flagOffered = false;
// Retake starts the step over: a box still waiting for its answer, and the
// failure that offered the flag, belong to the take being thrown away.
function forgetBox() {
  boxing = null;
  collarAsked = null;
  collarFailedAt = "";
  inkClear();
}
let collarStanding = false;  // the collar step has a verdict with a number, or a flag

function renderCollar(s) {
  const box = $("collarStrip");
  const step = s.step;
  const isCollar = !!(step && step.kind === "collar");
  const held = !!s.held;
  boxable = isCollar && held;
  $("feed").classList.toggle("boxable", boxable);
  collarStuck = false;
  collarStuckWhy = "";
  flagOffered = false;
  collarStanding = false;
  if (!isCollar) {
    box.hidden = true;
    return;
  }
  box.hidden = false;
  // A verdict with a number already stands on the step (a Back to a step
  // that passed): a box refused now is about that box, not the step.
  const res = s.result;
  const standing = !!(res && (res.measured || res.manual_qc));
  collarStanding = standing;

  const c = s.collar || {};
  const answered = !collarAsked || (c.seq || 0) > collarAsked.seq || Date.now() - collarAsked.at > 15000;
  if (answered && collarAsked) collarAsked = null;
  const around = step.measure === "around";
  const edge = around ? "the whole collar"
             : step.seam === "inner" ? "the band's inner edge, the opening"
                                     : "the band's outer edge, the attachment seam";
  const length = around ? "along the band's outer edge over the front and the edge of the shirt across the back, " +
                          "shoulder point to shoulder point both ways"
               : step.measure === "chord" ? "straight across, shoulder point to shoulder point"
                                          : "along it, shoulder point to shoulder point";
  const flagged = !!(s.result && s.result.manual_qc);
  let hint, bad = false;
  $("collarHead").textContent = around ? "The whole collar" : "Collar";
  if (!held) {
    hint = "Lay the shirt flat and face up with the collar flattened down, and wait for the reading to hold.";
  } else if (!answered) {
    hint = "Reading the collar...";
  } else if (c.state === "read" && around && c.back_mm !== undefined) {
    hint = fmt(c.mm, 1) + " mm - the whole collar: " + fmt(c.front_mm, 1) + " mm over the front and " +
           fmt(c.back_mm, 1) + " mm across the back. The dotted line is the opening, shown and not measured. " +
           cap(c.why) + ".";
  } else if (c.state === "read") {
    hint = fmt(c.mm, 1) + " mm - " + edge + ", " + length + ". " + cap(c.why) + ".";
  } else if (flagged) {
    hint = "Flagged for manual QC: measure this collar by hand. Retake to try the box again.";
    bad = "warn";
  } else if (c.state === "choose" && s.manual_steps) {
    hint = cap(c.why) + ".";
  } else if (c.state === "choose") {
    // No band: the machine found lines, and cannot say which one is the
    // collar. A polo's folded leaf lands here every time.
    collarStuck = !standing;
    collarStuckWhy = "No collar band found";
    hint = "Polo and shirt-style collars can't be read automatically yet. " +
           (standing ? "The reading already on this step stands." :
                       "Redraw the box around the collar, or flag the shirt for manual QC.");
    bad = true;
  } else if (c.state === "refused") {
    collarStuck = !standing;
    collarStuckWhy = "The collar was not read";
    hint = cap(c.why) + ". " + (standing ? "The reading already on this step stands." :
                                           "Redraw the box around the collar, or flag the shirt for manual QC.");
    bad = true;
  } else {
    hint = "Drag a box round the whole collar - past both shoulders, with a little of the mat " +
           "beyond the edge of the shirt inside it. This step measures " + edge + ", " + length +
           ". Where you draw the box does not change the number.";
  }
  $("collarHint").textContent = hint;
  $("collarHint").classList.toggle("bad", bad === true);
  $("collarHint").classList.toggle("warn", bad === "warn");
  const take = enteredStep;
  // A new lay is a new try: the box failing on the last one is not a reason
  // to offer the flag on this one before a box has been drawn.
  if (!held) collarFailedAt = "";
  if (collarStuck) collarFailedAt = take;
  flagOffered = held && !standing && (collarStuck || collarFailedAt === take);

  const list = $("collarLines");
  list.innerHTML = "";
  const row = (label, mm, chosen) => {
    const r = document.createElement("div");
    if (chosen) r.className = "chosen";
    const what = document.createElement("span");
    what.textContent = label;
    const value = document.createElement("span");
    value.textContent = fmt(mm, 1) + " mm";
    r.appendChild(what);
    r.appendChild(value);
    list.appendChild(r);
  };
  // Only a collar that was read has numbers worth showing. The lines of one
  // refused, or with no band behind them, are not readings - unless the
  // hand-drawn fallback is on and one of them is to be tapped.
  if (answered && (c.state === "read" || (c.state === "choose" && s.manual_steps))) {
    for (const l of c.lines || []) {
      row((l.role === "outer" ? "outer edge" : l.role === "inner" ? "inner edge (dotted)" : "a line") +
          (l.chosen ? (around ? ", measured over the front" : ", measured") : ""),
          step.measure === "chord" ? l.chord_mm : l.arc_mm, !!l.chosen);
    }
    if (around && c.state === "read" && c.back_mm !== undefined) {
      row("back of the neck, measured", c.back_mm, true);
      row("the whole collar", c.mm, true);
    }
  }
  list.hidden = !list.children.length;
  $("btnCollarClear").disabled = !held || !c.box;
  // While the big button says Redraw, it is this button.
  $("btnCollarClear").parentElement.hidden = collarStuck;
  // The station now draws the box it was sent; the one dragged comes off.
  if (answered && !boxing) inkClear();
}

// -- a perimeter step --
//
// Read in two halves, joined at two clips. The strip says which half is
// wanted, what the guide on the feed makes of the lay, and what each half
// read. Nothing is typed or tapped.
function renderPerimeter(s) {
  const box = $("perimStrip");
  const step = s.step;
  if (!(step && step.kind === "perimeter")) {
    box.hidden = true;
    return;
  }
  box.hidden = false;
  const p = s.perimeter || {};
  const phase = p.phase || "first";
  $("perimHead").textContent = phase === "done" ? "Both halves read"
                             : phase === "second" ? "Second half"
                             : "First half";
  $("perimHint").textContent = p.message || "";
  $("perimHint").classList.toggle("bad", !!p.bad);

  const list = $("perimHalves");
  list.innerHTML = "";
  const row = (label, mm, chosen) => {
    const r = document.createElement("div");
    if (chosen) r.className = "chosen";
    const what = document.createElement("span");
    what.textContent = label;
    const value = document.createElement("span");
    value.textContent = fmt(mm, 1) + " mm";
    r.appendChild(what);
    r.appendChild(value);
    list.appendChild(r);
  };
  const halves = p.halves || [];
  for (let k = 0; k < halves.length; k++) row(k === 0 ? "first half" : "second half", halves[k].mm, false);
  if (phase === "done" && p.total_mm !== undefined) {
    row(p.whole ? "total, from one picture" : "total", p.total_mm, true);
  }
  list.hidden = !list.children.length;
}

// -- a step's marks, kept --
//
// The station clears the feed on every move between steps, and has to. But the
// outline and the line a step was read from are worth seeing again when the
// operator goes back to it. So the station sends its marks beside each
// verdict, this page keeps them in the browser's own storage against that
// verdict, and draws them back over the feed only when that step is shown
// again. They go as soon as anything else is: another step, a retake, a new
// reading, a point or a box put down. Only the shirt on the mat is kept.
const MARKS_KEY = "verimeasure.marks";
const kept = { step: "", live: false, drawn: "" };

function keptLoad() {
  try { return JSON.parse(localStorage.getItem(MARKS_KEY) || "null"); }
  catch (e) { return null; }
}

function keptSave(store) {
  try { localStorage.setItem(MARKS_KEY, JSON.stringify(store)); }
  catch (e) { /* no storage here: nothing is kept, and nothing breaks */ }
}

function renderMarks(s) {
  const canvas = $("marks"), img = $("video");
  const step = s.step, res = s.result;
  const shirt = (s.style || "") + "|" + (s.garment || 0);
  const here = step ? shirt + "|" + step.index : "";
  const verdict = res ? [res.timestamp, res.result, res.measured_mm].join("|") : "";
  if (here !== kept.step) {
    kept.step = here;
    kept.live = false;
  }

  let store = keptLoad();
  if (s.marks && step && verdict) {
    // The station is drawing this verdict's marks itself. Kept the first time
    // they are seen: what is kept is what the reading was taken from.
    kept.live = true;
    if (!store || store.shirt !== shirt || !store.steps) store = { shirt: shirt, steps: {} };
    const had = store.steps[step.index];
    if (!had || had.verdict !== verdict) {
      store.steps[step.index] = { verdict: verdict, marks: s.marks };
      keptSave(store);
    }
  }
  if (s.seeds > 0 || s.tracing || (s.collar && s.collar.box) || drawing || boxing) kept.live = true;

  const entry = store && store.shirt === shirt && store.steps && step ? store.steps[step.index] : null;
  const calibrating = !!(s.calibration && s.calibration.active);
  const r = img.getBoundingClientRect();
  if (!entry || !verdict || entry.verdict !== verdict || kept.live || calibrating ||
      img.hidden || !(r.width > 0 && r.height > 0)) {
    if (!canvas.hidden) {
      canvas.getContext("2d").clearRect(0, 0, canvas.width, canvas.height);
      canvas.hidden = true;
    }
    kept.drawn = "";
    return;
  }
  const at = [verdict, r.left, r.top, r.width, r.height, window.devicePixelRatio || 1].join("|");
  if (at === kept.drawn) return;
  kept.drawn = at;
  overPicture(canvas, img);
  drawMarks(canvas, entry.marks);
}

// The marks as the station drew them: its colours, and its weights, scaled
// from the picture it sent to the size shown here.
function drawMarks(canvas, m) {
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const k = canvas.width / (m.w || canvas.width);
  const shadow = "#141414";
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  for (const it of m.items || []) {
    const p = it.p || [];
    const pts = [];
    for (let i = 0; i + 1 < p.length; i += 2) pts.push([p[i] * canvas.width, p[i + 1] * canvas.height]);
    if (!pts.length) continue;
    if (it.t) {
      const centred = it.a === "center";
      ctx.font = "600 " + Math.max(10, Math.round(20 * k)) + "px " + getComputedStyle(document.body).fontFamily;
      ctx.textAlign = centred ? "center" : "left";
      ctx.textBaseline = centred ? "middle" : "alphabetic";
      ctx.lineWidth = Math.max(1, 4 * k);
      ctx.strokeStyle = shadow;
      ctx.strokeText(it.t, pts[0][0], pts[0][1]);
      ctx.fillStyle = it.c;
      ctx.fillText(it.t, pts[0][0], pts[0][1]);
      continue;
    }
    const trace = () => {
      ctx.beginPath();
      pts.forEach((q, i) => (i ? ctx.lineTo : ctx.moveTo).call(ctx, q[0], q[1]));
      if (it.closed) ctx.closePath();
      ctx.stroke();
    };
    ctx.setLineDash(it.dots ? [0.001, it.dots * k] : []);
    if (it.h) {
      ctx.strokeStyle = shadow;
      ctx.lineWidth = Math.max(1, it.h * k);
      trace();
    }
    ctx.strokeStyle = it.c;
    ctx.lineWidth = Math.max(1, it.w * k);
    trace();
    ctx.setLineDash([]);
    if (!it.d) continue;
    for (const q of [pts[0], pts[pts.length - 1]]) {
      for (const [radius, colour] of [[it.dh, shadow], [it.d, it.c]]) {
        if (!radius) continue;
        ctx.fillStyle = colour;
        ctx.beginPath();
        ctx.arc(q[0], q[1], radius * k, 0, 2 * Math.PI);
        ctx.fill();
      }
    }
  }
}

// -- an outline arriving --
//
// When a reading lands, what it was taken from - the outline and the line
// measured along it - draws itself on in a glowing stroke of the accent and
// fades, leaving the station's own drawing. Only for a fresh verdict on the
// step showing, never for one carried from another step, and it stops the
// moment the step changes. It is only ever drawn over; nothing waits for it.
const glowState = { key: "", raf: 0 };
function stopGlow() {
  if (glowState.raf) cancelAnimationFrame(glowState.raf);
  glowState.raf = 0;
  const c = $("glow");
  if (!c.hidden) {
    c.getContext("2d").clearRect(0, 0, c.width, c.height);
    c.hidden = true;
  }
}
function renderGlow(s, fresh) {
  const step = s.step, res = s.result;
  const key = step && res ? s.garment + ":" + step.index + "|" + verdictKey(res) : "";
  if (glowState.key && key !== glowState.key) { stopGlow(); glowState.key = ""; }
  // Only an outline something was measured from; a flag or an unread step
  // has nothing to show arriving.
  if (!fresh || !s.marks || !key || reduced() || !res.measured || res.manual_qc) return;
  const img = $("video");
  const r = img.getBoundingClientRect();
  if (img.hidden || !(r.width > 0 && r.height > 0)) return;
  const lines = [];
  for (const it of s.marks.items || []) {
    if (it.t || !it.p || it.p.length < 4 || it.dots) continue;
    lines.push(it);
  }
  if (!lines.length) return;
  stopGlow();
  glowState.key = key;
  const c = $("glow");
  overPicture(c, img);
  const ctx = c.getContext("2d");
  const dpr = window.devicePixelRatio || 1;
  const paths = lines.map((it) => {
    const pts = [];
    for (let i = 0; i + 1 < it.p.length; i += 2) pts.push([it.p[i] * c.width, it.p[i + 1] * c.height]);
    if (it.closed) pts.push(pts[0]);
    let len = 0;
    for (let i = 1; i < pts.length; i++) len += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
    return { pts, len };
  });
  const col = accent();
  const t0 = performance.now(), DRAW = 520, HOLD = 260, FADE = 620;
  const frame = (t) => {
    const e = t - t0;
    const u = Math.min(1, e / DRAW);
    const drawn = 1 - Math.pow(1 - u, 3);
    const alpha = e < DRAW + HOLD ? 1 : Math.max(0, 1 - (e - DRAW - HOLD) / FADE);
    ctx.clearRect(0, 0, c.width, c.height);
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    for (const p of paths) {
      ctx.setLineDash([p.len * drawn, p.len + 1]);
      ctx.beginPath();
      p.pts.forEach((q, i) => (i ? ctx.lineTo : ctx.moveTo).call(ctx, q[0], q[1]));
      ctx.globalAlpha = alpha * 0.55;
      ctx.shadowColor = col;
      ctx.shadowBlur = 18 * dpr;
      ctx.strokeStyle = col;
      ctx.lineWidth = 5 * dpr;
      ctx.stroke();
      ctx.globalAlpha = alpha;
      ctx.shadowBlur = 6 * dpr;
      ctx.lineWidth = 2.2 * dpr;
      ctx.strokeStyle = "#E8FAFF";
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    ctx.shadowBlur = 0;
    ctx.setLineDash([]);
    if (e < DRAW + HOLD + FADE) glowState.raf = requestAnimationFrame(frame);
    else { glowState.raf = 0; c.hidden = true; }
  };
  glowState.raf = requestAnimationFrame(frame);
}

// -- the station log --
function renderLog(s) {
  const entries = s.log || [];
  const box = $("log");
  box.innerHTML = "";
  for (const e of entries) {
    const d = document.createElement("div");
    const b = document.createElement("b");
    b.textContent = e.time;
    d.appendChild(b);
    d.appendChild(document.createTextNode(e.text));
    box.appendChild(d);
  }
  // The log is the engineer's, in the drawer. The one line of it the
  // operator needs is why a lay that settled was not read, which otherwise
  // looks like the station doing nothing: shown under the instruction while
  // the step has no verdict.
  const last = entries[0];
  const why = last && !s.result && /^Not measured/.test(last.text) ? last.text : "";
  $("said").textContent = why;
  $("said").hidden = !why;
}

// -- recalibrating --
//
// The only control on this page that takes seconds rather than a frame. The
// card says which part of the job the station is on and how far it has got,
// then how it ended and why.
const calib = {
  asked: 0,       // when Recalibrate was pressed, until the station takes it up
  askedSeq: 0,    // the last run the station had numbered at that moment
  watch: 0,       // the run this page is following
  shownAt: 0,     // when that run's outcome went up
  closed: 0,      // the run whose outcome has been put away
  lastDone: 0,    // how far the wait for a still mat had got at the last poll
  movedAt: 0,     // when that went backwards: something moved on the mat
  refused: null   // {text, at}: the station would not start one
};

// Asks the station for a recalibration - plain, or with a new exposure level -
// and follows it on the card over the picture. True when it was taken.
async function askCalibration(url) {
  const c = lastStatus && lastStatus.calibration;
  calib.asked = Date.now();
  calib.askedSeq = c ? (c.seq || 0) : 0;
  calib.closed = calib.watch;
  calib.refused = null;
  renderCalibration(lastStatus);
  let reply;
  try {
    reply = await (await fetch(url, { method: "POST" })).json();
  } catch (e) {
    reply = { ok: false, error: "the station did not answer" };
  }
  if (!reply.ok && !reply.busy) {
    calib.asked = 0;
    calib.refused = { text: reply.error || "the station would not start it", at: Date.now() };
    renderCalibration(lastStatus);
  }
  return !!reply.ok;
}
$("btnCal").onclick = () => {
  if ($("btnCal").disabled) return;
  askCalibration("/api/command?name=calibrate");
};

// -- the camera's exposure --
//
// The camera is exposed from the empty base, metered to a level; a pale shirt
// has only the room above that level before it clips white, and clipped white
// glows into the base and reads long. Darker and Brighter choose a new level;
// setting it is a recalibration with the base clear, which meters the camera
// to it and retakes the base at the new exposure.
let expoWanted = null;
function stepExposure(by) {
  const k = lastStatus && lastStatus.exposure;
  if (!k) return;
  const have = expoWanted !== null ? expoWanted : Math.round(k.level / 10) * 10;
  expoWanted = Math.min(220, Math.max(60, have + by));
  renderExposure(lastStatus);
}
$("expoDarker").onclick = () => stepExposure(-10);
$("expoBrighter").onclick = () => stepExposure(10);
$("expoApply").onclick = async () => {
  if ($("expoApply").disabled || expoWanted === null) return;
  const level = expoWanted;
  if (await askCalibration("/api/command?name=exposure_level&arg=" + level)) expoWanted = null;
  renderExposure(lastStatus);
};

function renderExposure(s) {
  const k = s && s.exposure;
  if (!k) return;
  const now = Math.round(k.level);
  $("expoNow").textContent = k.text || "-";
  $("expoLevelNow").textContent = String(now);
  const f = s.last && s.last.saturated_fraction;
  const clip = f === undefined || f === null ? null : f * 100;
  $("expoClip").textContent = clip === null ? "-" : clip < 0.05 ? "0%" : clip.toFixed(clip < 10 ? 1 : 0) + "%";
  $("expoClipFact").className = "fact" + (clip === null ? "" : clip > 0.5 ? " no" : " ok");
  const want = expoWanted !== null ? expoWanted : now;
  $("expoLevel").textContent = String(want);
  const changed = expoWanted !== null && expoWanted !== now;
  const calibrating = !!(s.calibration && s.calibration.active);
  $("expoApply").disabled = !changed || calibrating || !s.ready || !!k.pending;
  $("expoApply").textContent = changed ? "Set the camera to " + want + " (base must be clear)" : "Set the camera";
  let note = "Lower is a darker picture: more room before a white shirt clips and glows into the base. " +
             "Higher is brighter, for very dark cloth. Setting it recalibrates: take everything off the base first.";
  if (clip !== null && clip > 0.5) note = "The last shirt was " + $("expoClip").textContent +
    " clipped white, which glows into the base and reads long: go darker. " + note;
  if (k.by_hand) note += " The exposure was set by hand; setting a level hands it back to the station.";
  $("expoNote").textContent = note;
}

$("calibOk").onclick = () => {
  calib.closed = calib.watch;
  calib.refused = null;
  $("calib").hidden = true;
};

// Only while the station waits for the base to be cleared: nothing has been
// touched yet, so there is nothing to undo.
$("calibCancel").onclick = async () => {
  $("calibCancel").disabled = true;
  try {
    await fetch("/api/command?name=calibrate_cancel", { method: "POST" });
  } catch (e) {
  }
};

function calibMode(kind) {
  $("calibBox").className = "card " + kind;
  $("calibSpin").hidden = kind !== "busy";
  $("calibGlyph").hidden = kind === "busy";
  $("calibGlyph").textContent = kind === "ok" ? TICK : "!";
  $("calibMeter").hidden = kind !== "busy";
  $("calibActions").hidden = kind === "busy";
  $("calibCancel").hidden = true;
  $("calibOk").hidden = false;
}

// A fraction fills the bar; null sweeps it, for the part with no count to show.
function calibMeter(fraction) {
  $("calibMeter").classList.toggle("busy", fraction === null);
  $("calibFill").style.width = fraction === null ? "" : Math.round(fraction * 100) + "%";
}

function renderCalibration(s) {
  const c = (s && s.calibration) || { active: false, seq: 0, last: null };
  const ready = !!(s && s.ready);
  const busy = !!c.active;
  const last = c.last || null;
  const now = Date.now();

  if (busy) {
    if (calib.watch !== c.seq) {
      calib.watch = c.seq;
      calib.lastDone = 0;
      calib.movedAt = 0;
      calib.shownAt = 0;
    }
    calib.asked = 0;
  } else if (calib.asked && last && last.seq > calib.askedSeq) {
    calib.watch = last.seq;
    calib.shownAt = 0;
    calib.asked = 0;
  } else if (calib.asked && now - calib.asked > 10000) {
    calib.asked = 0;
  }
  const starting = calib.asked > 0;

  $("btnCal").disabled = !ready || busy || starting || scale.open;
  $("btnCal").textContent = (busy || starting) ? "Recalibrating..." : "Recalibrate";

  const box = $("calib");
  if (busy || starting) {
    box.hidden = false;
    calibMode("busy");
    $("calibTitle").textContent = "Recalibrating";
    $("calibTime").textContent = busy ? Math.floor(c.elapsed_s || 0) + " s" : "";
    $("calibDetail").textContent = "";
    const phase = busy ? c.phase : "queued";
    if (phase === "clear") {
      // Waiting on the operator, so nothing on the card moves: it says what
      // is in the way and starts by itself once the base is clear.
      calibMode("wait");
      $("calibActions").hidden = false;
      $("calibOk").hidden = true;
      $("calibCancel").hidden = false;
      $("calibTitle").textContent = "Recalibrating";
      $("calibStep").textContent = "Lift everything off the base";
      $("calibCount").textContent = Math.max(1, Math.round((c.hidden || 0) * 100)) +
        "% of the printed grid is hidden";
      $("calibNote").textContent = "Recalibrating starts by itself as soon as the base is clear.";
    } else if (phase === "exposure") {
      $("calibStep").textContent = "Setting the camera's exposure for the light on the mat";
      $("calibCount").textContent = "";
      calibMeter(null);
      $("calibNote").textContent = "Keep the mat clear and your hands out of the picture.";
    } else if (phase === "settle" && c.needed > 0) {
      const need = c.needed;
      const done = Math.min(c.done || 0, need);
      if (done < calib.lastDone) calib.movedAt = now;
      calib.lastDone = done;
      $("calibStep").textContent = now - calib.movedAt < 2500
        ? "Something moved - waiting for the mat to settle again"
        : "Waiting for the mat to hold still";
      $("calibCount").textContent = done + " of " + need + " still frames";
      calibMeter(done / need);
      $("calibNote").textContent = "Keep the mat clear and your hands out of the picture.";
    } else if (phase === "fit") {
      $("calibStep").textContent = "Reading the printed grid and fitting the model";
      $("calibCount").textContent = "";
      calibMeter(null);
      $("calibNote").textContent = "The background is taken. A few seconds more.";
    } else {
      $("calibStep").textContent = "Starting";
      $("calibCount").textContent = "";
      calibMeter(null);
      $("calibNote").textContent = "Clear the mat and keep your hands out of the picture.";
    }
    return;
  }

  if (calib.refused && now - calib.refused.at < 6000) {
    const t = calib.refused.text;
    box.hidden = false;
    calibMode("fail");
    $("calibTitle").textContent = "Could not recalibrate";
    $("calibTime").textContent = "";
    $("calibStep").textContent = t.charAt(0).toUpperCase() + t.slice(1) + ".";
    $("calibCount").textContent = "";
    $("calibNote").textContent = "";
    $("calibDetail").textContent = "";
    return;
  }
  calib.refused = null;

  if (last && calib.watch && last.seq === calib.watch && calib.closed !== last.seq) {
    if (!calib.shownAt) calib.shownAt = now;
    // A success says so and gets out of the way. A failure stays until it is
    // read, and so does a success that came with something to do.
    const age = now - calib.shownAt;
    if (last.cancelled || (last.ok && !last.warn && age > 6000) || age > 60000) {
      calib.closed = last.seq;
      box.hidden = true;
      $("calibCancel").disabled = false;
      return;
    }
    box.hidden = false;
    $("calibCancel").disabled = false;
    calibMode(last.ok ? (last.warn ? "warn" : "ok") : "fail");
    $("calibTitle").textContent = last.headline || (last.ok ? "Recalibrated" : "Recalibration failed");
    $("calibTime").textContent = "took " + fmt(last.seconds, 1) + " s";
    $("calibCount").textContent = "";
    if (last.ok) {
      $("calibStep").textContent = last.detail || "";
      $("calibNote").textContent = "The background was retaken from the same frames. Ready to measure.";
      $("calibDetail").textContent = "";
    } else {
      $("calibStep").textContent = "Clear everything off the mat, then press Recalibrate again.";
      $("calibNote").textContent = "The previous calibration is still in use.";
      $("calibDetail").textContent = last.detail ? "Reason: " + last.detail : "";
    }
    return;
  }
  box.hidden = true;
}

// -- checking the scale --
//
// A steel rule of known length laid on the base, read, and the calibration's
// scale corrected to agree with it. While this card is open the station
// judges nothing - the rule is not a garment - and it reads the rule on the
// base's plane, so its thickness is allowed for here.
const SCALE_KEY = "verimeasure.scaleRef";
const scale = {
  open: false,
  asked: 0,     // when the card was opened, until the station has it open too
  appliedAt: 0  // Apply pressed, so a second press does not follow on its heels
};

function loadScaleRef() {
  try {
    const v = JSON.parse(localStorage.getItem(SCALE_KEY) || "null");
    return v && typeof v === "object" ? v : null;
  } catch (e) {
    return null;
  }
}

function saveScaleRef() {
  try {
    localStorage.setItem(SCALE_KEY, JSON.stringify({
      len: $("scaleTrue").value, thick: $("scaleThick").value }));
  } catch (e) {
  }
}

async function scaleSend(name, arg) {
  let q = "/api/command?name=" + encodeURIComponent(name);
  if (arg !== undefined) q += "&arg=" + encodeURIComponent(arg);
  try {
    await fetch(q, { method: "POST" });
  } catch (e) {
  }
}

$("btnScale").onclick = () => {
  if ($("btnScale").disabled) return;
  const saved = loadScaleRef();
  if (saved && saved.len) $("scaleTrue").value = saved.len;
  if (saved && saved.thick !== undefined && saved.thick !== "") $("scaleThick").value = saved.thick;
  if ($("scaleThick").value === "") {
    const c = lastStatus && lastStatus.scale_check;
    $("scaleThick").value = c && c.thickness_mm !== undefined ? c.thickness_mm : 1;
  }
  scale.open = true;
  scale.asked = Date.now();
  scale.appliedAt = 0;
  $("scale").hidden = false;
  scaleSend("scale_check", "start");
  renderScale(lastStatus);
};

$("scaleClose").onclick = () => {
  scale.open = false;
  $("scale").hidden = true;
  scaleSend("scale_check", "stop");
};

$("scaleApply").onclick = () => {
  if ($("scaleApply").disabled) return;
  const len = parseFloat($("scaleTrue").value);
  const thick = parseFloat($("scaleThick").value);
  if (!(len > 0)) return;
  saveScaleRef();
  scale.appliedAt = Date.now();
  $("scaleApply").disabled = true;
  scaleSend("verify", len + "," + (thick >= 0 ? thick : 0));
};

for (const id of ["scaleTrue", "scaleThick"]) {
  $(id).addEventListener("input", () => renderScale(lastStatus));
  $(id).addEventListener("keydown", (e) => {
    if (e.key === "Enter") {
      e.preventDefault();
      $("scaleApply").click();
    }
  });
}

// The reference's top face, which is what the camera sees and where its length
// is known: the station's reading on the base's plane, scaled up to it.
function scaleTopFace(s, planeMm, thick) {
  const d = s && s.working_distance_mm;
  return d && thick > 0 && d > thick ? planeMm * (d - thick) / d : planeMm;
}

function renderScale(s) {
  const c = (s && s.scale_check) || null;
  const calibrating = !!(s && s.calibration && s.calibration.active);
  const verified = !!(s && s.scale_verified);
  $("scaleState").textContent = !(s && s.calibrated) ? "-" : verified ? "checked" : "not checked";
  $("scaleFact").className = "fact" + (s && s.calibrated ? (verified ? " ok" : " no") : "");
  $("btnScale").disabled = !(s && s.ready && s.calibrated) || calibrating || scale.open;
  if (!scale.open) return;

  // Closed at the station - restarted, or closed from another screen.
  if (c && !c.active && Date.now() - scale.asked > 3000) {
    scale.open = false;
    $("scale").hidden = true;
    $("btnScale").disabled = !(s && s.ready && s.calibrated) || calibrating;
    return;
  }

  const t = Math.max(0, parseFloat($("scaleThick").value) || 0);
  const len = parseFloat($("scaleTrue").value);
  const reading = c && c.readings > 0 && c.reading_mm !== null ? scaleTopFace(s, c.reading_mm, t) : null;
  // Read in more than one part of the base: each place's reading, and the
  // correction each asks for. They have to agree for Apply to use them.
  const places = (c && Array.isArray(c.places)) ? c.places : [];
  const several = places.length >= 2;
  const sgn = (f) => (f >= 1 ? "+" : "") + ((f - 1) * 100).toFixed(2) + "%";
  // Nothing turns on this card: it is waiting on the operator and the rule.
  if (reading === null) {
    $("scaleRead").className = "scaleRead num waiting";
    $("scaleRead").textContent = "Waiting for the reference to be laid down and read";
  } else if (several) {
    $("scaleRead").className = "scaleRead num";
    $("scaleRead").textContent = places.map((p) => p.where + " " +
      scaleTopFace(s, p.reading_mm, t).toFixed(2)).join(DOT) + " mm";
  } else {
    $("scaleRead").className = "scaleRead num";
    $("scaleRead").textContent = "Reads " + reading.toFixed(2) + " mm";
  }

  let note = "";
  let big = false;
  if (reading !== null && len > 0 && several) {
    const f = places.map((p) => len / scaleTopFace(s, p.reading_mm, t));
    const lo = Math.min.apply(null, f);
    const hi = Math.max.apply(null, f);
    const each = places.map((p, k) => p.where + " " + sgn(f[k])).join(", ");
    const agree = c && c.agree_pct !== undefined ? c.agree_pct : 0.3;
    if ((hi / lo - 1) * 100 > agree) {
      note = each + ": " + ((hi / lo - 1) * 100).toFixed(2) + "% apart. A scale error is the same " +
             "everywhere, so this is the lens or the corners, and Apply will not use them.";
      big = true;
    } else {
      const mean = f.reduce((a, b) => a + b, 0) / f.length;
      note = each + ". They agree: Apply corrects every reading by " + sgn(mean) + ".";
      if (Math.abs(mean - 1) > 0.02) {
        note += " That is a large correction: check the length typed, and that only the rule was read.";
        big = true;
      }
    }
  } else if (reading !== null && len > 0) {
    const p = (len / reading - 1) * 100;
    note = "Apply corrects every reading by " + (p >= 0 ? "+" : "") + p.toFixed(2) + "%.";
    // Past 2% it is more likely a length typed wrong than a scale that far out.
    if (Math.abs(p) > 2) {
      note += " That is a large correction: check the length typed, and that only the rule was read.";
      big = true;
    }
  } else if (reading !== null) {
    note = "Type its true length, end to end, and its thickness.";
  } else {
    note = "A steel rule is best: straight, rigid, and thin. Lay it in the middle of the base, " +
           "where the garments go, then toward the left and the right: each place is read, and " +
           "they must agree. Nothing is judged while this is open.";
  }
  $("scaleNote").textContent = note;
  $("scaleNote").className = "cardNote" + (big ? " big" : "");
  $("scaleApply").disabled = !(reading !== null && len > 0) || calibrating ||
    Date.now() - scale.appliedAt < 1500;

  const o = c && c.outcome;
  $("scaleOutcome").textContent = o ? o.text : "";
  $("scaleOutcome").className = "cardDetail" + (o ? (o.ok ? " ok" : " bad") : "");
}

// -- a step finishing --
//
// A verdict lands as a band of colour across the top of the picture: green,
// red, or amber for a step handed on to be measured by hand. A pass that the
// station is about to move on from turns into the countdown to the next step,
// with the button to go now. The step arrived at is said for a moment. A
// finished shirt is a card with the button for the next one.
//
// Nothing here decides anything. The station has already judged the step and
// already armed the move; this only draws the wait it was always going to
// take. The move is cancelled by Back, Retake, a new shirt or another style.
const FLOW_ARRIVED_MS = 1800;  // how long the new step is announced
const FLOW_SHIRT_MS = 6000;    // how long a finished shirt is announced
const flow = {
  watchStep: "",  // the step the page showed at the last poll, garment:index
  watchRes: "",   // and the verdict it had then
  key: "",        // the completion being shown, so each is only shown once
  mode: "",       // "", "done", "moving", "arrived" or "shirt"
  from: 0,        // the step that completed, counted from one
  garment: 0,
  total: 0,       // the station's wait before it moves on, seconds; 0 for none
  start: 0,       // when the pass landed, by this page's clock
  shown: 0,       // when a mode that times out began
  info: {}
};
const bannerState = { key: "", mini: false, shown: "", landed: 0 };

// How long the pass is shown before the countdown: a second or so, and never
// more than two fifths of the wait.
function flowCompleteMs() {
  return flow.total > 0 ? Math.min(1500, Math.max(700, flow.total * 400)) : 1500;
}

// Next, from the panel, the keyboard or the banner. While a pass is counting
// down to the next step it takes that move now rather than asking for another:
// a Next that reached the station just after the countdown ran out would move
// a second step and skip one.
function nextStep() {
  if ((flow.mode === "done" || flow.mode === "moving") && flow.total > 0) {
    send("advance_now", flow.from);
  } else {
    send("next");
  }
}

$("bStart").onclick = act(() => nextStep());
$("shirtGo").onclick = act(() => send("next_shirt"));
// A finished shirt can be put away by tapping around it, to see the feed.
$("flow").onclick = (e) => {
  if (e.target === $("flow") && flow.mode === "shirt") {
    flow.mode = "";
    drawFlow(lastStatus, Date.now());
  }
};

function renderFlow(s) {
  const now = Date.now();
  const step = s.step;
  const res = s.result;
  const stepKey = step ? s.garment + ":" + step.index : "";
  const resKey = res ? [res.timestamp, res.result, res.measured_mm].join("|") : "";
  const pass = !!(step && res && res.result === "PASS" && res.step_index === step.index);
  const fresh = pass && stepKey === flow.watchStep && resKey !== flow.watchRes;
  flow.watchStep = stepKey;
  flow.watchRes = resKey;
  const pending = s.advance_in_s !== null && s.advance_in_s !== undefined;

  if (pass && (fresh || pending) && stepKey + "|" + resKey !== flow.key) {
    const next = (s.steps || [])[step.index];  // counted from zero: the one after
    flow.key = stepKey + "|" + resKey;
    flow.mode = "done";
    flow.from = step.index;
    flow.garment = s.garment;
    flow.total = pending ? Number(s.advance_total_s || s.advance_in_s) || 0 : 0;
    flow.start = pending ? now - Math.max(0, flow.total - s.advance_in_s) * 1000 : now;
    flow.info = {
      count: step.total,
      last: !!step.last,
      name: nice(step.name),
      reading: res.measured ? fmt(res.measured_mm, 1) + " mm" : "",
      next: next ? nice(next.name) : ""
    };
  }

  if (flow.mode === "done" || flow.mode === "moving") {
    const here = s.garment === flow.garment && !!step && step.index === flow.from;
    const onward = s.garment === flow.garment && !!step && step.index === flow.from + 1;
    if (flow.total > 0 && onward) {
      // The station has moved on - by itself, or because Next was pressed.
      flow.mode = "arrived";
      flow.shown = now;
      flow.info.now = nice(step.name);
      flow.info.kind = step.kind;
    } else if (!here || !pass || (flow.total > 0 && !pending)) {
      flow.mode = "";  // the move was cancelled
    } else {
      if (pending) flow.start = Math.min(flow.start, now - (flow.total - s.advance_in_s) * 1000);
      if (now - flow.start >= flowCompleteMs()) {
        if (flow.total > 0) {
          flow.mode = "moving";
        } else if (flow.info.last && flow.mode === "done") {
          flow.mode = "shirt";
          flow.shown = now;
        } else {
          flow.mode = "";
        }
      }
    }
  } else if (flow.mode === "arrived") {
    const onward = s.garment === flow.garment && !!step && step.index === flow.from + 1;
    if (!onward || now - flow.shown > FLOW_ARRIVED_MS) flow.mode = "";
  } else if (flow.mode === "shirt") {
    const here = s.garment === flow.garment && !!step && step.index === flow.from && pass;
    if (!here || now - flow.shown > FLOW_SHIRT_MS) flow.mode = "";
  }
  drawFlow(s, now);
}

function drawFlow(s, now) {
  // The step arrived at, said briefly over the picture.
  const toast = $("toast");
  if (flow.mode === "arrived") {
    const i = flow.info;
    const what = i.kind === "curve" ? "point at the seam"
               : i.kind === "distance" ? "tap the two points"
               : i.kind === "collar" ? "draw a box round the collar"
               : i.kind === "perimeter" ? "lay the first half"
               : "lay the fold against the rail";
    const text = "Step " + (flow.from + 1) + " of " + i.count + DOT + i.now + DOT + what;
    if (toast.hidden || toast.dataset.text !== text) {
      toast.dataset.text = text;
      toast.innerHTML = "";
      const a = document.createElement("span");
      a.textContent = ARROW;
      toast.appendChild(a);
      toast.appendChild(document.createTextNode(text));
      toast.hidden = false;
    }
  } else {
    toast.hidden = true;
  }

  // A finished shirt.
  const box = $("flow");
  if (flow.mode === "shirt") {
    const steps = (s && s.steps) || [];
    const passed = steps.filter((p) => p.state === "pass").length;
    const flaggedN = steps.filter((p) => p.manual_qc).length;
    const failed = steps.filter((p) => p.state === "fail" && !p.manual_qc).length;
    const open = steps.length - passed - failed - flaggedN;
    const notes = [];
    if (failed) notes.push(failed + " failed");
    if (flaggedN) notes.push(flaggedN + " for manual QC");
    if (open) notes.push(open + " not measured");
    notes.push("Press Next shirt for the next garment");
    // Green only when every step passed; red for a fail; amber for a shirt
    // that is not finished - a step unmeasured, or handed on to be measured
    // by hand.
    const tone = passed === steps.length ? "pass" : failed ? "fail" : "warn";
    $("shirtCard").className = "card shirtCard " + tone;
    // Finished only when every step has its verdict; that is also when the
    // shift's figures count it.
    $("shirtLabel").textContent = open ? "Shirt not finished" : "Shirt complete";
    if (open) notes.splice(notes.length - 1, 0, "not counted in the shift");
    $("shirtIcon").textContent = tone === "pass" ? TICK : "!";
    $("shirtMain").textContent = passed === steps.length ? "All " + steps.length + " steps passed"
                                                          : passed + " of " + steps.length + " steps passed";
    $("shirtSub").textContent = notes.join(DOT);
    box.hidden = false;
  } else {
    box.hidden = true;
  }
  if (s) drawBanner(s, now);
}

// The verdict band across the top of the picture, for as long as the verdict
// stands. A new verdict opens it out again if it had been folded down.
function drawBanner(s, now) {
  const banner = $("banner");
  const step = s.step, res = s.result;
  const calibrating = !!(s.calibration && s.calibration.active);
  if (!step || !res || res.step_index !== step.index || calibrating || flow.mode === "shirt") {
    banner.hidden = true;
    bannerState.shown = "";
    $("bValue").dataset.v = "";
    return;
  }
  const key = s.garment + ":" + step.index + "|" + verdictKey(res);
  if (key !== bannerState.key) {
    bannerState.key = key;
    bannerState.landed = now;
  }
  // A pass the station is about to move on from counts down on the band.
  const counting = (flow.mode === "done" || flow.mode === "moving") && flow.total > 0;
  // On a step drawn on - a box, a seam - the verdict lands full size, then
  // folds to a slim strip after a moment so the top of the picture is free
  // to draw on. It never takes a tap either way. Not while it counts down:
  // its button is the one way to go now that is on the picture.
  bannerState.mini = (pointable || boxable) && !counting &&
                     (now - bannerState.landed > 2500 || !!boxing || !!drawing);
  const passed = res.result === "PASS";
  const flagged = !!res.manual_qc;
  const mode = flagged ? "warn" : passed ? "pass" : "fail";

  let icon = passed ? TICK : flagged ? "!" : CROSS;
  let word = flagged ? "MANUAL QC" : res.result;
  let sub = nice(step.name);
  let value = "";
  let devText = "";
  if (res.measured) {
    value = fmt(res.measured_mm, 1);
    devText = signed(res.has_percent ? pct(res.deviation_pct)
                     : (res.deviation_mm >= 0 ? "+" : "") + fmt(res.deviation_mm, 2) + " mm");
  }
  if (flagged) sub += DOT + "measure by hand";
  else if (!passed && res.reason) sub += DOT + res.reason;
  let meter = null;
  if (counting) {
    // The seconds left, counted down from the moment the pass landed, by the
    // station's own clock (flow.start is kept to it).
    const left = Math.max(0, flow.total * 1000 - (now - flow.start));
    meter = Math.min(1, Math.max(0, 1 - left / (flow.total * 1000)));
    $("bCount").textContent = String(Math.ceil(left / 1000));
    $("bNext").textContent = "Next: " + (flow.info.next || "the next step") + DOT +
                             "step " + (flow.from + 1) + " of " + flow.info.count;
  }

  // Slides in when a verdict arrives; otherwise it stays exactly where it is.
  const sig = key + "|verdict";
  const arriving = banner.hidden || bannerState.shown !== sig;
  bannerState.shown = sig;
  banner.className = "banner " + mode + (bannerState.mini ? " mini" : "");
  banner.hidden = false;
  if (arriving && !reduced()) {
    void banner.offsetWidth;
    banner.classList.add("enter");
  }
  $("bIcon").textContent = icon;
  $("bWord").textContent = word;
  $("bSub").textContent = sub;
  const bv = $("bValue");
  if (value) {
    // Rebuilt for every new verdict, not only a new number: a retake that
    // reads the same tenth of a millimetre against a new target has another
    // deviation, and another verdict.
    const vkey = key + "|" + value + "|" + devText;
    if (bv.dataset.v !== vkey) {
      bv.dataset.v = vkey;
      bv.innerHTML = "";
      const b = document.createElement("b");
      b.textContent = value;
      const small = document.createElement("small");
      small.textContent = "mm";
      const d = document.createElement("div");
      d.textContent = devText + " off target";
      bv.appendChild(b);
      bv.appendChild(small);
      bv.appendChild(d);
      if (verdictWatch.freshKey === key) countUp(b, res.measured_mm, 1, true);
    }
    bv.hidden = false;
  } else {
    bv.dataset.v = "";
    bv.innerHTML = "";
    bv.hidden = true;
  }
  $("bGo").hidden = meter === null || bannerState.mini;
  if (meter !== null) $("bFill").style.width = Math.round(meter * 100) + "%";
}

// -- the big button --
function renderActions(s) {
  const step = s.step, res = s.result;
  const btn = $("btnPrimary");
  const moving = (flow.mode === "done" || flow.mode === "moving") && flow.total > 0;
  let does, text;
  if (!step) {
    does = "none"; text = "Choose a style to start";
  } else if (moving) {
    does = "advance"; text = "Next step now";
  } else if (collarStuck) {
    does = "redraw"; text = "Redraw the box around the collar";
  } else if (step.kind === "collar" && !collarStanding) {
    // The next thing on a collar step with no reading is drawn on the
    // picture, not pressed; skipping the collar is a deliberate "Skip step"
    // beside Flag, never the big button a finger is already on.
    does = "none"; text = "Draw a box around the collar";
  } else if (step.last && res) {
    does = "shirt"; text = "Next shirt";
  } else if (step.last) {
    // Nothing to press until the last step is read: one tap on a big "Next
    // shirt" would leave it unmeasured. Next shirt is still beside it.
    does = "none"; text = "Last step";
  } else {
    does = "next"; text = "Next step";
  }
  // A press on the big button while it was disabled cannot have been aimed
  // at what it says now, so becoming live is not a change to guard.
  if (does !== primaryDoes && primaryDoes !== "none") primaryChangedAt = Date.now();
  primaryDoes = does;
  btn.textContent = text;
  btn.disabled = does === "none";
  // On a collar step, the row for a collar the box could not read - Flag,
  // and skipping the step - has its place from the moment the step comes up,
  // shown or not, so nothing moves under a finger when a box is refused or
  // read. The row under it never changes: Next shirt keeps its place, only
  // unseen, while the big button says the same.
  const collarStep = !!(step && step.kind === "collar");
  const row = $("secondaryRow");
  row.hidden = !collarStep;
  // Once the collar has its reading the row has nothing to offer, and keeps
  // its place unseen.
  row.classList.toggle("reserve", collarStep && collarStanding);
  $("btnSecondary").disabled = !flagOffered;
  $("btnNext").disabled = collarStanding || !step || step.last;
  const shirtHere = does === "shirt";
  $("btnShirt").classList.toggle("reserve", shirtHere);
  $("btnShirt").disabled = shirtHere;
  $("btnBack").disabled = !step || step.index <= 1;
  $("btnRetake").disabled = !step;

  const dn = doNow(s);
  $("doNow").textContent = dn[0];
  $("doNow").className = "doNow" + (dn[1] ? " " + dn[1] : "");
}

// -- the admin drawer's view of the station --
function renderAdmin(s) {
  $("fps").textContent  = fmt(s.fps, 1);
  $("rmse").textContent = fmt(s.rmse_mm, 3);
  if (Date.now() - delayChangedAt > 1500 && s.advance_delay_s !== undefined) {
    $("delayValue").textContent = Math.round(s.advance_delay_s) + " s";
  }
  const box = $("manualSteps");
  if (Date.now() - manualChangedAt > 1500) box.checked = !!s.manual_steps;
  const hidden = s.hidden_steps || 0;
  $("manualNote").textContent = s.manual_steps
    ? "Shown. Switching restarts the shirt on the bench from step 1."
    : (hidden ? hidden + (hidden === 1 ? " step is" : " steps are") + " hidden in this style. " : "") +
      "Switching restarts the shirt on the bench from step 1.";
}

// -- the picture, as large as the feed allows --
//
// The stream is sent at whatever size the station renders it and shown as
// large as the feed can hold it, keeping its shape. The element is sized
// rather than the picture fitted inside it, so the element's box stays
// exactly the picture: a tap is measured against that box.
function fitVideo() {
  const img = $("video");
  if (img.hidden) return;
  const nw = img.naturalWidth, nh = img.naturalHeight;
  const f = $("feed");
  const fw = f.clientWidth, fh = f.clientHeight;
  // The banner's type is sized from the feed's width.
  if (fw > 0) f.style.setProperty("--fw", fw + "px");
  if (!(nw > 0 && nh > 0 && fw > 0 && fh > 0)) return;
  const k = Math.min(fw / nw, fh / nh);
  const w = Math.floor(nw * k), h = Math.floor(nh * k);
  if (img._w === w && img._h === h) return;
  img._w = w;
  img._h = h;
  img.style.width = w + "px";
  img.style.height = h + "px";
}
if (window.ResizeObserver) new ResizeObserver(() => fitVideo()).observe($("feed"));
window.addEventListener("resize", fitVideo);

// -- the shift --
//
// Kept by the station, so every screen on it shows the same figures; this
// only draws them. Three on the bar, the rest in the drawer.
function dur(v) {
  if (v === null || v === undefined || isNaN(v)) return "-";
  const t = Math.round(v);
  return t < 60 ? t + " s" : Math.floor(t / 60) + ":" + String(t % 60).padStart(2, "0");
}
const resultTone = (r) => r === "PASS" ? "pass" : r === "MANUAL QC" ? "warn" : "fail";
let statsShown = "";
function renderStats(s) {
  const k = s.stats;
  $("stats").hidden = !k;
  $("statsSec").hidden = !k;
  if (!k) return;
  const shown = JSON.stringify(k);
  if (shown === statsShown) return;
  statsShown = shown;
  const rate = k.per_hour === null || k.per_hour === undefined ? "-" : String(Math.round(k.per_hour));
  $("stShirts").textContent = k.shirts;
  $("stAvg").textContent = dur(k.avg_s);
  $("stRate").textContent = rate;
  $("stats").setAttribute("aria-label", "This shift: " + k.shirts + " shirts, " + dur(k.avg_s) +
                          " average, " + rate + " per hour. Open the figures.");
  $("stSince").textContent = k.since ? "since " + k.since.slice(0, 5) : "";
  $("sdShirts").textContent = k.shirts;
  // Coloured only when there is something to count, so a zero is quiet.
  const qc = k.manual_qc || 0;
  $("sdPassed").textContent = k.passed;
  $("sdPassedFact").className = "fact big" + (k.passed ? " ok" : "");
  $("sdFailed").textContent = k.failed;
  $("sdFailedFact").className = "fact big" + (k.failed ? " bad" : "");
  $("sdQc").textContent = qc;
  $("sdQcFact").className = "fact big" + (qc ? " no" : "");
  $("sdAvg").textContent = dur(k.avg_s);
  $("sdMedian").textContent = dur(k.median_s);
  $("sdRate").textContent = rate;
  $("sdIdle").textContent = k.idle;
  const last = k.last;
  $("sdLast").textContent = last ? dur(last.seconds) + (last.idle ? " idle" : "") : "-";
  $("sdLastLabel").textContent = last ? "last shirt, " + last.result : "last shirt";
  $("sdLastAt").textContent = last ? last.at.slice(0, 5) : "-";
  const list = $("sdRecent");
  list.innerHTML = "";
  for (const r of k.recent || []) {
    const row = document.createElement("div");
    const a = document.createElement("b");
    a.textContent = "Shirt " + r.shirt;
    const at = document.createElement("span");
    at.textContent = r.at.slice(0, 5);
    const d = document.createElement("span");
    d.textContent = dur(r.seconds) + (r.idle ? " idle" : "");
    const v = document.createElement("span");
    v.className = resultTone(r.result);
    v.textContent = r.result;
    row.appendChild(a);
    row.appendChild(at);
    row.appendChild(d);
    row.appendChild(v);
    list.appendChild(row);
  }
}
$("stats").onclick = (e) => {
  if (e.detail > 0) e.currentTarget.blur();
  showSettings(true);
  panelScrollTo($("statsSec"));
};
// A new shift clears the figures for good, so it asks twice - and a second
// tap hard on the heels of the first is a finger bouncing, not an answer.
let newShiftArmed = 0;
$("btnNewShift").onclick = async () => {
  const now = Date.now();
  const since = now - newShiftArmed;
  if (since < 600) return;
  if (since > 4000) {
    newShiftArmed = now;
    const k = lastStatus && lastStatus.stats;
    $("btnNewShift").textContent = "Tap again to clear " + (k ? k.shirts + (k.shirts === 1 ? " shirt" : " shirts") : "the figures");
    $("btnNewShift").classList.add("warnBtn");
    setTimeout(() => {
      if (Date.now() - newShiftArmed >= 4000) {
        $("btnNewShift").textContent = "New shift";
        $("btnNewShift").classList.remove("warnBtn");
      }
    }, 4100);
    return;
  }
  newShiftArmed = 0;
  $("btnNewShift").textContent = "New shift";
  $("btnNewShift").classList.remove("warnBtn");
  try { await fetch("/api/command?name=stats_new_shift", { method: "POST" }); } catch (e) {}
};

function render(s) {
  const fresh = watchVerdict(s);
  renderStyles(s.styles, s.style);
  renderStrip(s.steps);
  renderStep(s);
  renderPointing(s);
  renderCollar(s);
  renderPerimeter(s);
  renderResult(s, fresh);
  renderShot(s);
  renderMoving(s.advance_in_s);
  renderMarks(s);
  renderGlow(s, fresh);
  renderFlow(s);
  renderActions(s);
  renderCalibration(s);
  renderScale(s);
  renderAdmin(s);
  renderExposure(s);
  renderStats(s);
  renderLog(s);
  fitVideo();
}

async function poll() {
  let s;
  try {
    s = await (await fetch("/api/status", { cache: "no-store" })).json();
  } catch (e) {
    $("state").textContent = "no answer";
    $("lamp").title = "No answer from the station";
    $("lamp").className = "lamp fault";
    if (!$("calib").hidden && ($("calibBox").classList.contains("busy") || $("calibBox").classList.contains("wait"))) {
      $("calibStep").textContent = "No answer from the station";
      $("calibCount").textContent = "";
    }
    // Nor count down to a step a silent station is not going to show.
    flow.mode = "";
    $("flow").hidden = true;
    $("toast").hidden = true;
    $("banner").hidden = true;
    return;
  }
  lastStatus = s;

  const calibrating = !!(s.calibration && s.calibration.active);
  const state = calibrating ? "CALIBRATING" : (s.state || "?");
  const words = { WAITING: "ready", STABILISING: "settling", CAPTURING: "reading",
                  MEASURED_WAIT_FOR_CLEAR: "holding", CALIBRATING: "calibrating",
                  STARTING: "starting", FAULT: "fault" };
  $("state").textContent = words[state] || state.toLowerCase();
  $("lamp").className = "lamp " +
    (state === "FAULT" ? "fault" : calibrating ? "busy" : s.held ? "held" : s.ready ? "live" : "");

  if (s.ready && $("video").hidden) {
    $("video").src = "/stream.mjpg";
    $("video").hidden = false;
    $("videoWait").hidden = true;
  }
  if (state === "FAULT") {
    $("videoWait").hidden = false;
    $("videoWait").textContent = s.fault || "The station stopped";
    $("video").hidden = true;
  }

  $("barFill").style.width = Math.round((s.progress || 0) * 100) + "%";
  $("bar").className = "bar" + (s.held ? " held" : "");
  $("garment").textContent = s.garment ? ("Shirt " + s.garment) : "-";

  render(s);
}

poll();
setInterval(poll, 250);
</script>
</body>
</html>
)VLPAGE";

}  // namespace verimeasure
