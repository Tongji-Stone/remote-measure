// Tavily -> Anthropic Messages 翻译层
//
// 背景：DSH 的网页搜索插件（dsh-web-search-deepseek）只会说一种话：
//   POST <baseURL>/messages   { model, max_tokens, messages, tools:[{type:"web_search_20250305"}] }
// 并期望回来这样的东西：
//   { content: [ {type:"web_search_tool_result", content:[{type:"web_search_result",url,title,page_age}]},
//                 {type:"text", text:"...", citations:[{url, cited_text}]} ] }
//
// Tavily 不会说这种话，它的接口是 POST https://api.tavily.com/search
// 这个小服务夹在中间做翻译：把插件的请求翻成 Tavily 的调用，再把 Tavily 的结果翻回插件要的格式。
//
// 关键设计：Tavily 的钥匙直接从插件发来的 x-api-key / Authorization 头里取，
// 所以这个文件里不存任何密钥。

import http from 'node:http';

const PORT = Number(process.env.SHIM_PORT || 8787);
const TAVILY_URL = 'https://api.tavily.com/search';
const MAX_RESULTS_CAP = 10;

/** 从插件的请求里把真正的搜索词抠出来。 */
function extractQuery(body) {
  const texts = [];
  for (const m of body.messages ?? []) {
    if (typeof m.content === 'string') texts.push(m.content);
    else if (Array.isArray(m.content)) {
      for (const b of m.content) if (b?.type === 'text' && typeof b.text === 'string') texts.push(b.text);
    }
  }
  const joined = texts.join('\n');
  const m = joined.match(/Perform a web search for the query:\s*([\s\S]*)$/i);
  return (m ? m[1] : joined).trim();
}

/** 从插件请求的头里取 Tavily 钥匙。 */
function extractKey(req) {
  const auth = req.headers['authorization'];
  if (typeof auth === 'string' && auth.toLowerCase().startsWith('bearer ')) {
    const k = auth.slice(7).trim();
    if (k) return k;
  }
  const x = req.headers['x-api-key'];
  if (typeof x === 'string' && x.trim()) return x.trim();
  return '';
}

function readBody(req) {
  return new Promise((resolve, reject) => {
    const chunks = [];
    let size = 0;
    req.on('data', (c) => {
      size += c.length;
      if (size > 2_000_000) { reject(new Error('request body too large')); req.destroy(); return; }
      chunks.push(c);
    });
    req.on('end', () => resolve(Buffer.concat(chunks).toString('utf8')));
    req.on('error', reject);
  });
}

/** 把 Tavily 的结果翻译成插件要的 Anthropic 形状。 */
function toAnthropicShape(query, tavily) {
  const results = Array.isArray(tavily?.results) ? tavily.results : [];
  const items = [];
  const citations = [];

  for (const r of results) {
    const url = typeof r?.url === 'string' ? r.url : '';
    if (!url) continue;
    const title = typeof r?.title === 'string' ? r.title : '';
    const snippet = typeof r?.content === 'string' ? r.content : '';

    const item = { type: 'web_search_result', url, title };
    // 插件只认 page_age 是字符串的情况，Tavily 一般不给发布时间，没给就不填。
    if (typeof r?.published_date === 'string' && r.published_date) item.page_age = r.published_date;
    items.push(item);

    // 摘要是分开走的：插件从 text 块的 citations[].cited_text 里取，
    // 按 url 配对，所以这里必须 url 对得上。
    if (snippet) {
      citations.push({
        type: 'web_search_result_location',
        url,
        title,
        cited_text: snippet.slice(0, 1200),
      });
    }
  }

  // 顺便把人工可读的答案也带上，插件不用，但便于直接 curl 调试。
  const answer = typeof tavily?.answer === 'string' ? tavily.answer : '';

  const content = [
    { type: 'web_search_tool_result', tool_use_id: 'shim', content: items },
    {
      type: 'text',
      text: answer || `Tavily returned ${items.length} result(s) for: ${query}`,
      ...(citations.length ? { citations } : {}),
    },
  ];

  return {
    id: 'msg_shim',
    type: 'message',
    role: 'assistant',
    model: 'tavily-shim',
    stop_reason: 'end_turn',
    stop_sequence: null,
    usage: { input_tokens: 0, output_tokens: 0 },
    content,
  };
}

const server = http.createServer(async (req, res) => {
  const send = (code, obj) => {
    const payload = JSON.stringify(obj);
    res.writeHead(code, { 'content-type': 'application/json', 'content-length': Buffer.byteLength(payload) });
    res.end(payload);
  };

  const url = new URL(req.url || '/', 'http://localhost');
  if (req.method === 'GET' && (url.pathname === '/health' || url.pathname === '/')) {
    return send(200, { ok: true, service: 'tavily-anthropic-shim', port: PORT });
  }
  if (!(req.method === 'POST' && url.pathname.replace(/\/$/, '').endsWith('/messages'))) {
    return send(404, { type: 'error', error: { type: 'not_found_error', message: `no route for ${req.method} ${url.pathname}` } });
  }

  try {
    const raw = await readBody(req);
    let body = {};
    try { body = JSON.parse(raw); } catch { /* 交给下面报错 */ }

    const query = extractQuery(body);
    const key = extractKey(req);
    if (!key) {
      return send(401, { type: 'error', error: { type: 'authentication_error', message: 'shim: no API key on the request (looked at authorization / x-api-key)' } });
    }
    if (!query) {
      return send(400, { type: 'error', error: { type: 'invalid_request_error', message: 'shim: could not extract a search query from messages' } });
    }

    const maxUses = Number(body?.tools?.[0]?.max_uses);
    const maxResults = Number.isFinite(maxUses) && maxUses > 0 ? Math.min(maxUses, MAX_RESULTS_CAP) : 5;

    const tavilyRes = await fetch(TAVILY_URL, {
      method: 'POST',
      headers: { authorization: `Bearer ${key}`, 'content-type': 'application/json' },
      body: JSON.stringify({
        query,
        max_results: maxResults,
        search_depth: 'advanced',
        include_answer: true,
        include_raw_content: false,
      }),
      signal: AbortSignal.timeout(45_000),
    });

    const text = await tavilyRes.text();
    if (!tavilyRes.ok) {
      // 把 Tavily 的错误原样翻译成 Anthropic 的风格，方便插件把它显示出来。
      return send(tavilyRes.status, {
        type: 'error',
        error: { type: 'api_error', message: `Tavily HTTP ${tavilyRes.status}: ${text.slice(0, 500)}` },
      });
    }
    let parsed;
    try { parsed = JSON.parse(text); } catch {
      return send(502, { type: 'error', error: { type: 'api_error', message: `Tavily returned non-JSON: ${text.slice(0, 300)}` } });
    }

    console.log(`[shim] q=${JSON.stringify(query.slice(0, 80))} results=${parsed?.results?.length ?? 0}`);
    return send(200, toAnthropicShape(query, parsed));
  } catch (err) {
    const aborted = err?.name === 'TimeoutError' || err?.name === 'AbortError';
    return send(aborted ? 504 : 500, {
      type: 'error',
      error: { type: aborted ? 'timeout_error' : 'api_error', message: `shim: ${String(err)}` },
    });
  }
});

server.listen(PORT, '127.0.0.1', () => {
  console.log(`[shim] listening on http://127.0.0.1:${PORT}  ->  POST /v1/messages -> Tavily`);
});
