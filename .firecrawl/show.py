import json, sys, glob, os
for f in sys.argv[1:]:
    for path in glob.glob(f):
        print("="*20, os.path.basename(path))
        try:
            d = json.load(open(path, encoding="utf-8"))
        except Exception as e:
            print("PARSE FAIL", e); continue
        data = d.get("data", d)
        web = data.get("web") if isinstance(data, dict) else None
        if web is None and isinstance(data, list):
            web = data
        for r in (web or [])[:10]:
            print("-", r.get("title"))
            print("  ", r.get("url"))
            desc = (r.get("description") or "")[:300].replace("\n", " ")
            if desc: print("   >", desc)
