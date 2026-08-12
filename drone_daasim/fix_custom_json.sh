#!/usr/bin/env bash
# Godot の custom.json のチャネルを master(webavatar.json) に整合させる/元に戻す。
#   apply   : Drone 等のチャネルを webavatar に合わせて remap（backup=custom.json.bak）
#   restore : custom.json.bak から復元
#   show    : 現在の Drone pos チャネルを表示
# multi-process（native=master）では pos=ch1 に整合が必要。
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
CMD="${1:-show}"
cd "$GODOT_DRONE"

case "$CMD" in
  apply)
    [ -f custom.json.bak ] || cp custom.json custom.json.bak
    "$PYENV_PY" - <<'PY'
import json
cust=json.load(open("custom.json")); web=json.load(open("webavatar.json"))
wmap={}
for r in web["robots"]:
    m={}
    for k in ("shm_pdu_readers","shm_pdu_writers","rpc_pdu_readers","rpc_pdu_writers"):
        for p in r.get(k,[]): m[p["org_name"]]=(p["channel_id"],p["pdu_size"])
    wmap[r["name"]]=m
n=0
for r in cust["robots"]:
    wm=wmap.get(r["name"],{})
    for k in ("shm_pdu_readers","shm_pdu_writers"):
        for p in r.get(k,[]):
            o=p["org_name"]
            if o in wm and (p.get("channel_id")!=wm[o][0] or p.get("pdu_size")!=wm[o][1]):
                print(f"  remap {r['name']}/{o}: ch {p.get('channel_id')}->{wm[o][0]} sz {p.get('pdu_size')}->{wm[o][1]}")
                p["channel_id"],p["pdu_size"]=wm[o]; n+=1
json.dump(cust,open("custom.json","w"),indent=2,ensure_ascii=False)
print("remaps:",n)
PY
    ;;
  restore)
    [ -f custom.json.bak ] && mv -f custom.json.bak custom.json && echo "restored custom.json" || echo "no backup"
    ;;
  show)
    "$PYENV_PY" -c "import json;d=json.load(open('custom.json'));[print(r['name'],p['org_name'],'ch',p['channel_id'],'sz',p['pdu_size']) for r in d['robots'] if r['name']=='Drone' for p in r.get('shm_pdu_readers',[])]"
    ;;
  *) echo "usage: $0 apply|restore|show";;
esac
